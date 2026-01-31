/*
    MIDI writing (replaces previous VGM writer)

    This file keeps the original external API (vgm_open, vgm_write, vgm_delay, vgm_close, ...)
    but the internals now produce a Standard MIDI File (SMF, single-track).
    The mapping from the previous VGM-style parameters to MIDI is intentionally simple:

    - vgm_write(command, port, reg, value)
        * If `command` is a MIDI status like 0x90 (Note On), 0x80 (Note Off),
          0xB0 (Control Change), 0xC0 (Program Change), 0xE0 (Pitch Bend),
          the write will produce the corresponding MIDI message.
        * `port` is interpreted as MIDI channel (0-15).
        * `reg` and `value` provide message data (note number/controller number/LSB/MSB etc.)
        * Unknown commands are ignored (no-op).

    - vgm_delay(delay)
        * `delay` uses the same units as before (the original code accumulated
          delayq as "VGM samples * 10" and then used delayq/10 when writing).
        * For MIDI output we use the same approach: accumulated delay is converted
          to delta-time units by dividing by 10 (so 1 VGM sample => 1 tick).
        * The file's time division (ticks per quarter note) is set to 480; since we
          don't set tempo events, timings are relative: the mapping preserves
          relative timing between events.

    Implementation notes:
    - Produces a single-track (format 0) MIDI file.
    - Writes a placeholder track length and patches it on vgm_close().
    - Uses the existing write_file(...) helper from fileio.h (unchanged).
    - Buffer auto-expands with realloc() like the original implementation.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "vgm.h"
#include "fileio.h"

// initial buffer: keep same strategy (can grow)
#define MIDI_BUFFER 50000000

uint32_t buffer_size;
uint32_t delayq;           // accumulated delay (same semantics as original: VGM samples*10)
uint8_t* vgmdata;
uint8_t* data;
char* filename;

// Track bookkeeping
uint8_t* track_len_ptr = NULL;     // pointer to 4-byte track length field to patch later
uint8_t* track_start_ptr = NULL;   // start of track data (first byte after length)

// Internal helpers

// Write a single byte into data buffer
static inline void write_u8(uint8_t v)
{
    *data++ = v;
    // Grow if needed
    if (buffer_size - (data - vgmdata) < 1024) {
        uint8_t* temp = realloc(vgmdata, buffer_size * 2);
        if (temp) {
            size_t offset = data - vgmdata;
            buffer_size *= 2;
            vgmdata = temp;
            data = vgmdata + offset;
        }
    }
}

static inline void write_bytes(const uint8_t* src, size_t len)
{
    memcpy(data, src, len);
    data += len;
    if (buffer_size - (data - vgmdata) < 1024) {
        size_t offset = data - vgmdata;
        uint8_t* temp = realloc(vgmdata, buffer_size * 2);
        if (temp) {
            buffer_size *= 2;
            vgmdata = temp;
            data = vgmdata + offset;
        }
    }
}

// Write a big-endian 32-bit value
static inline void write_be32(uint32_t v)
{
    write_u8((v >> 24) & 0xFF);
    write_u8((v >> 16) & 0xFF);
    write_u8((v >> 8) & 0xFF);
    write_u8((v >> 0) & 0xFF);
}

// Write a big-endian 16-bit value
static inline void write_be16(uint16_t v)
{
    write_u8((v >> 8) & 0xFF);
    write_u8((v >> 0) & 0xFF);
}

// Write MIDI variable length quantity
static void write_varlen(uint32_t value)
{
    // Collect bytes in a temporary buffer (max 5 bytes for 32-bit)
    uint8_t buffer[5];
    int idx = 0;
    buffer[idx++] = value & 0x7F;
    value >>= 7;
    while (value) {
        buffer[idx++] = 0x80 | (value & 0x7F);
        value >>= 7;
    }
    // Write in reverse order
    for (int i = idx - 1; i >= 0; --i) {
        write_u8(buffer[i]);
    }
}

// Public API (names kept to minimize changes elsewhere)

// Open (create) a new MIDI file buffer
void vgm_open(char* fname)
{
    // store filename (ensure .mid extension)
    size_t flen = strlen(fname);
    size_t alloc_len = flen + 8;
    filename = (char*)malloc(alloc_len);
    if (!filename) return;
    strcpy(filename, fname);

    // If filename doesn't end with .mid, append .mid
    if (flen < 4 || strcmp(filename + flen - 4, ".mid") != 0) {
        strcat(filename, ".mid");
    }

    delayq = 0;

    // allocate buffer
    vgmdata = (uint8_t*)malloc(MIDI_BUFFER);
    if (!vgmdata) return;
    data = vgmdata;
    buffer_size = MIDI_BUFFER;
    memset(vgmdata, 0, buffer_size);

    // MIDI header chunk: MThd
    // Chunk id
    write_bytes((const uint8_t*)"MThd", 4);
    // Header length = 6
    write_be32(6);
    // Format 0 (single track)
    write_be16(0);
    // Number of tracks = 1
    write_be16(1);
    // Division (ticks per quarter note), choose 480 TPQN
    write_be16(480);

    // Start Track chunk: MTrk with placeholder length
    write_bytes((const uint8_t*)"MTrk", 4);
    // Placeholder for 4-byte length; remember pointer to patch later
    track_len_ptr = data;
    // write zero for now
    write_be32(0);
    // mark start of actual track data
    track_start_ptr = data;
}

// A simple helper for converting accumulated delay (same units as previous code)
// into delta-time for MIDI. The original code used delayq in "VGM samples*10"
// and often called add_delay(&data, delayq/10). We'll use the same mapping:
// delta_ticks = delayq / 10  (1 VGM sample -> 1 MIDI tick).
static inline uint32_t flush_delay_get_ticks()
{
    uint32_t ticks = delayq / 10;
    delayq = 0;
    return ticks;
}

// Write a MIDI event respecting the accumulated delay (delta-time).
// The function signature mirrors the original vgm_write so existing callers
// require minimal changes. Mapping notes:
//  - command 0x90: Note On
//  - command 0x80: Note Off
//  - command 0xB0: Control Change
//  - command 0xC0: Program Change
//  - command 0xE0: Pitch Bend (14-bit from `value`)
//  - command 0xF0: SysEx start (value = length, data bytes follow via vgm_datablock in original; here ignored)
// Unknown commands are currently ignored.
void vgm_write(uint8_t command, uint8_t port, uint16_t reg, uint16_t value)
{
    // flush accumulated delay before writing this event
    if (delayq >= 10) {
        uint32_t ticks = flush_delay_get_ticks();
        write_varlen(ticks);
    } else {
        // zero delta-time
        write_varlen(0);
    }

    uint8_t status = command & 0xF0;
    uint8_t channel = port & 0x0F;

    if (status == 0x90) {
        // Note On: reg = note, value = velocity
        write_u8(0x90 | channel);
        write_u8(reg & 0x7F);
        write_u8(value & 0x7F);
    }
    else if (status == 0x80) {
        // Note Off
        write_u8(0x80 | channel);
        write_u8(reg & 0x7F);
        write_u8(value & 0x7F);
    }
    else if (status == 0xB0) {
        // Control Change: reg = controller, value = controller value
        write_u8(0xB0 | channel);
        write_u8(reg & 0x7F);
        write_u8(value & 0x7F);
    }
    else if (status == 0xC0) {
        // Program Change: value = program number
        write_u8(0xC0 | channel);
        write_u8(value & 0x7F);
    }
    else if (status == 0xE0) {
        // Pitch Bend: 14-bit value (LSB, MSB)
        uint16_t bend = value & 0x3FFF;
        uint8_t lsb = bend & 0x7F;
        uint8_t msb = (bend >> 7) & 0x7F;
        write_u8(0xE0 | channel);
        write_u8(lsb);
        write_u8(msb);
    }
    else if (command == 0xFF) {
        // Meta-events: interpret reg as meta type and value as length/data if simple
        // Here we support End of Track (0x2F) if requested by caller.
        uint8_t meta_type = reg & 0xFF;
        if (meta_type == 0x2F) {
            // End of Track
            write_u8(0xFF);
            write_u8(0x2F);
            write_u8(0x00);
        } else {
            // unsupported meta event: no-op
        }
    }
    else if (command == 0xF0 || command == 0xF7) {
        // SysEx: Not implemented here (would require caller to pass actual sysex bytes).
        // No-op for now.
    }
    else {
        // Unknown command: no-op (preserves timing but writes no MIDI message)
    }
}

// Add to the accumulated delay (same semantics as original: delay in VGM samples*10)
void vgm_delay(uint32_t delay)
{
    delayq += delay;
}

// Convenience: write a 32-bit poke (kept for compatibility, no effect for MIDI)
void vgm_poke32(int32_t offset, uint32_t d)
{
    // No-op for MIDI-based writer, but preserved to avoid breaking callers.
    (void)offset;
    (void)d;
}

void vgm_poke8(int32_t offset, uint8_t d)
{
    (void)offset;
    (void)d;
}

// Datablocks and loop markers from VGM are not directly applicable to MIDI.
// Provide stubbed implementations to preserve API.
void vgm_datablock(uint8_t dbtype, uint32_t dbsize, uint8_t* datablock, uint32_t maxsize, uint32_t mask, int32_t flags)
{
    (void)dbtype; (void)dbsize; (void)datablock; (void)maxsize; (void)mask; (void)flags;
    // No-op for MIDI; sysex might be implemented here if needed in the future.
}

void vgm_setloop()
{
    // MIDI loop points must be handled with sequencer-specific meta/data; no-op here.
}

// Write a simple GD3-like tag as a text meta-event (optional).
// The original VGM GD3 block is not used for MIDI; we offer a small helper that appends
// a text meta event with the provided name and timestamp.
void vgm_write_tag(char* gamename, int songid)
{
    // flush any pending delay before inserting tag
    if (delayq >= 10) {
        uint32_t ticks = flush_delay_get_ticks();
        write_varlen(ticks);
    } else {
        write_varlen(0);
    }

    // Build a textual tag with timestamp and optional song id
    time_t t = time(NULL);
    struct tm* tm = localtime(&t);
    char ts[64];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm);
    char buf[512];
    if (songid >= 0) {
        snprintf(buf, sizeof(buf), "%s — Song ID: %03x — Generated: %s", gamename ? gamename : "", songid & 0x7FF, ts);
    } else {
        snprintf(buf, sizeof(buf), "%s — Generated: %s", gamename ? gamename : "", ts);
    }

    size_t len = strlen(buf);
    // Write a Text meta event (0x01) containing the tag as ASCII
    write_u8(0xFF);
    write_u8(0x01);
    write_varlen((uint32_t)len);
    write_bytes((const uint8_t*)buf, len);
}

// Finish the MIDI file: write End of Track and patch the track length, then save to disk.
void vgm_close()
{
    // Flush any remaining delay
    if (delayq >= 10) {
        uint32_t ticks = flush_delay_get_ticks();
        write_varlen(ticks);
    } else {
        write_varlen(0);
    }

    // Write End of Track meta event
    write_u8(0xFF);
    write_u8(0x2F);
    write_u8(0x00);

    // Patch track length: length = current_data_end - track_start_ptr
    if (track_len_ptr && track_start_ptr) {
        uint32_t track_length = (uint32_t)(data - track_start_ptr);
        // Write big-endian length into the placeholder
        track_len_ptr[0] = (track_length >> 24) & 0xFF;
        track_len_ptr[1] = (track_length >> 16) & 0xFF;
        track_len_ptr[2] = (track_length >> 8) & 0xFF;
        track_len_ptr[3] = (track_length >> 0) & 0xFF;
    }

    // Write buffer to file
    size_t total_size = (size_t)(data - vgmdata);
    write_file(filename, vgmdata, total_size);

    // Free buffer and filename
    free(vgmdata);
    free(filename);
    vgmdata = NULL;
    data = NULL;
    filename = NULL;
}
