/*
    VGM writing
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <wchar.h>
#include <time.h>

#include "vgm.h"
#include "fileio.h"

// has to be larger than ~20MB
#define VGM_BUFFER 50000000
#define VGM_MIDI_CHANNELS 8

    uint32_t buffer_size;
    uint32_t delayq;
    uint32_t samplecnt;
    uint32_t loop_set;
    uint8_t* vgmdata;
    uint8_t* data;
    char* filename;
    FILE* note_log;
    int note_log_dirty;
    char note_log_notes[32][4];
    uint8_t* miditrack;
    uint32_t midi_buffer_size;
    uint32_t midi_track_size;
    uint32_t midi_delta;
    uint32_t midi_delta_rem;
    uint8_t midi_note[32];
    uint8_t midi_note_active[32];

extern const char* Q_NoteNames[12];

// Increments destination pointer
void my_memcpy(uint8_t** dest, void* src, int size)
{
    memcpy(*dest,src,size);
    *dest += size;
}

void add_datablockcmd(uint8_t** dest, uint8_t dtype, uint32_t size, uint32_t romsize, uint32_t offset)
{
    **dest = 0x67;*dest+=1;
    **dest = 0x66;*dest+=1;
    **dest = dtype;*dest+=1;
    size += 8;
    my_memcpy(dest,&size,4);
    my_memcpy(dest,&romsize,4);
    my_memcpy(dest,&offset,4);
}

static void midi_write_byte(uint8_t value)
{
    if(midi_track_size >= midi_buffer_size)
    {
        uint8_t* temp;
        temp = realloc(miditrack,midi_buffer_size*2);
        if(!temp)
            return;
        miditrack = temp;
        midi_buffer_size *= 2;
    }
    miditrack[midi_track_size++] = value;
}

static void midi_write_varlen(uint32_t value)
{
    uint32_t buffer = value & 0x7f;
    while((value >>= 7))
    {
        buffer <<= 8;
        buffer |= ((value & 0x7f) | 0x80);
    }
    while(1)
    {
        midi_write_byte(buffer & 0xff);
        if(buffer & 0x80)
            buffer >>= 8;
        else
            break;
    }
}

static uint8_t midi_note_from_log_note(uint8_t note)
{
    int midi_note_value = note + 9;
    if(midi_note_value > 127)
        midi_note_value = 127;
    return midi_note_value;
}

static uint8_t midi_channel_from_log_channel(int channel)
{
    return channel % VGM_MIDI_CHANNELS;
}

static void midi_write_event(uint8_t status, uint8_t data1, uint8_t data2)
{
    if(!miditrack)
        return;
    midi_write_varlen(midi_delta);
    midi_write_byte(status);
    midi_write_byte(data1);
    midi_write_byte(data2);
    midi_delta = 0;
}

static void midi_add_delay(int samples)
{
    uint32_t ticks;
    if(!miditrack || samples <= 0)
        return;
    midi_delta_rem += (uint32_t)samples * 960;
    ticks = midi_delta_rem / 44100;
    midi_delta_rem %= 44100;
    midi_delta += ticks;
}

static void midi_open(void)
{
    midi_buffer_size = 0x10000;
    midi_track_size = 0;
    midi_delta = 0;
    midi_delta_rem = 0;
    memset(midi_note,0,sizeof(midi_note));
    memset(midi_note_active,0,sizeof(midi_note_active));
    miditrack = malloc(midi_buffer_size);
    if(!miditrack)
        return;

    /* tempo: 120 bpm */
    midi_write_varlen(0);
    midi_write_byte(0xff);
    midi_write_byte(0x51);
    midi_write_byte(0x03);
    midi_write_byte(0x07);
    midi_write_byte(0xa1);
    midi_write_byte(0x20);
}

static void midi_note_off_all(void)
{
    int i;
    for(i=0;i<32;i++)
    {
        if(midi_note_active[i])
        {
            midi_write_event(0x80 | midi_channel_from_log_channel(i),midi_note[i],0);
            midi_note_active[i] = 0;
        }
    }
}

static void midi_close(void)
{
    char* midiname;
    char* ext;
    FILE* midifile;
    uint32_t len;

    if(!miditrack)
        return;

    midi_note_off_all();
    midi_write_varlen(midi_delta);
    midi_write_byte(0xff);
    midi_write_byte(0x2f);
    midi_write_byte(0x00);
    midi_delta = 0;

    midiname = (char*)malloc(strlen(filename)+10);
    strcpy(midiname,filename);
    ext = strrchr(midiname,'.');
    if(ext != NULL)
        strcpy(ext,".mid");
    else
        strcat(midiname,".mid");

    midifile = fopen(midiname,"wb");
    if(midifile)
    {
        fwrite("MThd",1,4,midifile);
        fputc(0x00,midifile); fputc(0x00,midifile); fputc(0x00,midifile); fputc(0x06,midifile);
        fputc(0x00,midifile); fputc(0x00,midifile);
        fputc(0x00,midifile); fputc(0x01,midifile);
        fputc(0x01,midifile); fputc(0xe0,midifile);
        fwrite("MTrk",1,4,midifile);
        len = midi_track_size;
        fputc((len>>24)&0xff,midifile);
        fputc((len>>16)&0xff,midifile);
        fputc((len>>8)&0xff,midifile);
        fputc(len&0xff,midifile);
        fwrite(miditrack,1,midi_track_size,midifile);
        fclose(midifile);
    }
    free(midiname);
    free(miditrack);
    miditrack = NULL;
}

static void vgm_note_log_write_header(void)
{
    int i;
    if(!note_log)
        return;
    for(i=0;i<32;i++)
        fprintf(note_log,"Channel %-9d%s",i+1,(i == 31) ? "\n" : " ");
}

static void vgm_note_log_flush(void)
{
    int i;
    if(!note_log || !note_log_dirty)
        return;
    for(i=0;i<32;i++)
        fprintf(note_log,"%-17s%s",note_log_notes[i],(i == 31) ? "\n" : " ");
    note_log_dirty = 0;
}

void add_delay(uint8_t** dest, int delay)
{
    if(delay > 0)
    {
        vgm_note_log_flush();
        midi_add_delay(delay);
    }
    samplecnt += delay;

    int commandcount = floor(delay/65535);
    uint16_t finalcommand = delay%65535;

    while(commandcount)
    {
        **dest = 0x61;*dest+=1;
        **dest = 0xff;*dest+=1;
        **dest = 0xff;*dest+=1;
        commandcount--;
    }

    if(finalcommand > 16)
    {
        **dest = 0x61;*dest+=1;
        my_memcpy(dest,&finalcommand,2);
    }
    else if(finalcommand > 0)
    {
        **dest = 0x70 + finalcommand-1;
        *dest+=1;
    }
}

void vgm_open(char* fname)
{
    filename = (char*)malloc(strlen(fname)+10);
    strcpy(filename,fname);
    samplecnt=0;
    delayq=0;
    loop_set=0;
    note_log = NULL;
    note_log_dirty = 0;
    memset(note_log_notes, 0, sizeof(note_log_notes));
    miditrack = NULL;
    midi_open();

    {
        char* txtname = (char*)malloc(strlen(fname)+10);
        char* ext;
        strcpy(txtname,fname);
        ext = strrchr(txtname,'.');
        if(ext != NULL)
            strcpy(ext,".txt");
        else
            strcat(txtname,".txt");
        note_log = fopen(txtname,"w");
        if(note_log)
            vgm_note_log_write_header();
        free(txtname);
    }

    // create initial buffer
    vgmdata=(uint8_t*)malloc(VGM_BUFFER);
    data = vgmdata;
    buffer_size = VGM_BUFFER;
    memset(data, 0, VGM_BUFFER);

    // vgm magic
    memcpy(data, "Vgm ", 4);

    // version
    data+=8;
    *data++ = 0x71;
    *data++ = 0x01;

    //data offset
    *(uint32_t*)(vgmdata+0x34)=0x100-0x34;

    data=vgmdata+0x100;
}

void vgm_poke32(int32_t offset, uint32_t d)
{
    *(uint32_t*)(vgmdata+offset)= d;
}

void vgm_poke8(int32_t offset, uint8_t d)
{
    *(uint8_t*)(vgmdata+offset)= d;
}

// notice: start offset was replaced with ROM mask.
void vgm_datablock(uint8_t dbtype, uint32_t dbsize, uint8_t* datablock, uint32_t maxsize, uint32_t mask, int32_t flags)
{
    add_datablockcmd(&data, dbtype, dbsize|flags, maxsize, 0);

    int i;
    for(i=0;i<dbsize;i++)
        *data++ = datablock[i & mask];

    //my_memcpy(&data, datablock, dbsize);
}

void vgm_setloop()
{
    // add delays
    if(delayq/10 > 1)
    {
        add_delay(&data,delayq/10);
        delayq=delayq%10;
    }

    loop_set = samplecnt;
    *(uint32_t*)(vgmdata+0x1c)= data-vgmdata-0x1c;
}

void vgm_write(uint8_t command, uint8_t port, uint16_t reg, uint16_t value)
{
    if(delayq/10 > 1)
    {
        add_delay(&data,delayq/10);
        delayq=delayq%10;
    }

// todo: need to handle command types if using other chips
    *data++ = command;

    if(command == 0xe1) // C352
    {
        *data++ = reg>>8;
        *data++ = reg&0xff;
        *data++ = value>>8;
        *data++ = value&0xff;
    }
    else if(command == 0x54) // YM2151
    {
        *data++ = reg;
        *data++ = value;
    }
    else // following is for D0-D6 commands...
    {
        *data++ = port;
        *data++ = (reg&0xff);
        *data++ = (value&0xff);
    }

    // resize buffer if needed
    if(buffer_size-(data-vgmdata) < 1000000)
    {
        uint8_t* temp;
        temp = realloc(vgmdata,buffer_size*2);
        if(temp)
        {
            buffer_size *= 2;
            data = temp+(data-vgmdata);
            vgmdata = temp;
        }
    }
}

// delay is in VGM samples*10.
void vgm_delay(uint32_t delay)
{
    delayq+=delay;
}

void vgm_note_on(int channel, uint8_t note)
{
    int octave;
    uint8_t midi_note_value;
    if(channel < 0 || channel >= 32)
        return;
    midi_note_value = midi_note_from_log_note(note);
    octave = (note-3)/12;
    note %= 12;
    snprintf(note_log_notes[channel],sizeof(note_log_notes[channel]),"%s%d",Q_NoteNames[note],octave);
    note_log_dirty = 1;
    if(midi_note_active[channel] && midi_note[channel] == midi_note_value)
        return;
    if(midi_note_active[channel])
        midi_write_event(0x80 | midi_channel_from_log_channel(channel),midi_note[channel],0);
    midi_write_event(0x90 | midi_channel_from_log_channel(channel),midi_note_value,100);
    midi_note[channel] = midi_note_value;
    midi_note_active[channel] = 1;
}

void vgm_note_from_c352(int channel, uint16_t freq)
{
    int note;
    if(freq == 0)
        return;
    note = (int)floor((12.0 * log((double)freq / 0x88) / log(2.0)) + 0.5);
    if(note < 0)
        note = 0;
    if(note > 127)
        note = 127;
    vgm_note_on(channel,(uint8_t)note);
}

void vgm_note_off(int channel)
{
    if(channel < 0 || channel >= 32)
        return;
    strcpy(note_log_notes[channel],"---");
    note_log_dirty = 1;
    if(midi_note_active[channel])
    {
        midi_write_event(0x80 | midi_channel_from_log_channel(channel),midi_note[channel],0);
        midi_note_active[channel] = 0;
    }
}

// https://github.com/cppformat/cppformat/pull/130/files
void gd3_write_string(char* s)
{
    size_t l;
    #if defined(_WIN32) && defined(__MINGW32__) && !defined(__NO_ISOCEXT)
        l = _snwprintf((wchar_t*)data,256,L"%S", s);
    #else
        l = swprintf((wchar_t*)data,256,L"%s", s);
    #endif // defined

    data += (l+1)*2;
}

void vgm_write_tag(char* gamename,int songid)
{
    time_t t;
    struct tm * tm;
    time(&t);
    tm = localtime(&t);
    char ts [32];
    strftime(ts,32,"%Y-%m-%d %H:%M:%S",tm);
    char tracknotes[256];
    tracknotes[0] = 0;
    if(songid >= 0)
        sprintf(tracknotes,"Song ID: %03x\n",songid&0x7ff);
    strcpy(tracknotes+strlen(tracknotes),"Generated using QuattroPlay by ctr (Built "__DATE__" "__TIME__")");

    // Tag offset
    *(uint32_t*)(vgmdata+0x14)= data-vgmdata-0x14;

    memcpy(data, "Gd3 \x00\x01\x00\x00" , 8);
    uint8_t* len_s = data+8;
    data+=12;

    gd3_write_string(""); // Track name
    gd3_write_string(""); // Track name (native)
    gd3_write_string(gamename); // Game name
    gd3_write_string(""); // Game name (native)
    gd3_write_string("Arcade Machine"); // System name
    gd3_write_string(""); // System name (native)
    gd3_write_string(""); // Author name
    gd3_write_string(""); // Author name (native)
    gd3_write_string(ts); // Time
    gd3_write_string(""); // Pack author
    gd3_write_string(tracknotes); // Notes

    *(uint32_t*)(len_s) = data-len_s-4;        // length
}

void vgm_stop()
{
    if(delayq/10 > 1)
    {
        add_delay(&data,delayq/10);
        delayq=0;
    }
    *data++ = 0x66;

    // Sample count/loop sample count
    *(uint32_t*)(vgmdata+0x18)= samplecnt;
    if(loop_set)
        *(uint32_t*)(vgmdata+0x20)= samplecnt-loop_set;
}

void vgm_close()
{
    vgm_note_log_flush();
    // EoF offset
    *(uint32_t*)(vgmdata+0x04)= data-vgmdata-4;

    write_file(filename, vgmdata, data-vgmdata);

    if(note_log)
    {
        fclose(note_log);
        note_log = NULL;
    }
    midi_close();

    free(vgmdata);
    free(filename);
}
