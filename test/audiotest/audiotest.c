/*
 * audiotest: play, or write to an AIFF, a sine tone at a chosen rate, channel
 * count and sample width. Build on IRIX with irix.sh (MIPSpro cc, -laudio).
 *
 *   audiotest [-r RATE] [-c 1|2] [-b 8|16] [-s SECONDS] [-f HZ] [-n] [-w FILE.aiff]
 *
 * Plays through the audio library the way a game does: alSetParams sets the
 * output rate on the default output device, then a port writes the frames.
 * -n leaves the device rate alone (what a program that never sets one hears).
 * -w writes the same tone as an AIFF instead, for playaiff and sfplay.
 * Prints the rate the device reports after the set.
 */
#include <dmedia/audio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CHUNK 2048

static void put32(unsigned char *p, unsigned v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static void put16(unsigned char *p, unsigned v) { p[0] = v >> 8; p[1] = v; }

/* The sample rate as AIFF stores it: an IEEE 754 80-bit extended float. */
static void put_ext80(unsigned char *p, unsigned rate)
{
    int e = 0;
    unsigned m = rate;
    while (!(m & 0x80000000u)) { m <<= 1; e++; }
    put16(p, 16383 + 31 - e);
    put32(p + 2, m);
    put32(p + 6, 0);
}

static void tone(signed char *b, int bits, int ch, int n, int *at, double rate, double hz)
{
    int i, c;
    for (i = 0; i < n; i++, (*at)++) {
        double v = 0.5 * sin(2 * M_PI * hz * (*at) / rate);
        for (c = 0; c < ch; c++) {
            if (bits == 8) b[i * ch + c] = (signed char)(v * 127);
            else ((short *)b)[i * ch + c] = (short)(v * 32767);
        }
    }
}

static int write_aiff(const char *path, int rate, int ch, int bits, int frames, double hz)
{
    FILE *f = fopen(path, "wb");
    unsigned char h[54];
    static signed char b[CHUNK * 2 * 2];
    int at = 0, n, bytes = frames * ch * bits / 8;
    if (!f) { perror(path); return 1; }
    memcpy(h, "FORM", 4);
    memcpy(h + 8, "AIFFCOMM", 8); put32(h + 16, 18);
    put16(h + 20, ch); put32(h + 22, frames); put16(h + 26, bits); put_ext80(h + 28, rate);
    memcpy(h + 38, "SSND", 4); put32(h + 42, 8 + bytes); put32(h + 46, 0); put32(h + 50, 0);
    /* FORM's size is everything after its own 8-byte header. */
    put32(h + 4, 4 + 8 + 18 + 8 + 8 + bytes);
    fwrite(h, 1, 54, f);
    while (at < frames) {
        n = frames - at < CHUNK ? frames - at : CHUNK;
        tone(b, bits, ch, n, &at, rate, hz);
        fwrite(b, ch * bits / 8, n, f);
    }
    return fclose(f) != 0;
}

int main(int argc, char **argv)
{
    int rate = 44100, ch = 2, bits = 16, set_rate = 1, o, frames, at = 0, n;
    double secs = 2.0, hz = 1000.0;
    const char *aiff = 0;
    static signed char b[CHUNK * 2 * 2];
    ALconfig cfg;
    ALport port;
    ALpv pv;

    while ((o = getopt(argc, argv, "r:c:b:s:f:nw:")) != -1) {
        switch (o) {
        case 'r': rate = atoi(optarg); break;
        case 'c': ch = atoi(optarg); break;
        case 'b': bits = atoi(optarg); break;
        case 's': secs = atof(optarg); break;
        case 'f': hz = atof(optarg); break;
        case 'n': set_rate = 0; break;
        case 'w': aiff = optarg; break;
        default:
            fprintf(stderr, "usage: audiotest [-r rate] [-c 1|2] [-b 8|16] [-s secs] [-f hz] [-n] [-w file.aiff]\n");
            return 2;
        }
    }
    if ((ch != 1 && ch != 2) || (bits != 8 && bits != 16) || rate <= 0) {
        fprintf(stderr, "audiotest: bad format\n");
        return 2;
    }
    frames = (int)(secs * rate);
    if (aiff) return write_aiff(aiff, rate, ch, bits, frames, hz);

    if (set_rate) {
        pv.param = AL_RATE;
        pv.value.ll = alDoubleToFixed((double)rate);
        if (alSetParams(AL_DEFAULT_OUTPUT, &pv, 1) < 0) { fprintf(stderr, "audiotest: alSetParams failed\n"); return 1; }
    }
    pv.param = AL_RATE;
    if (alGetParams(AL_DEFAULT_OUTPUT, &pv, 1) < 0) { fprintf(stderr, "audiotest: alGetParams failed\n"); return 1; }
    printf("rate %.0f\n", alFixedToDouble(pv.value.ll));

    cfg = alNewConfig();
    alSetChannels(cfg, ch);
    alSetSampFmt(cfg, AL_SAMPFMT_TWOSCOMP);
    alSetWidth(cfg, bits == 8 ? AL_SAMPLE_8 : AL_SAMPLE_16);
    port = alOpenPort("audiotest", "w", cfg);
    if (!port) { fprintf(stderr, "audiotest: alOpenPort failed\n"); return 1; }
    while (at < frames) {
        n = frames - at < CHUNK ? frames - at : CHUNK;
        tone(b, bits, ch, n, &at, (double)rate, hz);
        alWriteFrames(port, b, n);
    }
    while (alGetFilled(port) > 0)
        sginap(5);
    alClosePort(port);
    alFreeConfig(cfg);
    return 0;
}
