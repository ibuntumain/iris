#!/bin/sh
# sweep.sh: run in the guest, in the directory holding audiotest (irix.sh
# builds it). Plays 1 kHz, 2.0 s tones at each rate x channels x width through
# three paths, and a sticky-rate pair. Captured by the emulator; check.py on
# the host judges the capture.
#
#   sh sweep.sh cases        list the cases, one name per line
#   sh sweep.sh prep NAME    what must happen before NAME's timed play
#   sh sweep.sh play NAME    play NAME (this is the part that gets timed)
#   sh sweep.sh all          prep and play every case, for listening by ear
#
# Case names (RATE_Cc_Bb is e.g. 22050_2c_16b):
#   gen_RATE_Cc_Bb        audiotest plays through the audio library
#   playaiff_RATE_Cc_Bb   playaiff plays an AIFF audiotest wrote
#   sfplay_RATE_Cc_Bb     sfplay plays the same AIFF
#   sticky_lib            a 22050 Hz tone first, then 44100 Hz through the library
#   sticky_aiff           a 22050 Hz tone first, then a 44100 Hz AIFF in playaiff
#
# The AIFFs are written by audiotest on first use, so nothing needs staging
# beyond audiotest and this script. They go under /usr/tmp: IRIX clears /tmp
# on boot.
D=${AIFFDIR:-/usr/tmp/audiotest.aiff}
RATES="8000 11025 16000 22050 32000 44100 48000"

aiff() {   # RATE CH BITS: print the AIFF's path, writing it if absent
    f=$D/t$1_$2c_$3.aiff
    mkdir -p $D
    [ -f $f ] || ./audiotest -r $1 -c $2 -b $3 -w $f || exit 1
    echo $f
}

# fields RATE CH BITS out of a name like gen_22050_2c_16b
fields() {
    echo $1 | sed 's/^[a-z]*_\([0-9]*\)_\([12]\)c_\([0-9]*\)b$/\1 \2 \3/'
}

case $1 in
cases)
    for r in $RATES; do for c in 1 2; do for b in 8 16; do
        echo gen_${r}_${c}c_${b}b
    done; done; done
    for p in playaiff sfplay; do
        for r in $RATES; do for c in 1 2; do for b in 8 16; do
            echo ${p}_${r}_${c}c_${b}b
        done; done; done
    done
    echo sticky_lib
    echo sticky_aiff
    ;;
prep)
    case $2 in
    sticky_*) ./audiotest -r 22050 -c 2 -b 16 ;;
    playaiff_*|sfplay_*) set -- x `fields $2`; aiff $2 $3 $4 >/dev/null ;;
    esac
    ;;
play)
    case $2 in
    gen_*) set -- x `fields $2`; ./audiotest -r $2 -c $3 -b $4 ;;
    playaiff_*) set -- x `fields $2`; playaiff `aiff $2 $3 $4` ;;
    sfplay_*) set -- x `fields $2`; sfplay `aiff $2 $3 $4` ;;
    sticky_lib) ./audiotest -r 44100 -c 2 -b 16 ;;
    sticky_aiff) playaiff `aiff 44100 2 16` ;;
    *) echo "sweep.sh: unknown case $2" >&2; exit 2 ;;
    esac
    ;;
all)
    for c in `sh $0 cases`; do
        echo "== $c"; sh $0 prep $c; sh $0 play $c
    done
    ;;
*)
    sed -n 2,10p $0; exit 2 ;;
esac
