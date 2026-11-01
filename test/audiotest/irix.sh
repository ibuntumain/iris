#!/bin/sh
# Build the audio test on IRIX with MIPSpro cc (stock IRIX has no bash).
# Usage: sh irix.sh

echo "cc -o audiotest audiotest.c"
cc -o audiotest audiotest.c -mips3 -n32 -laudio -lm || exit 1
echo "Done. Run sh sweep.sh all"
