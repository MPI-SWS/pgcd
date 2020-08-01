#!/bin/bash

# this path is relative to the test folder, not this folder
export PYTHONPATH=$PYTHONPATH:.:..:../..:../choreography:
# echo PYTHONPATH is $PYTHONPATH

if ! command -v dreal &> /dev/null
then
    echo "dreal not found. Aborting."
    exit
fi

files=()
opts=()

while [ ! -z "$1" ]; do
    case "$1" in
        -*)
            opts+=($1)
            shift
            ;;
        *)
            files+=($1)
            shift
            ;;
    esac
done

echo "Running test(s)"
cd test
if [ ${#files[@]} -eq 0 ]; then
    for f in *_test.py; do
        echo $f
        python3 -m unittest $f $opts
    done
else
    for f in $files; do
        echo $f
        python3 -m unittest `basename $f` $opts
    done
fi
