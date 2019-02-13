#!/bin/sh

# this path is relative to the test folder, not this folder
export PYTHONPATH=$PYTHONPATH:.:..:../..:../../interpreter:../choreography:

echo PYTHONPATH is $PYTHONPATH

echo These tests need dreal in the PATH to run.

echo Running tests
cd test
if [ $# -eq 0 ]; then
    for f in *_test.py; do
        echo $f
        python3 -m unittest $f
    done
else
    for f in $@; do
        echo $f
        python3 -m unittest `basename $f`
    done
fi
