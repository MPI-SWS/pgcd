#! /bin/sh

export PYTHONPATH=$PYTHONPATH:.:..:../..:../../interpreter:$HOME/work/projects/CableRobot/Utils

echo this needs CableRobot/Utils in the PYTHONPATH and dreal in the PATH

echo Running tests
cd test
if [ $# -eq 0 ]; then
    for f in *_test.py; do
        python3 -m unittest $f
    done
else
    for f in $@; do
        python3 -m unittest `basename $f`
    done
fi
