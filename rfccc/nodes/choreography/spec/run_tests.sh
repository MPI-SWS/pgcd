#! /bin/sh

export PYTHONPATH=$PYTHONPATH:.:..:../..:../../..
. /home/gbbanusic/catkin_ws/venv2/bin/activate
echo Running tests
cd test
for f in *_test.py; do
    python3 -m unittest $f
done

