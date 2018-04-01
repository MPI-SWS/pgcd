#! /bin/sh

export PYTHONPATH=$PYTHONPATH:.:..:../..:../../..

echo Running tests
cd test
for f in *_test.py; do
    python3 -m unittest $f
done

