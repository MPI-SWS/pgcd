#! /bin/sh

export PYTHONPATH=$PYTHONPATH:.:..

echo Running tests
for f in test/*.py; do
    python3 -m unittest $f
done

