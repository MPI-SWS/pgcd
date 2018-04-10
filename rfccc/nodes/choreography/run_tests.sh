#! /bin/sh

export PYTHONPATH=$PYTHONPATH:.:..:spec:spec/test

echo Running tests
for f in test/*.py; do
    python3 -m unittest $f
done

