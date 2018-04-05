#! /bin/sh

export PYTHONPATH=$PYTHONPATH:.:..:../..:../../..:$HOME/work/projects/CableRobot/Utils
if [ -f "$HOME/catkin_ws/venv2/bin/activate" ]; then
	. "$HOME/catkin_ws/venv2/bin/activate"
fi

echo this needs CableRobot/Utils in the PYTHONPATH and dreal in the PATH

echo Running tests
cd test
for f in *_test.py; do
    python3 -m unittest $f
done

