#!/usr/bin/env bash

if [[ $# != 1 ]]; then
	echo "Usage:"
	echo "\t$0 <package name>"
fi

cd src
ros2 pkg create --build-type ament_python $1
cd $1

grep -rl 'script-dir' | xargs sed -i 's/script-dir/script_dir/g'
grep -rl 'install-scripts' | xargs sed -i 's/install-scripts/install_scripts/g'
grep -rl '<license>' | xargs sed -i 's/TODO: License declaration/GPLv3/g'
grep -rl '<version>' | xargs sed -i 's/0.0.0/0.0.1/g'
grep -rl '<description>' | xargs sed -i 's/TODO: Package description/crow package/g'
grep -rl 'meowxiik' | xargs sed -i 's/meowxiik/imitrob_project/g'
