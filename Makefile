.PHONY: build detector

build:
	env --ignore-environment /bin/bash -c 'source .env;export PYTHONWARNINGS;source /opt/ros/foxy/setup.bash; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

pkg:
	env --ignore-environment /bin/bash -c 'source /opt/ros/foxy/setup.bash; colcon build --symlink-install --packages-select $(SELECT)'

clean:
	@rm -rf log build install
	find . | grep '\.pyc$$' | xargs rm
	find . | grep '__pycache__$$' | xargs rm -r

git:
	git submodule deinit --all -f
	git submodule init
	git submodule update

rosdep:
	rosdep install --from-paths src -y --ignore-src
