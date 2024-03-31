. params.sh

colcon build --packages-skip-regex sjtu_drone* --symlink-install #--cmake-args -DCMAKE_BUILD_TYPE=Release #--packages-select my_package
. install/setup.bash