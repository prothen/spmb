source ./../../devel/setup.bash
cd ~/Arduino/libraries
NAME='_backup_ros_lib_'$(date '+%Y%m%d-%H%M%S')
cp -drf ros_lib ./../$NAME
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

