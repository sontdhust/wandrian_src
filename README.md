####Project Properties:


_Go to_: 

__C/C++ General__ > __Paths and Symbols__ > __Include__ > __GNU C++__

_Add_:

 `/opt/ros/indigo/include`
 
####Setup:

Change to catkin root directory then run:

    $ catkin_make --force-cmake
    $ . devel/setup.bash
    $ . src/wandrian_run/setup.sh

####Build for testing:

    $ cd src/wandrian_run/test/
    $ ./spiral_stc_test.sh 4 0.5 full_spiral_stc

####Run simulator:

    $ roslaunch wandrian_run environment.launch world_file:=prefered_full_spiral_stc
    $ roslaunch wandrian_run run_simulator.launch plan_name:=full_spiral_stc starting_point_x:=0.75 starting_point_y:=0.25 tool_size:=0.5

####Run practically:

    $ roslaunch kobuki_node minimal.launch --screen
    $ sudo chmod a+rw /dev/ttyACM0
    $ rosrun hokuyo_node hokuyo_node
    $ roslaunch wandrian_run run_practically.launch plan_name:=full_spiral_stc tool_size:=0.4 proportion_ranges_sum:=0.05 augmentation_factor_range:=2.0

####Running mstc_online:
    $ roslaunch wandrian_run environment.launch world_file:=prefered_mstc_online
    $ roslaunch wandrian_run add_robots.launch starting_point_x_robot1:=-1.25 starting_point_y_robot1:=-1.75 starting_point_x_robot2:=1.75 starting_point_y_robot2:=0.25
    $ roslaunch wandrian_run algorithm.launch plan_name:=mstc_online tool_size:=0.5 robot_name:=robot1 starting_point_x:=-1.25 starting_point_y:=-1.75
    $ roslaunch wandrian_run algorithm.launch plan_name:=mstc_online tool_size:=0.5 robot_name:=robot2 starting_point_x:=1.75 starting_point_y:=0.25
