####Project Properties:


_Go to_: 

__C/C++ General__ > __Paths and Symbols__ > __Include__ > __GNU C++__

_Add_:

 `/opt/ros/indigo/include`
 
####Setup:

Change to catkin root directory then run:

    $ rm -rf build/
    $ catkin_make --force-cmake
    $ . devel/setup.bash
    $ . src/wandrian/setup.sh

####Build for testing:

    $ cd src/wandrian/test/
    $ ./test.sh 4 0.4 0.4 full_spiral_stc

####Run simulator:

    $ roslaunch wandrian environment.launch world_file:=prefered_full_spiral_stc
    $ roslaunch wandrian run_simulator.launch map_boundary_width:=4 map_boundary_height:=4 tool_size:=0.5 starting_point_x:=0.75 starting_point_y:=0.25 plan_name:=full_spiral_stc

####Run practically:

    $ roslaunch kobuki_node minimal.launch --screen
    $ sudo chmod a+rw /dev/ttyACM0
    $ rosrun hokuyo_node hokuyo_node
    $ roslaunch wandrian run_practically.launch mb_w:=4.8 mb_h:=3.2 ts:=0.4 sp_x:=0.6 sp_y:=-0.6 pn:=full_spiral_stc lv:=0.15 av:=0.75 pr_c:=0.5 pr_s:=0.2 af_r:=2.0 e_rd:=0.06 e_md:=0.24 e_p:=0.06

####Running mstc_online:

######2 robots (add one by one robot):
    $ roslaunch wandrian environment.launch world_file:=prefered_mstc_online_for_show
    $ roslaunch wandrian add_1_robot.launch starting_point_x_robot:=-1.25 starting_point_y_robot:=-1.75 robot_name:=robot1
    $ roslaunch wandrian add_1_robot.launch starting_point_x_robot:=1.75 starting_point_y_robot:=0.25 robot_name:=robot2
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot1 tool_size:=0.5 starting_point_x:=-1.25 starting_point_y:=-1.75 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=x.x.x.x
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot2 tool_size:=0.5 starting_point_x:=1.75 starting_point_y:=0.25 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=x.x.x.x

######2 robots:
    $ roslaunch wandrian environment.launch world_file:=prefered_mstc_online_for_show
    $ roslaunch wandrian add_2_robots.launch starting_point_x_robot1:=-1.25 starting_point_y_robot1:=-1.75 starting_point_x_robot2:=1.75 starting_point_y_robot2:=0.25
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot1 tool_size:=0.5 starting_point_x:=-1.25 starting_point_y:=-1.75 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot2 tool_size:=0.5 starting_point_x:=1.75 starting_point_y:=0.25 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need

######3 robots:
    $ roslaunch wandrian environment.launch world_file:=prefered_mstc_online_for_show
    $ roslaunch wandrian add_3_robots.launch starting_point_x_robot1:=-1.25 starting_point_y_robot1:=-1.75 starting_point_x_robot2:=1.75 starting_point_y_robot2:=0.25 starting_point_x_robot3:=-1.25 starting_point_y_robot3:=0.25
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot1 tool_size:=0.5 starting_point_x:=-1.25 starting_point_y:=-1.75 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot2 tool_size:=0.5 starting_point_x:=1.75 starting_point_y:=0.25 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot3 tool_size:=0.5 starting_point_x:=-1.25 starting_point_y:=0.25 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need

######4 robots:
    $ roslaunch wandrian environment.launch world_file:=prefered_mstc_online_for_show
    $ roslaunch wandrian add_4_robots.launch starting_point_x_robot1:=-1.25 starting_point_y_robot1:=-1.75 starting_point_x_robot2:=1.75 starting_point_y_robot2:=0.25 starting_point_x_robot3:=-1.25 starting_point_y_robot3:=0.25 starting_point_x_robot4:=-1.25 starting_point_y_robot4:=1.25
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot1 tool_size:=0.5 starting_point_x:=-1.25 starting_point_y:=-1.75 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot2 tool_size:=0.5 starting_point_x:=1.75 starting_point_y:=0.25 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot3 tool_size:=0.5 starting_point_x:=-1.25 starting_point_y:=0.25 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need
    $ roslaunch wandrian algorithm_simulator.launch plan_name:=mstc_online robot_name:=robot4 tool_size:=0.5 starting_point_x:=-1.25 starting_point_y:=1.25 map_boundary_width:=4.0 map_boundary_height:=4.0 ip_server:=no_need

####Prefered arguments:

lv:=0.18
av:=0.9
pr_c:=0.4
pr_s:=0.2
af_r:=3.0
e_rd:=0.2
e_md:=0.36
e_p:=0.04
