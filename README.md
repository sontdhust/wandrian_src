####For wandrian_mstc_online project

####Project Properties:


_Go to_: 

__C/C++ General__ > __Paths and Symbols__ > __Include__ > __GNU C++__

_Add_:

 `/opt/ros/indigo/include`
 
####Setup:

Change to catkin root directory then run:

    $ catkin_make
    $ . devel/setup.bash
    $ . src/wandrian_run/setup.sh

####Running (wandrian_keyop project (2 robots versison)):

    $ roslaunch wandrian_mstc_online environment.launch world_file:=tmp

    $ roslaunch wandrian_mstc_online add_2_robots.launch starting_point_x_robot1:=0.75 starting_point_y_robot1:=0.25 starting_point_x_robot2:=1.75 starting_point_y_robot2:=-1.75
  
    $ rosrun create_publisher communicate

    $ roslaunch wandrian_mstc_online run_algorithm.launch plan_name:=spiral_stc robot_size:=0.5 robot_name:=robot1 starting_point_x:=0.75 starting_point_y:=0.25

    $ roslaunch wandrian_mstc_online run_algorithm.launch plan_name:=spiral_stc robot_size:=0.5 robot_name:=robot2 starting_point_x:=1.75 starting_point_y:=-1.75
