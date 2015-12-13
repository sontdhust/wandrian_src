

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

####Build for testing (for wandrian_run project):

    $ cd src/wandrian_run/test/
    $ ./test.sh

####Running:

#####1. wandrian_run project

    $ roslaunch wandrian_run environment.launch world_file:=tmp
    $ roslaunch wandrian_run run.launch plan_name:=spiral_stc starting_point_x:=0.75 starting_point_y:=-0.75 robot_size:=0.5

#####2. wandrian_keyop project (2 robots versison)

	$ roslaunch wandrian_keyop load_empty_world.launch
	$ roslaunch wandrian_keyop robots.launch
	$ roslaunch wandrian_keyop keyop.launch robot_name:=robot1
	$ roslaunch wandrian_keyop keyop.launch robot_name:=robot2
