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

####Build for testing:

    $ cd src/wandrian_run/test/
    $ ./test.sh

####Running:

    $ roslaunch wandrian_run environment.launch world_file:=tmp
    $ roslaunch wandrian_run run.launch plan_name:=online_boustrophedon starting_point_x:=-1.75 starting_point_y:=-1.75 robot_size:=0.5
