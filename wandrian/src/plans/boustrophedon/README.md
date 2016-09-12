####Project Properties:

_Go to_:

__C/C++ General__ > __Paths and Symbols__ > __Include__ > __GNU C++__

_Add_:

 `/opt/ros/indigo/include`

####Setup:

Change to catkin root directory then run:

    $ catkin_make
    $ . devel/setup.bash
    $ . src/wandrian/setup.sh

####Build for testing:

    $ cd src/wandrian/testt/
    $ ./test.sh

####Running:

    $ roslaunch wandrian environment.launch world_file:=tmp
    $ roslaunch wandrian run.launch plan_name:=boustrophedon starting_point_x:=-2.75 starting_point_y:=-2.75 robot_size:=0.5 environment_size:=6


echo "source phanthao/GR/workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc

roslaunch wandrian boustrophedon_run.launch plan_name:=boustrophedon starting_point_x:=-2.75 starting_point_y:=-2.75 tool_size:=0.5