### Preparation
<pre>
$ mkdir <i>catkin_workspace</i> && cd $_                    # Example: mkdir wandrian && cd $_
$ git clone https://github.com/sontdhust/wandrian_src src   # Clone repository and rename to `src`
</pre>

### Installation
#### 1. From scratch
  - Install `ROS Indigo desktop full` on [Ubuntu Trusty 14.04][1]
  ```
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
  $ sudo apt-get update
  $ sudo apt-get install ros-indigo-desktop-full
  $ source /opt/ros/indigo/setup.bash
  ```

  - Install `ROS Kobuki` + `ROS Kobuki Gazebo`
  ```
  $ sudo apt-get install ros-indigo-kobuki ros-indigo-kobuki-gazebo
  ```

  - Install `gmapping`
  ```
  $ sudo apt-get install ros-indigo-gmapping
  ```

#### 2. Use docker
  - Install `Docker` on [Ubuntu Xenial 16.04][2]

  - Build image and run container:
  <pre>
  $ cd src/docker
  $ ./build.sh <i>IMAGE_NAME</i>      # Build image
  $ mkdir .ros && mkdir .gazebo
  $ ./run.sh <i>IMAGE_NAME</i>        # Run container
  </pre>

### Building
_Note: Run these instructions inside container if you use `Docker`_
  - Build <i>catkin_workspace</i>
  ```
  $ rm -rf build/
  $ catkin_make --force-cmake
  $ . devel/setup.bash
  $ . src/wandrian/setup.sh
  ```

  - Install `OpenGL/GLUT` for testing only
  ```
  $ sudo apt-get install freeglut3-dev
  ```

### Testing:
<pre>
$ cd src/wandrian/test/
$ ./test.sh <i>boundary_size</i> <i>obstacle_size</i> <i>tool_size</i> <i>plan_name</i>
</pre>

### Run simulator:
<pre>
$ roslaunch wandrian environment.launch world_file:=<i>file</i>
$ roslaunch wandrian run_simulator.launch tool_size:=<i>size</i> starting_point_x:=<i>x</i> starting_point_y:=<i>y</i> plan_name:=<i>name</i>
$ rosrun gmapping slam_gmapping scan:=scan
$ rosrun rviz rviz
</pre>

### Run practically:
<pre>
$ roslaunch kobuki_node minimal.launch --screen
$ sudo chmod a+rw /dev/ttyACM0
$ rosrun hokuyo_node hokuyo_node
$ roslaunch wandrian run_practically.launch mn:=<i>name</i> ts:=<i>size</i> sp_x:=<i>x</i> sp_y:=<i>y</i> pn:=<i>name</i> lv:=<i>velocity</i> pav:=<i>velocity</i> nav:=<i>velocity</i> l_cr:=<i>rate</i> l_af:=<i>factor</i> e_rd:=<i>epsilon</i> e_md:=<i>epsilon</i> e_p:=<i>epsilon</i> d_lp:=<i>deviation</i> d_ap:=<i>deviation</i> t_lsc:=<i>threshold</i> t_asc:=<i>threshold</i>
$ rosrun gmapping slam_gmapping
$ rosrun map_server map_saver
</pre>

[1]: http://wiki.ros.org/indigo/Installation/Ubuntu
[2]: https://docs.docker.com/engine/installation/linux/ubuntulinux/
