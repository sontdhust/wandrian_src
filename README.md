### I. Preparation
- Clone this [repo][1]:
  <pre>
  $ mkdir <i>catkin_workspace</i> && cd $_                    # Example: mkdir wandrian && cd $_
  $ git clone https://github.com/sontdhust/wandrian_src src   # Clone repository and rename to `src`
  </pre>

- Install **build-essential** if not done so already:
  ```
  $ sudo apt-get install build-essential
  ```

### II. Installation
Platform: [Ubuntu Trusty 14.04][2]
- Setup `sources.list` and keys:
  ```
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
  $ sudo apt-get update
  ```

- Install **ROS Indigo Desktop-Full** and **Kobuki** + **Kobuki Gazebo** (_Recommended_):
  ```
  $ sudo apt-get install ros-indigo-desktop-full
  $ sudo apt-get install ros-indigo-kobuki ros-indigo-kobuki-gazebo
  ```
  Or install only **ROS Base** and **Kobuki** (Bare bone):
  ```
  $ sudo apt-get install ros-indigo-ros-base
  $ sudo apt-get install ros-indigo-kobuki
  ```

- Install **Hokuyo**:
  ```
  $ sudo apt-get install ros-indigo-hokuyo-node
  ```

- Install **gmapping**:
  ```
  $ sudo apt-get install ros-indigo-gmapping
  ```

### III. Building
- Build _catkin\_workspace_:
  ```
  $ rm -rf build/
  $ . /opt/ros/indigo/setup.bash
  $ catkin_make --force-cmake           # Do this to re-build when the code has changed
  ```

- Add _catkin\_workspace_ environment variables to bash session every time a new shell is launched:
  ```
  $ echo "" >> ~/.bashrc
  $ echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
  $ echo "source $(pwd)/src/wandrian/setup.sh" >> ~/.bashrc
  $ . ~/.bashrc
  ```

### IV. Testing
- Install **OpenGL/GLUT**:
  ```
  $ sudo apt-get install freeglut3-dev
  ```

- Test:
  <pre>
  $ cd src/wandrian/test/
  $ ./test.sh <i>boundary_size</i> <i>obstacle_size</i> <i>tool_size</i> <i>plan_name</i>
  </pre>

### V. Run on simulator
<pre>
$ roslaunch wandrian environment.launch world_file:=<i>file</i>
$ roslaunch wandrian run_simulator.launch tool_size:=<i>size</i> starting_point_x:=<i>x</i> starting_point_y:=<i>y</i> plan_name:=<i>name</i>
$ rosrun gmapping slam_gmapping scan:=scan
$ rosrun rviz rviz
</pre>

### VI. Run practically
Enable the device to appear on `/dev/kobuki` for the first time:
```
$ rosrun kobuki_ftdi create_udev_rules
```

#### 1. Single machine
- Run:
  <pre>
  $ roslaunch kobuki_node minimal.launch --screen
  $ sudo chmod a+rw /dev/ttyACM0
  $ rosrun hokuyo_node hokuyo_node
  $ roslaunch wandrian run_practically.launch mn:=<i>name</i> ts:=<i>size</i> sp_x:=<i>x</i> sp_y:=<i>y</i> pn:=<i>name</i> lv:=<i>velocity</i> pav:=<i>velocity</i> nav:=<i>velocity</i> l_cr:=<i>rate</i> l_af:=<i>factor</i> e_rd:=<i>epsilon</i> e_md:=<i>epsilon</i> e_p:=<i>epsilon</i> d_lp:=<i>deviation</i> d_ap:=<i>deviation</i> t_lsc:=<i>threshold</i> t_asc:=<i>threshold</i>
  $ rosrun gmapping slam_gmapping
  $ rosrun map_server map_saver         # Save map to disk when finished
  </pre>

#### 2. Multiple machines
- Configuration:
  + Kobuki setup:
    ```
    $ echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
    $ echo "export ROS_HOSTNAME=$(hostname)" >> ~/.bashrc
    ```

  + Remote PC setup:
    <pre>
    $ echo "export ROS_MASTER_URI=http://$(gethostip -d <i>KOBUKI_HOSTNAME</i>):11311" >> ~/.bashrc
    $ echo "export ROS_HOSTNAME=$(hostname)" >> ~/.bashrc
    </pre>

- Run:
  + On Kobuki:
    ```
    $ roslaunch kobuki_node minimal.launch --screen
    $ sudo chmod a+rw /dev/ttyACM0
    $ rosrun hokuyo_node hokuyo_node
    ```

  + On remote PC:
    <pre>
    $ roslaunch wandrian run_practically.launch mn:=<i>name</i> ts:=<i>size</i> sp_x:=<i>x</i> sp_y:=<i>y</i> pn:=<i>name</i> lv:=<i>velocity</i> pav:=<i>velocity</i> nav:=<i>velocity</i> l_cr:=<i>rate</i> l_af:=<i>factor</i> e_rd:=<i>epsilon</i> e_md:=<i>epsilon</i> e_p:=<i>epsilon</i> d_lp:=<i>deviation</i> d_ap:=<i>deviation</i> t_lsc:=<i>threshold</i> t_asc:=<i>threshold</i>
    $ rosrun gmapping slam_gmapping
    $ rosrun rviz rviz
    </pre>

[1]: https://github.com/sontdhust/wandrian_src
[2]: http://wiki.ros.org/indigo/Installation/Ubuntu
