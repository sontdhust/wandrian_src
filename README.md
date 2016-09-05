### Prerequisites:
- Install `Docker` on [Ubuntu Xenial 16.04][1]

- Build image and run container:
  <pre>
  $ cd docker
  $ ./build.sh <i>IMAGE_NAME</i>      # Build image
  $ mkdir .ros && mkdir .gazebo
  $ ./run.sh <i>IMAGE_NAME</i>        # Run container
  </pre>

-  Setup (inside container environment):
  ```
  $ rm -rf build/
  $ catkin_make --force-cmake
  $ . devel/setup.bash
  $ . src/wandrian/setup.sh
  ```

### Build for testing:
<pre>
$ cd src/wandrian/test/
$ ./test.sh <i>boundary_size</i> <i>obstacle_size</i> <i>tool_size</i> <i>plan_name</i>
</pre>

### Run simulator:
<pre>
$ roslaunch wandrian environment.launch world_file:=<i>file</i>
$ roslaunch wandrian run_simulator.launch map_boundary_width:=<i>width</i> map_boundary_height:=<i>height</i> tool_size:=<i>size</i> starting_point_x:=<i>x</i> starting_point_y:=<i>y</i> plan_name:=<i>name</i>
$ rosrun gmapping slam_gmapping scan:=scan
$ rosrun rviz rviz
</pre>

### Run practically:
<pre>
$ roslaunch kobuki_node minimal.launch --screen
$ sudo chmod a+rw /dev/ttyACM0
$ rosrun hokuyo_node hokuyo_node
$ roslaunch wandrian run_practically.launch mb_w:=<i>width</i> mb_h:=<i>height</i> ts:=<i>size</i> sp_x:=<i>x</i> sp_y:=<i>y</i> pn:=<i>name</i> lv:=<i>velocity</i> av:=<i>velocity</i> pr_c:=<i>proportion</i> pr_s:=<i>proportion</i> af_r:=<i>augmentation</i> e_rd:=<i>epsilon</i> e_md:=<i>epsilon</i> e_p:=<i>epsilon</i>
</pre>

[1]: https://docs.docker.com/engine/installation/linux/ubuntulinux/
