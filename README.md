<html>
<body>
    <a link="http://gazebosim.org/tutorials?tut=ros_roslaunch"> 
        Custom Gazebo World Creation
    </a>
    <h5>
        <b>ROS Setup:</b>
    </h5>
    <p>
        <ol>
            <li>sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'</li>
            <li>sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116</li>
            <li>sudo apt-get install ros-kinetic-desktop-full</li>
            <li>sudo rosdep init</li>
            <li>rosdep update</li>
            <li>echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc</li>
            <li>source ~/.bashrc</li>
            <li>sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential</li>
            <li>CAUTION: sudo apt-get install python3-catkin-pkg</li>
            <li>CAUTION: sudo apt-get install python3-catkin-pkg-modules</li>
            <li>Reference http://wiki.ros.org/IDEs#PyCharm_.28community_edition.29 for pycharm setup so you dont waste time</li>
        </ol>
        <p> For robot specific dependencies: </p>
        <ol>
            <li>sudo apt-get install ros-kinetic-amcl</li>
            <li>sudo apt-get install ros-kinetic-std-msgs</li>
            <li>sudo apt-get install ros-kinetic-map-server</li>
            <li>sudo apt-get install ros-kinetic-move-base</li>
            <li>sudo apt-get install ros-kinetic-turtlebot</li>
            <li>sudo apt-get install ros-kinetic-lms1xx</li>
        </ol>
    </p>  
    <h5>
        <b>Workspace Setup:</b>
    </h5>
    <p>
        <ol>
            <li>source /opt/ros/kinetic/setup.bash</li>
            <li>cd to home location (recommended to install in home/[user]/[workspace_ws])</li>
            <li>catkin_make</li>
	    <li>source devel/setup.bash</li>
	    <li>"Check if paths are correct: " echo $ROS_PACKAGE_PATH</li>
            <li>"If there are chenges to any of the sources: "catkin_make -DCMAKE_BUILD_TYPE=Release</li>
	    <li>"If you add a new package: " catkin_make --force-cmake</li>
        </ol>
        <p><b>File Structure: </b></p>
        <a>http://gazebosim.org/tutorials?tut=ros_roslaunch</a>
    </p>
    <h5>
        <b>Package Setup:</b>
    </h5>
    <p>
        You should use catkin_create_pkg
        You can reference the tutorial here:
        <a>wiki.ros.org/ROS/Tutorials/CreatingPackage</a> 
        <ol>
            <li>cd ~/lawnbot_ws/src </li>
            <li>catkin_create_pkg [package name] std_msg rospy roscpp </li>
            <li>cd ~/lawnbot_ws </li>
            <li>catkin_make </li>
            <li>. ~/lawnbot_ws/devel/setup.bash </li>
        </ol>
        As a side note, there are some packages you will need to
        download in their respective folders.  
        <a>wiki.ros.org/ROS/mallasrikanth/baxter</a> 
        <ol>
            <li>cd ~/lawnbot_ws/src </li>
            <li>git clone https://github.com/[repository.git]</li>
            <li>git clone https://github.com/jackal/jackal.git</li>
            <li>git clone https://github.com/ros-drivers/pointgrey_camera_driver.git</li>
            <li>git clone https://github.com/jackal/jackal_simulator.git</li>
            <li>git clone https://github.com/jackal/jackal_desktop.git</li>
            <li>git clone https://github.com/ros-visualization/interactive_marker_twist_server.git</li>
            <li>rosdep install --from-paths . --ignore-src --rosdistro=kinetic</li>
            <li>http://wiki.ros.org/jackal_viz</li>
            <li>https://gist.github.com/vfdev-5/57a0171d8f5697831dc8d374839bca12</li>
            <li>catkin build</li>
            <li>catkin_make</li>
        </ol>
    </p>
    <h5>
        <b>Start World:</b>
    </h5>
    <p>
        <ol>
            <li>. ~/lawnbot_ws/devel/setup.bash </li>
            <li>
                roslaunch lawnbot_gazebo turtle.launch world_file:=worlds/turtle_search_alg_world.world
                <br><b>or</b><br>
                roslaunch lawnbot_gazebo turtle.launch 
            </li>
            <li>roslaunch turtlebot_teleop keyboard_teleop.launch or
             roslaunch lawnbot_gazebo -v jackal_worlds.launch config:=front_lawnbot_config</li>
            <li>source devel/setup.bash </li>
            <li>rosrun gazebo_ros spawn_model -file `rospack find ROBOT_descripsourcrtion`/urdf/ROBOTNAME.urdf -urdf -z 1 -model ROBOTNAME
            <li>roslaunch jackal_viz view_robot.launch</li>
        </ol>
    </p>
    <h5>
        <b>Notes:</b>
    </h5>
    <p>
        <ol>
            <li>There is a bug with the default turtlebot. It is incompatable with the current
             version of gazebo without modifying default xml of the model. Specifically there something wrong with the motors.</li> 
            <li>for creating an empty world: roslaunch gazebo_ros empty_world.launch </li>
            <li>Note for model building in blender, you need to use the blender renderer, then simply export the material </li>
        </ol>
    </p>
        <h5>
        <b>Scripts:</b>
    </h5>
    <p>
        <ol>
            <li> make sure the pycharm's launcher is referencing bash (explained in first paragraph) </li>
            <li> create scripts folder at top of package </li>
            <li> add blank __init__.py </li>
            <li> do chmod -x on any scripts to be executable </li>
            <li> execute using: rosrun [package] [script] </li>
        </ol>
    </p>
    <p>
        <ol>
            <li> Note, the range 0 of the laser scanner is far right side </li>
        </ol>
    </p>
</body>
</html>



