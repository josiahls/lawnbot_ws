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
        </ol>
        <p> For robot specific dependencies: </p>
        <ol>
            <li>sudo apt-get install ros-kinetic-amcl</li>
            <li>sudo apt-get install ros-kinetic-std_msgs</li>
            <li>sudo apt-get install ros-kinetic-map-server</li>
            <li>sudo apt-get install ros-kinetic-move-base</li>
            <li>sudo apt-get install ros-kinetic-turtlebot</li>
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
    </p>
    <h5>
        <b>Start World:</b>
    </h5>
    <p>
        <ol>
            <li>. ~/lawnbot_ws/devel/setup.bash </li>
            <li>
                roslaunch turtle_gazebo turtle.launch world_file:=worlds/turtle_search_alg_world.world
                <br><b>or</b><br>
                roslaunch turtle_gazebo turtle.launch 
            </li>
            <li>roslaunch turtlebot_teleop keyboard_teleop.launch</li>
            <li>source devel/setup.bash </li>
            <li>rosrun gazebo_ros spawn_model -file `rospack find ROBOT_description`/urdf/ROBOTNAME.urdf -urdf -z 1 -model ROBOTNAME
        </ol>
    </p>
</body>
</html>



