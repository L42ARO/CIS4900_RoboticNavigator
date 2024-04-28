# CIS4900_RoboticNavigator

# Structure
This project is divided into the following folders:
- `docs`: Contains the literature review, the media, the final report, and the reports
- `src`: Contains the source code for the working ROS packages used in the project
- `testing_dev`: Contains programming files used for testing purposes (ignore this folder)
- `resources`: Contains code that was extracted from the robot's computer for reference into how it's algorithms work
- `graph_slam_python`: Contains an implementation of GraphSLAM in a simplified computer simulation that leverages `.g2o` files

# Important Files For Future Students
- `graph_slam_python/creator.py`: Is the file that showcases a simplified simulation of a robot navigating through a room and it creates for you a `.g2o` file that you can use to test the GraphSLAM algorithm
- `graph_slam_python/sample.py`: Is the file that showcases the implementation of the GraphSLAM algorithm
- `resources/run_docker.sh`: Is the updated script that runs the docker container on the robot. This script fixes issues with the depth camera that the default script was having.
- `resources/join-docker.sh`: This script is just a helper script so that in new terminals that you spin up, just call this script and it will join you to the main robot docker
- `src/control_room_ui`: Is the ROS2 package that spins up a server that you can use to send robot information and relay it over a network
- `src/my_graph_slam`: Is the ROS2 package that has the wall following script that should run alongside an OrbSLAM process
- `resources/yahboomcar_ros2_ws`: Is the program files extracted from the robot's computer that showcases how the robot's algorithms work, use these as a reference for your own code

# How to Run
The files that can be run on the ROSMASTERX3 robot are the ROS packages in the `src` folder.

The following instructions apply to those packages as well as any other new package you might want to create inside `src` folder:
1. Turn on the robot and connect to the robot's network `ROSMASTER`. Password for network is `12345678`.
2. SSH into the robot. Hostname is `jetson-desktop`. Password is `yahboom`.
3. Use a tool like winSCP to copy over the `run_docker.sh` script from the `resources` folder into the robot's root.
4. Make sure there is a folder called `usf-robotics` in the root of the `jetson` user. This is wehre your custom programs should be stored given that it's specified to sync with the docker container as a volume.
5. Run the `run_docker.sh` script. This will spin up the docker container that has all the necessary dependencies for the robot to run.
6. Open up 3 new terminals (or 3 new ssh sessions) and run the `join-docker.sh` script in each of them. This will join you to the main docker container.
7. Run the following commands in each of the terminals:
    - `ros2 run yahboomcar_bringup Mcnamu_driver_X3`
    - `ros2 launch astra_camera astro_pro_plus.launch.xml`
    - `ros2 launch sllidar_ros2 sllidar_launch.py`
8. Open a new terminal, use the `join-docker.sh` script to join the docker container, and the do the following:
    - `cd root/usf-robotics/CIS4900_RoboticNavigator/src`
    - `colcon build`
    - `source install/setup.bash`
7. With that done now you can run either the server or the wall following script. To run the server do the following in 2 separate terminals to start displaying the robot's information on a web interface:
    - `ros2 run control_room_ui server`
    - `ros2 run my_graph_slam lidar_visual`
8. To run the wall following script do the following in another terminal:
    - `ros2 run my_graph_slam wall_following`
9. IF you make changes to the code make sure to compile the code again, and to reduce compile times just compile the package that you made changes to:
    - `colcon build --packages-select <package_name>`
