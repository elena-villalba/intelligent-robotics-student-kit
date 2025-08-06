# intelligent-robotics-student-kit
Educational resources for the Intelligent Robotics course – ROS 2 examples, Gazebo simulations, setup scripts, and quick-start guides.

## Requirements

This course is designed to run on:

- Ubuntu 22.04 LTS
- ROS 2 Humble (installed via setup script)
- Gazebo 11 (installed via setup script)

## Installin Ubuntu 22.04

If you don't already have Ubuntu 22.04 installed, we recommend the following options (in order of preference):

- Install Ubuntu on an external SSD
- Dual-boot installation on a separate drive alongside Windows
- Virtual machine (e.g., VirtualBox – less recommended due to hardware limitations)

▶️ Recommended video tutorial: 

> The lab sessions are designed to be completed on the lab computers. You do not need to install anything on your personal machine unless you want to continue experimenting at home.

## Instalation of Required Resources:

To set up your environment for working with this repository, follow these steps:

1. **Download the install_resources.sh Script:**

   ```bash
   wget https://github.com/elena-villalba/intelligent-robotics-student-kit/raw/main/install_lab_env.sh
   ```
   
2. **Grand Execution Permissions:**

   ```bash
   chmod +x install_lab_env.sh
   ```

3. **Execute the script:**

   ```bash
   ./install_lab_env.sh
   ```

## Getting Started with ROS 2

Once the script has finished, try running a basic ROS 2 demo:

In one terminal:

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

In a second terminal:

   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

Use the arrow keys to move the turtle!

## Project

The `osr_maze_nav`package contains a very simple solution for the project goal: to automously navigate a maze without collisions, usign only Lidar 

### How to launch:

- **Terminal 1** - Launch the Gazebo maze world and robot:

   ```bash
   ros2 launch osr_bringup maze_world.launch.py 
   ```

- **Terminal 2** - Run the wall-following navigation algoritm:
   ```bash
   os2 run osr_maze_nav wall_follower 
   ```



## Acknowledgements

The `osr-gazebo` simulation assets are adapted from the [OSR Rover Code](https://github.com/nasa-jpl/osr-rover-code) by NASA JPL, used under the terms of the Apache 2.0 License.
