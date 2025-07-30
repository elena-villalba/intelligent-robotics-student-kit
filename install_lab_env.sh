#!/bin/bash

# Intelligent Robotics Lab – Setup Script
# This script installs ROS 2 Humble, Gazebo Classic, RViz, and all necessary tools for the course.
# Author: Elena Villalba
# Usage: bash setup_intelligent_robotics.sh

##########################################################
# 		           FUNCTION DEFINITIONS                 	 #
##########################################################

# Install Visual Studio Code 
install_vscode() {
  echo "Visual Studio Code is not installed. Proceeding with installation..."
  sudo apt update 
  sudo snap install code --classic
  echo "Visual Studio Code installation completed."
}

# Install terminator
install_terminator() {
  echo "Terminator is not installed. Proceeding with installation..."
  sudo apt update 
  sudo apt install -y terminator
  echo "Terminator installation completed."
}

# Install ROS 2 Humble
install_ros2() {
  echo "ROS 2 Humble is not installed. Proceeding with installation..."
  
  # Set locale
  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  
  # Setup sources
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  
  # Intall ROS 2 packages
  sudo apt update 
  sudo apt upgrade -y
  sudo apt install ros-humble-desktop -y
  source /opt/ros/humble/setup.bash
  
  # Add ROS 2 repository
  sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
  
  echo "ROS 2 Humble installation completed."
}

# Install the required ROS 2 packages
install_ros_packages() {
  echo "Installing the required ROS 2 packages..."
  sudo apt update

  sudo apt install -y ros-humble-xacro
  sudo apt install -y ros-humble-gazebo-ros
  sudo apt install -y ros-humble-gazebo-ros-pkgs
  sudo apt install -y ros-humble-rviz2

  # Needed for JPL ROS packages
  sudo apt install -y ros-humble-controller-manager
  sudo apt install -y ros-humble-robot-state-publisher
  sudo apt install -y ros-humble-joint-state-publisher
  sudo apt install -y ros-humble-joint-state-publisher-gui 
  sudo apt install -y ros-humble-trajectory-msgs
  sudo apt install -y ros-humble-velocity-controllers
  sudo apt install -y ros-humble-joint-trajectory-controller
  sudo apt install -y ros-humble-gazebo-ros2-control-demos

  echo "ROS 2 packages intallation completed."

}

# Install git 
install_git() {
  echo "Git is not installed. Proceeding with installation..."
  sudo apt update
  sudo apt install -y git
  echo "Git installation completed."
}

# Install colcon common extensions
install_colcon_extensions() {
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions
  echo "colcon common extensions installation completed."
}

# Function to configurate the bash file
configure_bashrc() {
  echo "Configuring the bash file..."
  # Ensure ROS 2 Humble environment setup is sourced in .bashrc
  if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "Adding ROS 2 Humble environment setup to .bashrc"
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  fi

  # Add colcon argcomplete to bashrc
  if ! grep -q "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc; then
    echo "Adding colcon argcomplete to .bashrc"
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
  fi

  # Add gazebo to bashrc
  if ! grep -q "source /usr/share/gazebo/setup.bash" ~/.bashrc; then
    echo "Adding gazebo to .bashrc"
    echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
  fi

  # Add gazebo model path 
  if ! grep -q "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robotino4-ros2/src/" ~/.bashrc; then
    echo "Adding gazebo model path to .bashrc"
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robotino4-ros2/src/" >> ~/.bashrc
  fi
   
  # Add workspace setup to bashrc
  if ! grep -q "source ~/robotino4-ros2/install/setup.bash" ~/.bashrc; then
    echo "Adding ROS 2 workspace setup to .bashrc"
    echo "source ~/robotino4-ros2/install/setup.bash" >> ~/.bashrc
  fi
  
  # Comment any other workspace setup lines
  sed -i '/source ~\/.*\/install\/setup.bash/ { /source ~\/robotino4-ros2\/install\/setup.bash/!s/^/#/ }' ~/.bashrc

  # Source the updated .bashrc
  source ~/.bashrc
  
  echo "Bash file configuration finished"
  
}


##########################################################
# 			                  MAIN 			                  	 #
##########################################################

echo "Installing the necesary resources..."

# Check if Visual Studio Code is installed
if ! snap list | grep -q code; then
  install_vscode
else
  echo "Visual Studio Code is already installed."
fi

# Check if terminator is installed
if ! dpkg -l | grep -q terminator; then
  install_terminator
else
  echo "Terminator is already installed."
fi

# Check if ROS 2 Humble is installed
if ! dpkg -l | grep -q ros-humble; then
  install_ros2
else
  echo "ROS 2 Humble is already installed."
fi

# Installing the required ROS 2 packages
install_ros_packages

# Check if Git is install
if ! command -v git &> /dev/null; then
  install_git
else
  echo "Git is already installed."
fi

# Check if pyhton colcon extensions are installed
if ! dpkg -l | grep -q python3-colcon-common-extensions; then
   install_colcon_extensions
else
   echo "colcon common extensions are already installed."
fi

# Check if python3 virtual library is installed
# Comprobamos si python3-venv está instalado
if ! dpkg -l python3.10-venv &>/dev/null; then
  install_python_virtual_env
else
    echo "python3-venv is already installed"
fi

# Bash file configuration
configure_bashrc

echo " "
echo "Installations and configurations are already completed!"
