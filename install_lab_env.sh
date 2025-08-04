#!/bin/bash

# Intelligent Robotics Lab – Setup Script
# This script installs ROS 2 Humble, Gazebo Classic, RViz, and all necessary tools for the course.
# Author: Elena Villalba
# Usage: bash setup_intelligent_robotics.sh

##########################################################
#                FUNCTION DEFINITIONS                    #
##########################################################

# Install Visual Studio Code 
install_vscode() {
  echo "Visual Studio Code is not installed. Proceeding with installation..."
  sudo apt update 
  sudo snap install --classic code
  echo "Visual Studio Code installation completed."
}

# Install terminator
install_terminator() {
  echo "Terminator is not installed. Proceeding with installation..."
  sudo apt update 
  sudo apt install -y terminator
  echo "Terminator installation completed."
}

# Install git 
install_git() {
  echo "Git is not installed. Proceeding with installation..."
  sudo apt update
  sudo apt install -y git
  echo "Git installation completed."
}

# Install ROS 2 Humble
install_ros2() {
  echo "ROS 2 Humble is not installed. Proceeding with installation..."
  
  # Set locale
  sudo apt update && sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  
  # Setup sources
  sudo apt install -y software-properties-common
  sudo add-apt-repository universe -y
  sudo apt update && sudo apt install curl -y
  
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
  sudo dpkg -i /tmp/ros2-apt-source.deb

  # Intall ROS 2 packages
  sudo apt update 
  sudo apt upgrade -y
  sudo apt install ros-humble-desktop -y
  source /opt/ros/humble/setup.bash
  
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

# Install colcon common extensions
install_colcon_extensions() {
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions
  echo "colcon common extensions installation completed."
}

# Configurate the bash file
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
   
  # Add workspace setup to bashrc
  if ! grep -q "source ~/ir_ws/install/setup.bash" ~/.bashrc; then
    echo "Adding ROS 2 workspace setup to .bashrc"
    echo "source ~/ir_ws/install/setup.bash" >> ~/.bashrc
  fi
  
  # Comment any other workspace setup lines
  sed -i '/source ~\/.*\/install\/setup.bash/ { /source ~\/ir_ws\/install\/setup.bash/!s/^/#/ }' ~/.bashrc

  # Source the updated .bashrc
  source ~/.bashrc
  
  echo "Bash file configuration finished"
  
}

# Create ROS 2 workspace
create_ros2_workspace() {
  # Create ROS 2 workspace if it doesn't exist
  echo "Creating ROS 2 workspace..."
  mkdir -p ~/ir_ws/src
  echo "ROS 2 workspace created."
}

# Clone this repository into the ROS 2 workspace
clone_robotino_ros2_repository() {
  echo "Cloning the repository into the ROS 2 workspace..."
  cd ~/ir_ws/src/
  git clone https://github.com/elena-villalba/intelligent-robotics-student-kit.git . 
  echo "Repository cloned successfully."
}

# Build ROS 2 workspace
build_ros2_workspace() {
  echo "Building ROS 2 workspace..."
  cd ~/ir_ws
  colcon build
  echo "ROS 2 workspace build completed."
  
  # Source the updated .bashrc
  source ~/.bashrc
}


##########################################################
#                        MAIN                            #
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

# Check if Git is install
if ! command -v git &> /dev/null; then
  install_git
else
  echo "Git is already installed."
fi

# Check if ROS 2 Humble is installed
if ! dpkg -l | grep -q ros-humble; then
  install_ros2
else
  echo "ROS 2 Humble is already installed."
fi


# Installing the required ROS 2 packages
install_ros_packages

# Check if pyhton colcon extensions are installed
if ! dpkg -l | grep -q python3-colcon-common-extensions; then
   install_colcon_extensions
else
   echo "colcon common extensions are already installed."
fi

configure_bashrc

create_ros2_workspace

clone_robotino_ros2_repository

build_ros2_workspace

echo ""
echo "✅ Intelligent Robotics Lab environment installed successfully!"
echo "📂 Workspace located at ~/ir_ws"
echo "💡 Run 'source ~/.bashrc' or open a new terminal to use ROS 2 commands."
