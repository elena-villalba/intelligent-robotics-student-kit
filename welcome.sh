#!/bin/bash

# ######################################################################################### #
# Script Name : welcome.sh
# Description : Displays a welcome message for the Intelligent Robotics course.
#               Optionally shows extra information when 'i' is typed and Enter is pressed.
#
# Author      : Elena Villalba
# Date        : 08-08-2025
# Course      : Intelligent Robotics 2025 – Lab Session 1
# Repository  : https://github.com/elena-villalba/intelligent-robotics-student-kit
#
# Usage       : ./welcome.sh
# Notes       : Make sure 'figlet' is installed to see the ASCII banner (optional).
#               Also, make the script executable with: chmod +x welcome.sh
# Dependencies: bash, figlet (optional)
# ######################################################################################### #

# Color variables
RED="\033[0;31m"
NC="\033[0m" # No Color

# Clear the terminal
clear

# Display header
if command -v figlet >/dev/null 2>&1; then
    printf "${RED}"
    figlet -c "Intelligent Robotics"
    printf "${NC}"
else
    printf "${RED}Intelligent Robotics${NC}\n"
    echo "Tip: Install 'figlet' to see this title in ASCII art: sudo apt install figlet"
fi

# Course info
echo
printf "                  Welcome to the ${RED}Intelligent Robotics${NC} Course\n\n"
printf "${RED}[${NC}About the course${RED}]${NC}\n\n"
echo "Coded by   : Elena Villalba"
echo "Version    : Practices - Session 1"
echo "Repository : https://github.com/elena-villalba/intelligent-robotics-student-kit"
echo

# User prompt
printf "Type '${RED}i${NC}' + Enter to view details about this script, or just press Enter to exit: "

# Read user input (on same line)
read -r input

# Display extra info
if [[ "$input" == "i" || "$input" == "I" ]]; then
    echo
    echo
    printf "${RED}[${NC}Information about this script${RED}]${NC}\n\n"
    echo "This script is part of the Intelligent Robotics 2025 course."
    echo "It is an example for Exercise 1 of Lab Session 1."
    echo "The goal is to help you practice terminal commands and basic scripting concepts."
    echo
    echo -n "Script directory : "
    pwd
    echo -n "Script details   : "
    ls -l "$0"
    echo
    echo
    printf "${RED}[${NC}Motivational quote${RED}]${NC}\n\n"
    echo "\"The best way to predict the future is to invent it.\" – Alan Kay"
    echo
    echo
else
    echo
    echo "Exiting..."
    sleep 1
    clear
fi