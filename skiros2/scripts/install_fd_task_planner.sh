#!/bin/bash

echo "Installing planner..."

#Navigate to install folder
default="~/.skiros/planner"
echo "Select install folder or leave blank for default [default: $default]:"
read folder
if [[ $folder == "" ]]; then
    folder=${default/"~"/$HOME}
else
    folder=${folder/"~"/$HOME}
fi

# Installing planner
if [ ! -d "$folder/tfd" ]; then
    echo "Installing planner."
    mkdir -p $folder
    cd $folder
    git clone https://github.com/neighthan/tfd
    cd tfd
    ./build
    cd -
else
    echo "Folder $folder/${tfd} already exists. Skipping installation."
    cd $folder
fi

echo "Add environment variable to bashrc"
string="export TFD_HOME=$(pwd)/tfd/downward"
temp=$(cat ~/.bashrc | grep "$string")
if [ -z "$temp" ]; then
    echo "# TDF Planner for SkiROS" >> ~/.bashrc
    echo $string >> ~/.bashrc
    echo "Printing string in .bashrc: $string"
else
    echo "No export string added to bashrc (it is already there)"
fi

source ~/.bashrc
