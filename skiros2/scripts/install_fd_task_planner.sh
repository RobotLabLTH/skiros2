#!/bin/bash

tfd='tfd-src-0.4'

echo "Installing planner..."

#Navigate to install folder
default="~/Software"
echo "Select install folder or leave blank for default [default: $default]:"
read folder
if [[ $folder == "" ]]; then
    folder=${default/"~"/$HOME}
else
    folder=${folder/"~"/$HOME}
fi

# Installing planner
if [ ! -d "$folder/${tfd}" ]; then
    echo "Installing planner."
    mkdir -p $folder
    cd $folder
    echo $folder

    #Install
    dpkg -s g++ &> /dev/null
    gpp_installed=$?
    dpkg -s make &> /dev/null
    make_installed=$?
    if [ $gpp_installed -ne 0 ] || [ $make_installed -ne 0 ]; then
        echo "Installing g++ and make"
        sudo apt install -y g++ make
    fi
    wget "http://gki.informatik.uni-freiburg.de/tools/tfd/downloads/version-0.4/${tfd}.tgz"
    tar xzf "${tfd}.tgz"
    cd "${tfd}"
    sed -e s/"-Werror"//g -i ./downward/search/Makefile
    sed -e s/"translate\/"//g -i ./downward/plan.py
    sed -e s/"preprocess\/"//g -i ./downward/plan.py
    sed -e s/"search\/"//g -i ./downward/plan.py
    ./build
    cd -
    rm -r "${tfd}.tgz"
else
	echo "Folder $folder/${tfd} already exists. Skipping installation."
fi

# Adding environment variables
echo "Add environment variable to bashrc"
string="export TFD_HOME=${folder}"
string2="export PATH=$""TFD_HOME/${tfd}/downward:$""TFD_HOME/${tfd}/downward/translate:$""TFD_HOME/${tfd}/downward/preprocess:$""TFD_HOME/${tfd}/downward/search:""$""PATH"
temp=$(cat ~/.bashrc | grep "$string")
if [ -z "$temp" ]; then
    echo "# TDF Planner for SkiROS" >> ~/.bashrc
	echo $string >> ~/.bashrc
	echo $string2 >> ~/.bashrc
	echo "Printing string in .bashrc: $string"
	echo "Printing string in .bashrc: $string2"
else
	echo "No export string added to bashrc (it is already there)"
fi

source ~/.bashrc
