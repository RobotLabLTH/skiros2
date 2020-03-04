#!/bin/bash

tfd='tfd-src-0.4'

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
mkdir -p $folder
cd $folder
echo $folder

#Install
wget "http://gki.informatik.uni-freiburg.de/tools/tfd/downloads/version-0.4/${tfd}.tgz"
tar xzf "${tfd}.tgz"
cd "${tfd}"
sed -e s/"-Werror"//g -i ./downward/search/Makefile
./build
cd -
rm -r "${tfd}.tgz"

#Add environment variable to bashrc
string="export TFD_HOME=$(pwd)/${tfd}/downward"
temp=$(cat ~/.bashrc | grep "$string")
if [ -z "$temp" ]; then
	echo $string >> ~/.bashrc
	echo "Printing string in .bashrc: $string"
else
	echo "No export string added to bashrc (it is already there)"
fi

source ~/.bashrc
