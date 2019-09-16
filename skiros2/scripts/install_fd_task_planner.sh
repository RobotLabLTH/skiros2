#!/bin/bash

tfd='tfd-src-0.4'

echo "Installing planner..."

#Navigate to install folder
default="~/utils"
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
sed -e s/"translate\/"//g -i ./downward/plan.py
sed -e s/"preprocess\/"//g -i ./downward/plan.py
sed -e s/"search\/"//g -i ./downward/plan.py
./build
cd -
rm -r "${tfd}.tgz"

#Add environment variable to bashrc
string="export TFD_HOME=$(pwd)"
string2="export PATH=$""TFD_HOME/${tfd}/downward:$""TFD_HOME/${tfd}/downward/translate:$""TFD_HOME/${tfd}/downward/preprocess:$""TFD_HOME/${tfd}/downward/search:""$""PATH"
temp=$(cat ~/.bashrc | grep "$string")
if [ -z "$temp" ]; then
	echo $string >> ~/.bashrc
	echo $string2 >> ~/.bashrc
	echo "Printing string in .bashrc: $string"
else
	echo "No export string added to bashrc (it is already there)"
fi

source ~/.bashrc
