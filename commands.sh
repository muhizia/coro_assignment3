#!/bin/bash
######################################################################################
##  start pulling from git hub the lynxmotion_al5d_description robot simulator      ##
######################################################################################

roscd lynxmotion_al5d_description
git remote set-url origin https://github.com/cognitive-robotics- course/lynxmotion_al5d_description.git
git pull origin master
roscd; cd ..
catkin_make         # executing the program

#########################################################################
##  done creating the lynxmotion_al5d_description robot simulator      ##
#########################################################################
echo "done pulling from git repo"

roscd; cd ../src
git clone https://github.com/cognitive-robotics-course/coro_examples.git
cd ..
catkin_make        # executing the example