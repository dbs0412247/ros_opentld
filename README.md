Work in progress: 

- Stereo Version of OpenTLD 

Currently only supporting horizontal stereo.

TODO: increase amount of information displayed

Setup:

- git clone this repo to $(YOUR_CATKIN_WORKSPACE)/src
- catkin_make 
- Launch your camera driver
- Modify $(THIS_REPO_ROOT)/launch/ros_tld_left_tracker.launch, $(THIS_REPO_ROOT)/launch/ros_tld_right_tracker.launch and $(THIS_REPO_ROOT)/launch/ros_tld_stereo.launch to provide the correct image topics

How to run/use:

- roslaunch (all 3 of) the launch files modified in Setup

* `q`     quit
* `b`     remember current frame as background model / clear background
* `c`     stop/resume tracking
* `l`     toggle learning
* `a`     toggle alternating mode (if true, detector is switched off when tracker is available)
* `e`     export model
* `i`     import model
* `r`     clear model
*  F5     refreshes reference image for user bounding box definition
*  Enter  starts tracking after bounding boxes are drawn on the stereo image

Like OpenTLD, ROS_OpenTLD is published under the terms of the GNU General Public License.

This repo is heavily borrowed from: 

https://github.com/Ronan0912/ros_opentld
IntRoLab
http://introlab.3it.usherbrooke.ca
Université de Sherbrooke, Québec, Canada
