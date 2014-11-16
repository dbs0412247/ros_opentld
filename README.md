Work in progress: 

- Stereo Version of OpenTLD 

Status: 
  Currently only supporting horizontal stereo.

Description: 
  This is a stereo implementation of OpenTLD, containing 3 nodes and requires stereo calibration inforation from left and right cameras' camera_info topic. The 3 nodes are: two tld_tracker nodes and one GUI node. Currently, the output from the stereo tracker is the triangulated distance of the tracked object from the left camera, shown on terminal.


Setup:

- git clone this repo to $(YOUR_CATKIN_WORKSPACE)/src
- catkin_make 
- Launch your (stereo) camera driver(s)
- Modify $(THIS_REPO_ROOT)/launch/ros_tld_left_tracker.launch, $(THIS_REPO_ROOT)/launch/ros_tld_right_tracker.launch and $(THIS_REPO_ROOT)/launch/ros_tld_stereoi_gui.launch to provide the correct image topics

How to run/use:

- roslaunch (all 3 of) the launch files modified in Setup, or;
- refer to ros_tld_stereo.launch for multiple nodes in a single launch 

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
