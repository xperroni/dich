DICH
====

The Difference Image Correspondence Hierarchy (DICH) is a an architecture for image-based teach-repeat navigation of differential drive robots. This

Build & Install
---------------

DICH is built using [catkin](http://wiki.ros.org/catkin). Type the commands below on a terminal window to create a catkin workspace, clone the repository and build the sources:

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    $ git clone https://github.com/xperroni/dich.git
    $ cd ..
    $ catkin_make

Usage
-----

Launch scripts in the `launch/` folder can be used for acquisition of teach records, computation of ground truth values, and performing replay trips.

A teach record is composed of a video file and a list of steering commands collected during the teach step. The `teach.launch` script will write a video file to the specified folder, but the steering commands file must be created out of the `teach` node log file. This is done using the `steerings.py` script, for example:

    $ python scripts/steering.py ~/.ros/log/latest/teach-4-stdout.log > steerings.txt