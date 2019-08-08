# Introduction

This is a OpenAI-gym extension for Gazebo Melodic, based on the deprecated project from [Erle Robotics: gym-gazebo](https://github.com/erlerobot/gym-gazebo)

Unlike the previous version, this one uses current and stable versions of all necessary programs.

# Requirements

- ROS Melodic Morenia :  [Install instructions](http://wiki.ros.org/melodic/Installation/Ubuntu), [wiki](http://wiki.ros.org/)

ROS Support Forum: <https://discourse.ros.org>

Gazebo Support Forum: <http://answers.gazebosim.org/questions>

- Ardupilot SITL 3.7.0 with ArduCopter (or APM:Copter) : [Install instructions](http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html), [code](https://github.com/ArduPilot/ardupilot), [wiki](http://ardupilot.org/copter/index.html)

Support Forum: <http://discuss.ardupilot.org/>

Community Site: <http://ardupilot.org>

Main developer wiki: <http://dev.ardupilot.org>

-Dronekit Python : [Install instructions](https://dronekit-python.readthedocs.io/en/latest/guide/quick_start.html#installation), [code](https://github.com/dronekit/dronekit-python), [wiki](https://dronekit-python.readthedocs.io/en/latest/guide/index.html)

- Theano : [Install instructions](http://deeplearning.net/software/theano/install_ubuntu.html)

- libgpuarray (or pygpu) : [Install instructions](http://deeplearning.net/software/libgpuarray/installation.html)

# Installation

Install some ROS-dependencies:
```cd gym-ws/gym-ws
./ros-dependencies.sh```

If this does not work try:
```chmod u+x ros-dependencies```
to make it executable and try it again.

In the root directory of the repository:

```bash
pip install -e .
```

if you have permission issues, you can try:

```bash
pip install -e . --user
```

or

```bash
sudo pip install -e .
```

# Usage

```bash
cd gym-ws/gym-ws/src
python dronekit_hover_qlearn.py
```

### Killing background processes

To end a simulation you should stop the python code with ctrl+c in your terminal. Sometimes `gzserver`, `gzclient` and `rosmaster` still working in the background. Make sure you end them before continuing, therefore you can create an alias in your `.bashrc` to kill these processes. After ending a simulation typ into a new tab oder terminal "kig" to use it.

```bash
echo "alias kig='killall -9 rosmaster gzserver gzclient rosout nodelet'" >> ~/.bashrc
```
