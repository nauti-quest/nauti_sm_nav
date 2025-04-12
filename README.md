# NautiQuest State Machine

A ROS package that implements autonomous navigation using state machines and YOLO object detection. The system switches between lawn mowing patterns and bounding box following based on object detection.

## Build Instructions

```bash
# Clone the repository into your catkin workspace
cd ~/nauti_ws/src
git clone https://github.com/nauti-quest/nauti_sm_nav.git

# Build the package
cd ~/nauti_ws
catkin build
source devel/setup.bash
```

## Dependencies

- ROS Noetic
- yolo-ros from NautiQuest
- nauti-gazebo repository for running simulations
- Python 3.8+

## How to Run

1. Make sure all dependencies are installed and built
2. Make sure that the simulations and the yolo-ros node is running in the background.
3. Launch the state machine script using the command:
```bash
roslaunch nauti_sm_nav sm_nav.launch
```
4. Now, the state machine navigation should start running in 5 seconds after the launch. Do switch over to the Gazebo window to see the results

The system will:
1. Start in lawn mowing pattern to search for objects
2. Switch to bounding box following when an object of interest is detected
3. Return to lawn mowing if object is lost for more than 10 seconds
