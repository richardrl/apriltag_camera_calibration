# Install
- https://github.com/richardrl/AprilTag
	- Build C++ per instructions
	- Update python path PYTHONPATH=/home/mobilerobot/richard/AprilTag/scripts:$PYTHONPATH
- opencv
	- micromamba install -c conda-forge opencv
- pyrealsense2
	- pip install pyrealsense2
- ros/robotstack to use rviz and visualize tcp

# Setup viz
Replace the urdf STLs files with full absolute paths.

# Physical setup
Print small apriltags of family tag36h11. Print it in the gripper where the TCP is. 
Visualize the TCP using your robot arm URDF. 
Note: The numerical Apriltag pose given by the detector is a 180-deg body-frame rotation away from the visualized Apriltag: https://github.com/Tinker-Twins/AprilTag/blob/0b700ac421a04401911a302b1305c1a350b655f9/scripts/apriltag.py#L559


# LeRobot setup
git clone https://github.com/huggingface/lerobot/commit/a75d00970f55b48061909e1b7fa825e27bf