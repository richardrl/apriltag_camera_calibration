import apriltag
import argparse
import cv2
import time
import pyrealsense2 as rs
import numpy as np

from utils import RealsenseImageGenerator
from default_multi_realsense_cfg import get_default_multi_realsense_cfg
import json
from scipy.spatial.transform import Rotation as R

camera_serial_numbers = ["215122255998"]
rsgi = RealsenseImageGenerator(camera_serial_numbers)

print("Intrinsics")
intrinsics_object = rsgi.image_pipelines[0].get_active_profile().get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

intrinsics_dict = dict(fx=intrinsics_object.fx,
                       fy=intrinsics_object.fy,
                       ppx=intrinsics_object.ppx,
                       ppy=intrinsics_object.ppy,
                       image_size=[1280, 720])
print("WARNING: SUBSEQUENT POSE ESTIMATION WILL BE WRONG IF YOU RESIZED THE IMAGE, BECAUSE INTRINSICS ARE WRONG!!!")

with open('calib_intrinsics.json', 'w') as f:
    json.dump(intrinsics_dict, f)
print(intrinsics_object)

"""
In order to separate the calibration and teleop code,
    In the teleop code, collect an episode, and end the episode once the object has been gripped properly and is inside the camera FOV, showing the apriltag at the TCP.
    Save the episode.
    The last frame of the episode has the RGB (with apriltag in it) and corresponding end effector pose.
    For calibration, we need the apriltag/TCP pose wrt to cam from apriltag detector and the apriltag/TCP pose wrt to robot (by applying TCP wrt end effector to the recorded end effector pose).
"""
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, MultiLeRobotDataset
# local dataset
dataset = LeRobotDataset(root="/home/mobilerobot/lw_hfdata/jmarangola/garbage_test_k", repo_id="jmarangola/garbage_test_k")
# dataset.push_to_hub()
color_image = (dataset[-1]['observation.image.ego_global'].permute(1, 2, 0) * 255).data.cpu().numpy().astype(np.uint8)

gripper_xyz = dataset[-1]['observation.state'][:3]
gripper_6d = dataset[-1]['observation.state'][3:9]
gripper_grip = dataset[-1]['observation.state'][-1]


options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())


gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
result, overlay = apriltag.detect_tags(color_image,
                                       detector,
                                       camera_params=[intrinsics_object.fx, intrinsics_object.fy, intrinsics_object.ppx, intrinsics_object.ppy],
                                       tag_size=0.01,
                                       vizualization=3,
                                       verbose=3,
                                       annotation=True)
if len(result) > 0:
    # result_pose = result[1]
    print("showing overlay")
    cv2.imshow('RealSense', overlay)
    cv2.waitKey(0)
    T_tag_wrt_camframe = result[1]
    print("T_tag_wrt_camframe")
    print(T_tag_wrt_camframe)


    """
    Pose of TCP in roboframe
    """
    homo_ph = np.eye(4)
    homo_ph[:3, 3] = gripper_xyz

    rx = gripper_6d[0:3]    
    ry = gripper_6d[3:6]
    rx = rx / np.linalg.norm(rx)
    ry = ry / np.linalg.norm(ry)
    rz = np.cross(rx, ry)
    homo_ph[:3, :3] = np.stack([rx, ry, rz])
    T_ee_wrt_roboframe = homo_ph

    print("T_ee_wrt_roboframe")
    print(T_ee_wrt_roboframe)

    print("pose_of_tag_in_roboframe")
    T_tcp_wrt_ee = np.eye(4)
    # 90 deg ccw rotation about the z of parent frame
    # 90 deg ccw rotation about the x axis of the new frame
    T_tcp_wrt_ee[:3, :3] = R.from_euler("X", 90, degrees=True).as_matrix() @ R.from_euler("Z", 90, degrees=True).as_matrix()
    T_tcp_wrt_ee[:3, 3] = np.array([0, 0, .1358])

    T_tcp_wrt_roboframe = T_ee_wrt_roboframe @ T_tcp_wrt_ee

    # this is equal to T_tag_wrt_roboframe

    """
    End get pose of TCP in roboframe
    """

    cv2.imwrite('calib_image.png', overlay)

    cv2.imwrite('calib_original_image.png', color_image)
    np.savez("calib.npz", T_tag_wrt_camframe=T_tag_wrt_camframe,
             T_ee_wrt_roboframe=T_ee_wrt_roboframe,
             T_tcp_wrt_roboframe=T_tcp_wrt_roboframe,
             T_tag_wrt_roboframe=T_tcp_wrt_roboframe)

    # apply a 90 deg CCW rotation along the Y
    # apply a 90 deg CCW rotation along Z
else:
    cv2.imshow('RealSense', color_image)
cv2.waitKey(1)

"""
Import the robotics controller code here. We will need to servo the robot into the view of the camera, record the end effector position, and use that for the calibration.
"""
# servo the robot controller to a location and record the joints

# grab the piece and servo it back

# the forward kinematics typically gives you the pose relative to some gripper frame, then we need to add the TCP offset
"""
End import
"""

# loop until we acquire an image
# while True:
#     time.sleep(1/30)

#     # convert from rgb back to bgr for opencv visualization
#     color_image = rsgi.get_images(False)[-1]['rgb'][:, :, ::-1]


#     options = apriltag.DetectorOptions(families="tag36h11")
#     # detector = apriltag.Detector(options)
#     detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())


#     gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
#     result, overlay = apriltag.detect_tags(color_image,
#                                            detector,
#                                            camera_params=[intrinsics_object.fx, intrinsics_object.fy, intrinsics_object.ppx, intrinsics_object.ppy],
#                                            tag_size=0.01,
#                                            vizualization=3,
#                                            verbose=3,
#                                            annotation=True
#                                            )

#     # retrieve pose of tag in camera frame
#     # retrieve pose of end effector (effectively pose of tag) in robot frame
#     # TODO: do some test to see that they are physically consistent

#     if len(result) > 0:
#         # result_pose = result[1]
#         print("showing overlay")
#         cv2.imshow('RealSense', overlay)
#         T_tag_wrt_camframe = result[1]

#         # robot_state = left_robot.get_state(robot_attr_map[Chirality.LEFT]['GripperInterface'])

#         # T_ee_wrt_roboframe = robot_state.ee_pose

#         # print("T_tag_wrt_camframe")
#         # print(T_tag_wrt_camframe)
#         # print("T_ee_wrt_roboframe")
#         # print(T_ee_wrt_roboframe)

#         cv2.imwrite('calib_image.png', overlay)

#         cv2.imwrite('calib_original_image.png', color_image)
#         # np.savez("calib.npz", T_tag_wrt_camframe=T_tag_wrt_camframe,
#         #          T_ee_wrt_roboframe=T_ee_wrt_roboframe)

#         # apply a 90 deg CCW rotation along the Y
#         # apply a 90 deg CCW rotation along Z
#     else:
#         cv2.imshow('RealSense', color_image)
#     cv2.waitKey(1)