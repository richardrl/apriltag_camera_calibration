from apriltag import _draw_pose_box, _draw_pose_axes
import cv2
import numpy as np
import json
import time
from scipy.spatial.transform import Rotation

tag_size = .01

original_image = cv2.imread("/home/mobilerobot/richard/apriltag_camera_calibration/calib_original_image.png")

with open('/home/mobilerobot/richard/apriltag_camera_calibration/calib_intrinsics.json', 'r') as file:
    intrinsics_json = json.load(file)

calib = np.load("/home/mobilerobot/richard/apriltag_camera_calibration/calib.npz")

camera_params=[intrinsics_json['fx'], intrinsics_json['fy'], intrinsics_json['ppx'], intrinsics_json['ppy']]


# this verifies that intrinsics and apriltag function properly
_draw_pose_box(original_image, camera_params, tag_size, calib['T_tag_wrt_camframe'])

while True:
    time.sleep(1/30)
    cv2.imshow("Apriltag projection", original_image)
    cv2.waitKey(1)