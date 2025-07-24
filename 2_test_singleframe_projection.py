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
apriltag_image = original_image.copy()
_draw_pose_box(apriltag_image, camera_params, tag_size, calib['T_tag_wrt_camframe'])

cv2.imshow("Apriltag projection", apriltag_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

"""
Start projection tcp onto image
"""
K_matrix = np.eye(3)
K_matrix[0][0] = intrinsics_json["fx"]
K_matrix[1][1] = intrinsics_json["fy"]
K_matrix[0][2] = intrinsics_json["ppx"]
K_matrix[1][2] = intrinsics_json["ppy"]

p_tag_wrt_roboframe = calib['T_tag_wrt_roboframe'][:3, 3]
T_cam_wrt_roboframe = calib['T_tag_wrt_roboframe'] @ np.linalg.inv(calib['T_tag_wrt_camframe'])
T_tag_wrt_camframe = np.linalg.inv(T_cam_wrt_roboframe) @ calib['T_tag_wrt_roboframe']

p_tag_wrt_camframe = (np.linalg.inv(T_cam_wrt_roboframe) @ np.append(p_tag_wrt_roboframe, 1))[:3]

p_tag_wrt_imageplane, jacobian = cv2.projectPoints(p_tag_wrt_camframe, np.zeros(3), np.zeros(3), K_matrix, np.zeros(5))
p_tag_wrt_imageplane = p_tag_wrt_imageplane[0][0]

projection_image = original_image.copy()

_draw_pose_axes(projection_image, camera_params, tag_size, T_tag_wrt_camframe, p_tag_wrt_imageplane)
cv2.imshow("Projection test", projection_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
