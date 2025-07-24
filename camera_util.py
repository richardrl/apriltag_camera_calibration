import numpy as np
import cv2
import json


def get_ee_wrt_imageplane(ee_wrt_robot_pos, cam_wrt_robot, K_matrix):
    """
    ee_wrt_robot_pos: N, 3

    """
    # takes points in robot frame
    # cam extrinsic in robot frame
    # intrinsc matrix
    # and returns the points in the camera plane
    # -> 4 -> 3D points in camera frame
    assert ee_wrt_robot_pos.shape[1] == 3, ee_wrt_robot_pos.shape
    assert len(ee_wrt_robot_pos.shape) == 2
    num_points = ee_wrt_robot_pos.shape[0]
    ee_wrt_cam = (np.linalg.inv(cam_wrt_robot) @ np.concatenate([ee_wrt_robot_pos, np.ones((num_points, 1))], -1).T)[:3]

    # project to 2D
    # ee_wrt_imageplane: num_points, 1, 2
    ee_wrt_imageplane, jacobian = cv2.projectPoints(ee_wrt_cam, np.zeros(3), np.zeros(3), K_matrix, np.zeros(5))

    # -> num_points, 2
    ee_in_uv = ee_wrt_imageplane[:, 0, :]
    return ee_in_uv


def get_K_matrix_from_intrinsics_json(intrinsics_json_path):
    with open(intrinsics_json_path, 'r') as file:
        intrinsics_json = json.load(file)

    K_matrix = np.eye(3)
    K_matrix[0][0] = intrinsics_json["fx"]
    K_matrix[1][1] = intrinsics_json["fy"]
    K_matrix[0][2] = intrinsics_json["ppx"]
    K_matrix[1][2] = intrinsics_json["ppy"]
    return K_matrix


def get_frames(mp4_path, frame_skip=10):
    out_frames = []
    cap = cv2.VideoCapture(mp4_path)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    i = 0
    # a variable to set how many frames you want to skip
    # a variable to keep track of the frame to be saved
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        if i % frame_skip == 0:
            out_frames.append(frame)
        i += 1

    cap.release()
    cv2.destroyAllWindows()
    return out_frames, width, height