# for processing the dataset associated with the 3d hands project
import numpy as np

def state_to_ee_pose(tend_state):
	gripper_xyz = tend_state[:3]
	gripper_6d = tend_state[3:9]
	gripper_grip = tend_state[-1]
	homo_ph = np.eye(4)
	homo_ph[:3, 3] = gripper_xyz
	rx = gripper_6d[0:3]    
	ry = gripper_6d[3:6]
	rx = rx / np.linalg.norm(rx)
	ry = ry / np.linalg.norm(ry)
	rz = np.cross(rx, ry)
	homo_ph[:3, :3] = np.stack([rx, ry, rz])
	return homo_ph


def dataset_float_img_to_uint_img(dataset_float_img):
	# note does not convert rgb
	return (dataset_float_img.permute(1, 2, 0) * 255).data.cpu().numpy().astype(np.uint8)