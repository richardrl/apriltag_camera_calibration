# as a final test
# loads an mp4
# produces images with it
# for each image, project the TCP into the image using the extrinsics


# for now: loop over a dex dataset
import numpy as np
import time
import camera_util
import cv2
import argparse
import lerobot_util
import tqdm


calib = np.load("/home/mobilerobot/richard/apriltag_camera_calibration/calib.npz")
mp4_path = "/home/mobilerobot/lw_hfdata/jmarangola/garbage_test_k/videos/chunk-000/observation.image.ego_global/episode_000000.mp4"


# mp4_frames, vid_width, vid_height = camera_util.get_frames(mp4_path, 1)
T_cam_wrt_roboframe = calib['T_cam_wrt_roboframe']
rendered_out_frames = []
K_matrix = camera_util.get_K_matrix_from_intrinsics_json('/home/mobilerobot/richard/apriltag_camera_calibration/calib_intrinsics.json')

# put a keypoint over each frame
print("ln53")


parser = argparse.ArgumentParser(description="A simple script using argparse.")
parser.add_argument("lerobot_root", type=str, default="/home/mobilerobot/lw_hfdata/jmarangola/garbage_test_k")
parser.add_argument("lerobot_repo_id", type=str, default="jmarangola/garbage_test_k")

# Parse the arguments
args = parser.parse_args() #

"""
Setup dataset
"""
# https://huggingface.co/docs/lerobot/il_robots?replay=API+example
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, MultiLeRobotDataset
dataset = LeRobotDataset(root=args.lerobot_root, repo_id=args.lerobot_repo_id, episodes=[0])
"""
End setup dataset
"""
for frame_idx in tqdm.tqdm(range(dataset.num_frames), total=dataset.num_frames):
# for frame_idx, of in enumerate(mp4_frames):
    # crop to be square

    # resize

    # -> H, W, 3
    original_image_timestep = lerobot_util.dataset_float_img_to_uint_img(dataset[frame_idx]['observation.image.ego_global'])
    if frame_idx == 0:
        vid_width = original_image_timestep.shape[1]
        vid_height = original_image_timestep.shape[0]
    ee_pose_timestep = lerobot_util.state_to_ee_pose(dataset[frame_idx]['observation.state'])

    t0 = time.time()
    ee_in_uv = camera_util.get_ee_wrt_imageplane(ee_pose_timestep[:3, 3][np.newaxis, ...], T_cam_wrt_roboframe, K_matrix)[0]

    # print("ln74 projection time")
    # print(time.time() - t0)

    overlay = original_image_timestep.copy()
    cv2.circle(overlay, ee_in_uv.astype(np.int32), 10, (0, 0, 255), -1)
    rendered_out_frames.append(overlay)
print("ln72")

# Initialize video writer
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# writer = cv2.VideoWriter('output.avi', fourcc, 30, (vid_width, vid_height))
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
writer = cv2.VideoWriter('output.mp4', fourcc, 30, (vid_width, vid_height))

# frame = rendered_out_frames[0]
# cv2.imshow('Original (maybe BGR)', frame)
# cv2.waitKey(0)

# Write frames
for frame in rendered_out_frames:  # assuming frames is your list/iterator of numpy arrays
    writer.write(frame)  # frame should be uint8 BGR format

# Clean up
writer.release()