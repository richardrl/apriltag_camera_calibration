import time

import pyrealsense2 as rs
import numpy as np
import cv2
from default_multi_realsense_cfg import get_default_multi_realsense_cfg

def enable_single_realsense(
    serial, ctx, resolution_width=640, resolution_height=480, frame_rate=30
):
    pipe = rs.pipeline(ctx)
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(
        rs.stream.depth, resolution_width, resolution_height, rs.format.z16, frame_rate
    )
    cfg.enable_stream(
        rs.stream.color, resolution_width, resolution_height, rs.format.rgb8, frame_rate
    )
    pipe.start(cfg)

    return pipe



def get_rgbd_rs(pipe) -> dict:
   align_to = rs.stream.color
   align = rs.align(align_to)
   try:
    # Get frameset of color and depth
    frames = pipe.wait_for_frames(100)  # 100
    # frames = pipe.wait_for_frames(100)
   except RuntimeError as e:
    print(f"Runtime error: {e}")
    print(
        f"Couldn't get frame for device: {pipe.get_active_profile().get_device()}"
    )
    # continue
    raise
   # Align the depth frame to color frame
   aligned_frames = align.process(frames)
   # Get aligned frames
   aligned_depth_frame = aligned_frames.get_depth_frame()
   color_frame = aligned_frames.get_color_frame()
   if not color_frame:
        print("No color frame received")
   depth_image = np.asanyarray(aligned_depth_frame.get_data())
   color_image = np.asanyarray(color_frame.get_data())
   # the .copy() here is super important!
   img_dict = dict(rgb=color_image.copy(), depth=depth_image.copy())
   return img_dict
   
class RealsenseImageGenerator():
    def __init__(self, camera_serials):
        self.camera_serials = camera_serials

        rs_cfg = get_default_multi_realsense_cfg()
        resolution_width = rs_cfg.WIDTH  # pixels
        resolution_height = rs_cfg.HEIGHT  # pixels
        frame_rate = rs_cfg.FRAME_RATE  # fps
        ctx = rs.context()

        # self.image_width, self.image_height = 256, 256
        # self.image_width, self.image_height = 448, 252
        self.image_width, self.image_height = 1280, 720

        image_pipelines = []
        for serial in camera_serials:
            pipeline = enable_single_realsense(
                serial, ctx, resolution_width, resolution_height, frame_rate
            )
            image_pipelines.append(pipeline)
            time.sleep(1.0)
        self.image_pipelines = image_pipelines

    def get_images(self, show_image=False):
        out_imgs = []
        for i, pipe in enumerate(self.image_pipelines, start=1):
            # print(f"Showing image {i}")
            img = get_rgbd_rs(pipe)
            # if self.resize_images:
            img["rgb"] = cv2.resize(img["rgb"], (self.image_width, self.image_height))
            img["depth"] = cv2.resize(img["depth"], (self.image_width, self.image_height))

            if show_image:
                cv2.imwrite(f"rgb{i}.png", cv2.cvtColor(img["rgb"], cv2.COLOR_BGR2RGB))

                # the input into waitkey is the number of milliseconds to wait before displaying the next image
                # if it is too short, the image will be black
                # cv2.waitKey(100)

            out_imgs.append(img)
        return out_imgs