import pyrealsense2 as rs
import numpy as np
import cv2 as cv

class RealSenseRos:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None
        self.intr = None
        self.w = 848
        self.h = 480
        self.fps = 60

        self.decimation_filter = rs.decimation_filter(2)
        self.hole_filter = rs.hole_filling_filter(2)
        self.spatial_filter = rs.spatial_filter(0.4, 21, 2, 4)
        self.temporal_filter = rs.temporal_filter(0.40, 31, 3)
        self.threshold_filter = rs.threshold_filter(0.1, 1.5)
    
    def initialize_rs(self):
        self.config.enable_stream(rs.stream.depth,self.w, self.h, rs.format.bgr8, self.fps)
        cfg = self.pipeline.start(self.config)
        profile = cfg.get_stream(rs.stream.color)
        self.intr = profile.as_video_stream_profile().get_intrinsics()

        device = cfg.get_device()
        advnc_mode = rs.rs400_advanced_mode(device)
        
        with open("../config/high_density_present.json") as file:
            json_config_str = file.read()
            advnc_mode.load_json(json_config_str)
        
        depth_sensor = device.first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)