#!/usr/bin/python

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2


def nothing(x):
    pass


def main():
    # Create a pipeline
    pipeline = rs.pipeline()
    # Create a config and configure the pipeline to stream
    config = rs.config()

    # Create a pipeline
    pipeline = rs.pipeline()
    # Create a config and configure the pipeline to stream
    config = rs.config()

    f1 = open("/tmp/cv_fifo", "w")
    with open("fifo_empty.txt", "w") as file:
        file.write("0")

    w = 848
    h = 480
    fps = 60
    config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)

    # Note in the example code, cfg is misleadingly called "profile" but cfg is a better name
    cfg = pipeline.start(config)
    profile = cfg.get_stream(rs.stream.color)
    intr = profile.as_video_stream_profile().get_intrinsics()

    # enable advanced mode and load parameters
    device = cfg.get_device()
    advnc_mode = rs.rs400_advanced_mode(device)
    print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

    with open("high_density_preset.json") as file:
        json_config_str = file.read()
        advnc_mode.load_json(json_config_str)

    # device_product_line = str(device.get_info(rs.camera_info.product_line))
    decimation_filter = rs.decimation_filter(2)
    hole_filter = rs.hole_filling_filter(2)
    spatial_filter = rs.spatial_filter(0.4, 21, 2, 4)
    temporal_filter = rs.temporal_filter(0.40, 31, 3)
    threshold_filter = rs.threshold_filter(0.1, 1.5)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    # print("Depth Scale is: " , depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    # clipping_distance_in_meters = 1 #1 meter
    # clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # opencv
    cv2.namedWindow('image')

    # create trackbars for color change
    cv2.createTrackbar('H1', 'image', 207, 360, nothing)
    cv2.createTrackbar('S1', 'image', 90, 255, nothing)
    cv2.createTrackbar('V1', 'image', 0, 255, nothing)

    cv2.createTrackbar('H2', 'image', 305, 360, nothing)
    cv2.createTrackbar('S2', 'image', 255, 255, nothing)
    cv2.createTrackbar('V2', 'image', 255, 255, nothing)

    # Streaming loop

    points = []

    try:
        while True:
            # get current positions of four trackbars
            h1 = cv2.getTrackbarPos('H1', 'image')
            s1 = cv2.getTrackbarPos('S1', 'image')
            v1 = cv2.getTrackbarPos('V1', 'image')

            h2 = cv2.getTrackbarPos('H2', 'image')
            s2 = cv2.getTrackbarPos('S2', 'image')
            v2 = cv2.getTrackbarPos('V2', 'image')

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            # aligned_depth_frame is a 640x480 depth image
            aligned_depth_frame = aligned_frames.get_depth_frame()
            depth_frame = decimation_filter.process(aligned_depth_frame)
            depth_frame = spatial_filter.process(depth_frame)
            depth_frame = temporal_filter.process(depth_frame)
            depth_frame = hole_filter.process(depth_frame)
            depth_frame = threshold_filter.process(depth_frame)
            color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            depth_image = cv2.resize(
                depth_image, (w, h), interpolation=cv2.INTER_AREA)
            color_image = np.asanyarray(color_frame.get_data())

            # convert the BGR image to HSV
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            # print(s)
            # print(type(s))

            lower_bound = np.array([h1/2, s1, v1])
            upper_bound = np.array([h2/2, s2, v2])

            # threshold the HSV image to get purple colors within our range
            mask = cv2.inRange(hsv, lower_bound, upper_bound)

            # bitwise-AND mask and original image
            # masked_image = cv2.bitwise_and(color_image, color_image, mask= mask)

            eroded_img = cv2.erode(mask, np.ones((4, 4)))
            dilated_img = cv2.erode(eroded_img, np.ones((4, 4)))

            closed_img = cv2.morphologyEx(dilated_img, cv2.MORPH_CLOSE, cv2.getStructuringElement(
                cv2.MORPH_RECT, (5, 5)), iterations=5)

            # drawing contours
            contours, hierarchy = cv2.findContours(
                closed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(color_image, contours, -1, (0, 255, 0), 3)

            depth_mask = cv2.bitwise_and(depth_image, depth_image, mask=mask)

            try:
                max_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(max_contour)

                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                depth_area = depth_mask[cy-5:cy+5, cx-5:cx+5]
                depth = depth_area.mean()

                point = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth)
                point = [point[2], point[0], -point[1]]

                points.append(point)

                if len(points) > 50:
                    points_array = np.array(points)
                    avg_x = np.mean(points_array[:, 0])
                    avg_y = np.mean(points_array[:, 1])
                    avg_z = np.mean(points_array[:, 2])

                    std_x = np.std(points_array[:, 0])
                    std_y = np.std(points_array[:, 1])
                    std_z = np.std(points_array[:, 2])

                    points.clear()

                    thresh = 30

                    with open("fifo_empty.txt", "r") as f2:
                        if std_x < thresh and std_y < thresh and std_z < thresh and f2.read() == "0":
                            point = [avg_x, avg_y, avg_z]
                            print("writing to fifo")
                            print(f"point: {point[0]}, {point[1]}, {point[2]}")
                            f1.write("")
                            f1.write(
                                str(point[0]) + "\n" + str(point[1]) + "\n" + str(point[2]) + "\n")
                            f1.flush()
                            f2.close()

                            with open("fifo_empty.txt", "w") as file:
                                file.write("1")

                text = f"(x: {round(point[0])}, y: {round(point[1])}, z: {round(point[2])})"

                color_image = cv2.putText(
                    color_image, text, (cx-100, cy+40), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

                cv2.circle(color_image, (cx, cy), 10, (255, 255, 255), -1)
            except:
                pass

            # print(f"moment: {M}")

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # cv2.imshow('original', hsv)
            # cv2.imshow('mask', mask)
            # cv2.imshow('depth', depth_colormap)
            cv2.imshow('image', color_image)

            key = cv2.waitKey(1) & 0xFF
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
