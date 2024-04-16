# realsense and dobot calibration

import pyrealsense2 as rs
# import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
# from serial.tools import list_ports
# from pydobot import Dobot
import os
import time


# initialize realsense
# Create a context object. This object owns the handles to all connected realsense devices
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 分别是宽、高、数据格式、帧率
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
align = rs.align(rs.stream.color)

# Start streaming
pipeline.start(config)

# dictionary used in ArUco markers
dictionary = aruco.Dictionary_get(aruco.DICT_5X5_50)
# create parameters object
parameters = aruco.DetectorParameters_create()

def get_aruco_center(calib=True):
    # Create a pipeline object. This object configures the streaming camera and owns it's handle
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)
    # get depth frame
    depth = frames.get_depth_frame()

    # display color frame
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    cv2.putText(color_image, "Status: Calibrating", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 获取intelrealsense参数
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    # 内参矩阵，转ndarray方便后续opencv直接使用
    intr_matrix = np.array([
        [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
    ])
    intr_coeffs = np.array(intr.coeffs)
    # 输入rgb图, aruco的dictionary, 相机内参, 相机的畸变参数
    corners, ids, rejected_img_points = aruco.detectMarkers(color_image, dictionary, parameters=parameters)

    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.05, intr_matrix, intr_coeffs)

    center = None
    # if markers are detected
    if ids is not None:
        # draw borders around markers
        aruco.drawDetectedMarkers(color_image, corners)
        # draw axis around markers, parameters: image, camera internal parameters, distortion parameters, rotation vector, translation vector, length of axis line
        print('rvec:',rvec.shape)
        cv2.drawFrameAxes(color_image, intr_matrix, intr_coeffs, rvec[0], tvec[0], 0.05) # 注意这里旋转平移矩阵取了第一个，原代码没有取，应该会报错
        # print ids and corners of detected markers
        for i, corner in zip(ids, corners):
            # get aruco center coordinate
            if calib:
                x = (corner[0][0][0] + corner[0][3][0]) / 2
                y = (corner[0][0][1] + corner[0][3][1]) / 2
            else:
                x = (corner[0][0][0] + corner[0][2][0]) / 2
                y = (corner[0][0][1] + corner[0][2][1]) / 2

            cv2.circle(color_image, (int(x), int(y)), 3, (0, 0, 255), -1)

            # get middle pixel distance
            dist_to_center = depth.get_distance(int(x), int(y))
            # realsense提供的方法，将像素坐标转换为相机坐标系下的坐标
            x_cam, y_cam, z_cam = rs.rs2_deproject_pixel_to_point(intr, [x, y], dist_to_center)
            # display_txt = "x: {:.3f}, y: {:.3f}, z: {:.3f}".format(x_cam, y_cam, z_cam)
            cv2.putText(color_image, "x: {:.3f}m".format(x_cam), (int(x) + 50, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)
            cv2.putText(color_image, "y: {:.3f}m".format(y_cam), (int(x) + 50, int(y) + 30), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2)
            cv2.putText(color_image, "z: {:.3f}m".format(z_cam), (int(x) + 50, int(y) + 60), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2)
            # print(display_txt)

            center = [x_cam, y_cam, z_cam]

            # just need one marker
            break

    # depth frame
    depth_img = np.asanyarray(depth.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.219), cv2.COLORMAP_JET)

    # stack color frame and depth frame
    images = np.hstack((color_image, depth_colormap))
    return images, center


print("#################please put the aruco marker on the dobot end effector")
time.sleep(5)

# define move points, x, y, z, r
# default_cali_points = [
#     [180, -150, -60, 0], [220, -80, -60, 0],
#     [230, 0, -60, 0],
#     [285, 0, -60, 0],
#     [260, -100, -60, 0], [220, -180, -60, 0],
#     [250, -190, -60, 0], [290, -110, -60, 0],
#     [305, 0, -60, 0],
#
#     [230, 0, -40, 0],
#     [220, -80, -40, 0], [180, -150, -40, 0],
#     [220, -180, -40, 0], [260, -100, -40, 0],
#     [285, 0, -40, 0],
#     [305, 0, -40, 0],
#     [290, -110, -40, 0], [250, -190, -40, 0],
#
#     [180, -150, -20, 0], [220, -80, -20, 0],
#     [230, 0, -20, 0],
#     [285, 0, -20, 0],
#     [260, -100, -20, 0], [220, -180, -20, 0],
#     [250, -190, -20, 0], [290, -110, -20, 0],
#     [305, 0, -20, 0],
#
#     [230, 0, 5, 0],
#     [220, -80, 0, 0], [180, -150, 0, 0],
#     [220, -180, 0, 0], [260, -100, 0, 0],
#     [285, 0, 0, 0],
#     [305, 0, 0, 0],
#     [290, -110, 0, 0], [250, -190, 0, 0],
#
#     [180, -150, 20, 0], [220, -80, 20, 0],
#     [230, 0, 20, 0],
#     [285, 0, 20, 0],
#     [260, -100, 20, 0], [220, -180, 20, 0],
#     [250, -190, 20, 0], [290, -110, 20, 0],
#     [305, 0, 20, 0],
#
#     [230, 0, 40, 0],
#     [220, -80, 40, 0], [180, -150, 40, 0],
#     [220, -180, 40, 0], [260, -100, 40, 0],
#     [285, 0, 40, 0],
#     [305, 0, 40, 0],
#     [290, -110, 40, 0], [250, -190, 40, 0], ]
#
# np_cali_points = np.array(default_cali_points)
# arm_cord = np.column_stack(
#     (np_cali_points[:, 0:3], np.ones(np_cali_points.shape[0]).T)).T
# centers = np.ones(arm_cord.shape)

index = -1
while True:
# for index, point in enumerate(default_cali_points):
#                 print("#################dobot move to point {}, x: {}, y: {}, z: {}, r: {}".format(index, point[0], point[1], point[2], point[3]))
                # move to the point

                # add x offset
                # arm_cord.T[index][0] = arm_cord.T[index][0] + 40 # +40 因为 aruco marker 的中心点距离end effector 30mm

                # get the position of the aruco marker
                index += 1
                images, center = get_aruco_center()
                if center is not None:
                    # save the center
                    # centers[0:3, index] = center
                    # display the image
                    cv2.imshow("image", images)
                    cv2.waitKey(1)
                else:
                    print("no aruco marker detected")
                    continue

                # time.sleep(1)


