# realsense and dobot calibration

import pyrealsense2 as rs
# import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
from serial.tools import list_ports
from pydobot import Dobot
import os
import time


class Calibration:
    def __init__(self):
        # initialize dobot

        port = list_ports.comports()[0].device
        self.device = Dobot(port=port, verbose=True)
        (x, y, z, r, j1, j2, j3, j4) = self.device.pose()
        print("#################dobot pose")
        print("x: {}, y: {}, z: {}, r: {}, j1: {}, j2: {}, j3: {}, j4: {}".format(x, y, z, r, j1, j2, j3, j4))
        # move to calibration position
        self.device.move_to(x=200, y=0, z=0, r=-30, wait=False)

        # initialize realsense
        # Create a context object. This object owns the handles to all connected realsense devices
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 分别是宽、高、数据格式、帧率
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.align = rs.align(rs.stream.color)

        # Start streaming
        self.pipeline.start(config)

        # dictionary used in ArUco markers
        self.dictionary = aruco.Dictionary_get(aruco.DICT_5X5_50)
        # create parameters object
        self.parameters = aruco.DetectorParameters_create()

    def get_aruco_center(self, calib=True):
        # Create a pipeline object. This object configures the streaming camera and owns it's handle
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        # get depth frame
        depth = frames.get_depth_frame()

        # display color frame
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        # cv2.putText(color_image, "Status: Calibrating", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 获取intelrealsense参数
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        # 内参矩阵，转ndarray方便后续opencv直接使用
        intr_matrix = np.array([
            [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
        ])
        intr_coeffs = np.array(intr.coeffs)
        # 输入rgb图, aruco的dictionary, 相机内参, 相机的畸变参数
        corners, ids, rejected_img_points = aruco.detectMarkers(color_image, self.dictionary,
                                                                parameters=self.parameters)

        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.05, intr_matrix, intr_coeffs)

        center = None
        # if markers are detected
        if ids is not None:
            # draw borders around markers
            # aruco.drawDetectedMarkers(color_image, corners)
            # draw axis around markers, parameters: image, camera internal parameters, distortion parameters, rotation vector, translation vector, length of axis line
            # cv2.drawFrameAxes(color_image, intr_matrix, intr_coeffs, rvec, tvec, 0.05)
            # print ids and corners of detected markers
            for i, corner in zip(ids, corners):
                # get aruco center coordinate
                if calib:
                    x = (corner[0][0][0] + corner[0][3][0]) / 2
                    y = (corner[0][0][1] + corner[0][3][1]) / 2
                else:
                    x = (corner[0][0][0] + corner[0][2][0]) / 2
                    y = (corner[0][0][1] + corner[0][2][1]) / 2

                # cv2.circle(color_image, (int(x), int(y)), 3, (0, 0, 255), -1)

                # get middle pixel distance
                dist_to_center = depth.get_distance(int(x), int(y))
                # realsense提供的方法，将像素坐标转换为相机坐标系下的坐标
                x_cam, y_cam, z_cam = rs.rs2_deproject_pixel_to_point(intr, [x, y], dist_to_center)
                # display_txt = "x: {:.3f}, y: {:.3f}, z: {:.3f}".format(x_cam, y_cam, z_cam)
                # cv2.putText(color_image, "x: {:.3f}m".format(x_cam), (int(x) + 50, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1,
                #             (0, 255, 0), 2)
                # cv2.putText(color_image, "y: {:.3f}m".format(y_cam), (int(x) + 50, int(y) + 30),
                #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # cv2.putText(color_image, "z: {:.3f}m".format(z_cam), (int(x) + 50, int(y) + 60),
                #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # print(display_txt)

                center = [x_cam, y_cam, z_cam]

                # just need one marker
                break

        # depth frame
        depth_img = np.asanyarray(depth.get_data())
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.219), cv2.COLORMAP_JET)

        # stack color frame and depth frame
        # images = np.vstack((color_image, depth_colormap))
        return center

    def run_recog(self):
        if os.path.exists("./save_parms_test/image_to_arm.npy"):
            image_to_arm = np.load("./save_parms_test/image_to_arm.npy")
        # time.sleep(3)
        while True:
            center = self.get_aruco_center(calib=False)
            if center is not None:
                # cv2.imwrite("save.jpg", images)
                # cv2.imshow("image", images)
                # cv2.waitKey(1)
                img_pos = np.ones(4)
                img_pos[0:3] = center
                arm_pos = np.dot(image_to_arm, np.array(img_pos))
                # print(arm_pos)
                if (np.sqrt(arm_pos[0] * arm_pos[0] + arm_pos[1] * arm_pos[1]) > 320):
                    print("Can not reach!!!!!!!!!!!!!!!")
                    # time.sleep(1)
                    continue
                self.device.speed(100, 100)
                self.device.move_to(arm_pos[0]+10, arm_pos[1]-30, arm_pos[2] + 20, -30, wait=False)
                print("another one")


if __name__ == "__main__":
    cali = Calibration()
    cali.run_recog()

