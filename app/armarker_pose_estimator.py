
import numpy as np
import cv2 as cv
import sys
import argparse

from collections import namedtuple


PoseToKeyframe = namedtuple('PoseToKeyframe', 'pose, markerInfo')
MarkerInfo = namedtuple('MarkerInfo', 'rvec, tvec, corners')
Pose = namedtuple('Pose', 'rotMat, position')

class ARMakerPoseEstimator:

    def __init__(self, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        self.aruco_dict = cv.aruco.Dictionary_get(aruco_dict_type)
        self.matrix_coefficients = matrix_coefficients
        self.distortion_coefficients = distortion_coefficients

        self.current_frame_pose = None  # in armarker coordinate.
        self.key_frame_pose = None  # in armarker coordinate.

    def set_keyframe(self):
        if not self.current_frame_pose:
            return False

        self.key_frame_pose = self.current_frame_pose
        return True

    def get_pose_in_marker_coordinate(self, rvec, tvec):
        rot_mat = cv.Rodrigues(rvec)[0]
        rot_mat = np.linalg.inv(rot_mat)
        position = -rot_mat @ tvec[0][0]

        return Pose(rotMat=rot_mat, position=position)

    def get_key_frame_from_camera(self):
        if not self.current_frame_pose or not self.key_frame_pose:
            return

    def get_pose(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        parameters = cv.aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = cv.aruco.detectMarkers(gray, self.aruco_dict)

        # If only one marker is detected. Now reject multi makers.
        if len(corners) is not 1:
            self.current_frame_pose = None
            return

        rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners[0], 0.1, self.matrix_coefficients, self.distortion_coefficients)

        self.current_frame_pose = self.get_pose_in_marker_coordinate(rvec, tvec)

        markerInfo = MarkerInfo(rvec=rvec, tvec=tvec, corners=corners)

        if not self.key_frame_pose:
            return PoseToKeyframe(pose=None, markerInfo=markerInfo)

        rot_mat = np.linalg.inv(self.key_frame_pose.rotMat) @ self.current_frame_pose.rotMat
        position = self.key_frame_pose.position - self.current_frame_pose.position
        pose = Pose(rotMat=rot_mat, position=position)

        return PoseToKeyframe(pose=pose, markerInfo=markerInfo)


class App:
    def __init__(self, src, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        self.matrix_coefficients = matrix_coefficients
        self.distortion_coefficients = distortion_coefficients

        self.cap = cv.VideoCapture(src)
        self.frame = None
        self.paused = False
        self.arMakerPoseEstimator = ARMakerPoseEstimator(aruco_dict_type, matrix_coefficients, distortion_coefficients)

        cv.namedWindow('plane')

    def run(self):
        while True:
            if not self.paused:
                ret, frame = self.cap.read()
                if not ret:
                    break
                self.frame = frame.copy()

                vis = self.frame.copy()
                p2k = self.arMakerPoseEstimator.get_pose(self.frame)
                if p2k:

                    # Draw a square around the markers
                    cv.aruco.drawDetectedMarkers(vis, p2k.markerInfo.corners)

                    # Draw Axis
                    cv.drawFrameAxes(vis, self.matrix_coefficients, self.distortion_coefficients, p2k.markerInfo.rvec, p2k.markerInfo.tvec, 0.1)

                    if p2k.pose:
                        rot_mat = cv.Rodrigues(p2k.markerInfo.rvec)[0]
                        p =  rot_mat @ p2k.pose.position
                        cv.drawFrameAxes(vis, self.matrix_coefficients, self.distortion_coefficients, cv.Rodrigues((p2k.pose.rotMat.T))[0], p, 0.1)
                cv.imshow('plane', vis)
            ch = cv.waitKey(100)
            if ch == ord(' '):
                self.arMakerPoseEstimator.set_keyframe()
            if ch == ord('p'):
                self.paused = not self.paused
            if ch == 27:
                break

ARUCO_DICT = {
    "DICT_4X4_50": cv.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
}

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--Video_src", default=0, help="Video source selection")
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video_src = args["Video_src"]

    App(video_src, aruco_dict_type, k, d).run()

