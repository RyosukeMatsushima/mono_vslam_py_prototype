#!/usr/bin/env python

'''
Local pose estimation
=================

Usage
-----

set_keyframe(frame)

get_pose(frame)
    return pose to keyframe

'''


# Python 2/3 compatibility
from __future__ import print_function
import sys
PY3 = sys.version_info[0] == 3

if PY3:
    xrange = range

import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2 as cv

# built-in modules
from collections import namedtuple

FLANN_INDEX_LSH    = 6
flann_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2

MIN_MATCH_COUNT = 10

'''
  image     - image to track
  rect      - tracked rectangle (x1, y1, x2, y2)
  keypoints - keypoints detected inside rect
  descrs    - their descriptors
  data      - some user-provided data
'''
KeyframeData = namedtuple('KeyframeData', 'image, keypoints, descrs')

'''
  pose   - pose to keyframe 
  p0     - matched points coords in key image
  p1     - matched points coords in input frame
  H      - homography matrix from p0 to p1
'''
PoseToKeyframe = namedtuple('PoseToKeyframe', 'pose, p0, p1, is_close')


class LocalPoseEstimator:

    def __init__(self):
        self.detector = cv.ORB_create( nfeatures = 1000 )
        self.matcher = cv.FlannBasedMatcher(flann_params, {})  # bug : need to pass empty dict (#1329)
        self.keyframe_data = None

    def set_keyframe(self, frame):
        self.matcher.clear()
        raw_points, raw_descrs = self.detect_features(frame)
        points, descs = [], []
        for kp, desc in zip(raw_points, raw_descrs):
            x, y = kp.pt
            points.append(kp)
            descs.append(desc)
        descs = np.uint8(descs)
        self.matcher.add([descs])
        self.keyframe_data = KeyframeData(image = frame, keypoints = points, descrs=descs)
        return

    def did_set_keyframe(self):
        return self.matcher.empty()

    def get_pose(self, frame):
        '''Returns a list of detected TrackedTarget objects'''
        frame_points, frame_descrs = self.detect_features(frame)
        if len(frame_points) < MIN_MATCH_COUNT:
            return []
        matches = self.matcher.knnMatch(frame_descrs, k = 2)
        matches = [m[0] for m in matches if len(m) == 2 and m[0].distance < m[1].distance * 0.70]
        if len(matches) < MIN_MATCH_COUNT:
            return []

        p0 = [self.keyframe_data.keypoints[m.trainIdx].pt for m in matches]
        p1 = [frame_points[m.queryIdx].pt for m in matches]
        p0, p1 = np.float32((p0, p1))

        R, t, F, p0, p1 = self.get_R_t_from_2d_points(p0, p1, 1000)
        if len(p0) < MIN_MATCH_COUNT:
            return []

        p2k = PoseToKeyframe(pose=[R, t], p0=p0, p1=p1, is_close=self.is_close(p0, p1))
        return p2k

    def get_R_t_from_2d_points(self, pts1, pts2, focal):
        F, mask = cv.findEssentialMat(pts1, pts2, focal=focal, pp=(0.0, 0.0), method=cv.RANSAC, prob=0.999, threshold=1.0)
        pts1 = pts1[mask.ravel()==1]
        pts2 = pts2[mask.ravel()==1]
        retval, R, t, mask = cv.recoverPose(F, pts1, pts2, focal=focal)
        available_p1 = [ p for i, p in enumerate(pts1) if mask[i][0] == 255 ]
        available_p2 = [ p for i, p in enumerate(pts2) if mask[i][0] == 255 ]
        return R, t, F, available_p1, available_p2

    def is_close(self, p0, p1):

        p0 = np.array(p0)
        p1 = np.array(p1)

        d_list = np.linalg.norm(p0 - p1, axis=1)

        if len(d_list) < MIN_MATCH_COUNT:
            return False

        return np.sort(d_list)[MIN_MATCH_COUNT - 1] < 1.0

    def detect_features(self, frame):
        '''detect_features(self, frame) -> keypoints, descrs'''
        keypoints, descrs = self.detector.detectAndCompute(frame, None)
        if descrs is None:  # detectAndCompute returns descs=None if not keypoints found
            descrs = []
        return keypoints, descrs


class App:
    def __init__(self, src, route_dir):
        self.cap = cv.VideoCapture(src)
        self.frame = None
        self.localPoseEstimator = LocalPoseEstimator()

        self.route_dir = route_dir
        self.save_frame_num = 0
        self.paused = False

        cv.namedWindow('plane')

    def set_img(self, img):
        self.localPoseEstimator.set_keyframe(img)

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            self.frame = frame.copy()

            vis = self.frame.copy()
            p2k = self.localPoseEstimator.get_pose(self.frame)
            if p2k:
                r = R.from_matrix(p2k.pose[0])
                if not self.paused:
                    print('')
                    print(r.as_euler('zyx', degrees=True))
                    print(p2k.pose[1])
                    print(p2k.is_close)

                for (x0, y0), (x1, y1) in zip(np.int32(p2k.p0), np.int32(p2k.p1)):
                    cv.circle(vis, (x1, y1), 2, (255, 255, 255))
                    cv.line(vis, (x0, y0), (x1, y1), (255, 255, 255))

            if not self.paused:
                cv.imshow('plane', vis)
            ch = cv.waitKey(1)
            if ch == ord(' '):
                self.localPoseEstimator.set_keyframe(self.frame)
            #if ch == ord('s'):
                path = '{}frame_{}.png'.format(self.route_dir, self.save_frame_num)
                print('save frame to {}'.format(path))
                cv.imwrite(path, self.frame)
                self.save_frame_num += 1
            if ch == ord('p'):
                self.paused = not self.paused
            if ch == 27:
                break

if __name__ == '__main__':
    print(__doc__)

    import sys
    try:
        video_src = sys.argv[1]
    except:
        video_src = 0
    try:
        route_dir = sys.argv[2]
    except:
        route_dir = './route_imgs/'

    App(video_src, route_dir).run()
