#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2 as cv
import os

from local_pose_estimator import LocalPoseEstimator

class RouteTracker:

    def __init__(self, route_dir):
        self.img_dir = route_dir
        self.img_num = 0
        self.did_finish = False

        self.localPoseEstimator = LocalPoseEstimator()
        self.update_keyframe()

    def get_cmd(self, frame):

        p2k = self.localPoseEstimator.get_pose(frame)

        if not p2k:
            return None

        if p2k.is_close:
            self.update_keyframe()

        rotation = self.calculate_rotation_cmd(p2k)

        return p2k, self.did_finish, rotation

    def calculate_rotation_cmd(self, p2k):
        rotation_matrix = np.array(p2k.pose[0])
        frame_direction = rotation_matrix @ np.array([0, 0, 1])
        keyframe_direction = np.array(p2k.pose[1]).T[0]

        rotation_vector = np.cross(frame_direction, keyframe_direction)

        rotation = rotation_vector[1]

        print('frame_direction: {}'.format(frame_direction))
        print('keyframe_direction: {}'.format(keyframe_direction))
        print('rotation_vector: {}'.format(rotation_vector))
        print('')

        return rotation


    def update_keyframe(self):
        next_frame_str = '{}frame_{}.png'.format(self.img_dir, self.img_num)

        if not os.path.exists(next_frame_str):
            print('Not find next keyframe.')
            self.did_finish = True
            return

        print('update kfm to {}'.format(next_frame_str))
        try:
            self.current_keyframe = cv.imread(next_frame_str)
        except:
            print('cv.imread({}) has error.'.format(next_frame_str))
            self.did_finish = True
            return

        self.localPoseEstimator.set_keyframe(self.current_keyframe)
        self.img_num += 1

class App:
    def __init__(self, src, route_dir, web=False):
        self.cap = cv.VideoCapture(src)
        self.frame = None
        self.routeTracker = RouteTracker(route_dir)

        self.route_dir = route_dir
        self.save_frame_num = 0

        self.web = web
        if not self.web:
            cv.namedWindow('plane')
        self.paused = False

    def run(self):
        cv_vis_images = []
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            self.frame = frame.copy()

            vis = self.frame.copy()
            p2k, did_finish, _ = self.routeTracker.get_cmd(self.frame)
            if p2k:
                r = R.from_matrix(p2k.pose[0])

                for (x0, y0), (x1, y1) in zip(np.int32(p2k.p0), np.int32(p2k.p1)):
                    cv.circle(vis, (x1, y1), 2, (255, 255, 255))
                    cv.line(vis, (x0, y0), (x1, y1), (255, 255, 255))
            else:
                print('no p2k')

            if self.web:
                cv_vis_file_name = 'cv_vis.png'
                cv.imwrite(cv_vis_file_name, vis)
                cv_vis_images.append(cv.imread(cv_vis_file_name))
                os.remove(cv_vis_file_name)
                if len(cv_vis_images) == 30:
                    break
            else:
                if not self.paused:
                    if p2k:
                        self.routeTracker.calculate_cmd(p2k)
                    cv.imshow('plane', vis)
                ch = cv.waitKey(1)
                if ch == 27:
                    break
                if ch == ord('p'):
                    self.paused = not self.paused
                if did_finish:
                    print('finish')
                    break

        if self.web:
            size = (640, 480)
            fps = 10.0
            os.makedirs('route_tracker_movies', exist_ok=True)
            cv_vis_movie = cv.VideoWriter('route_tracker_movies/output.webm', cv.VideoWriter_fourcc(*'VP90'), fps, size)
            for i in range(len(cv_vis_images)):
                cv_vis_movie.write(cv_vis_images[i])
            cv_vis_movie.release()

if __name__ == '__main__':

    import sys
    try:
        video_src = sys.argv[1]
    except:
        video_src = 2
    try:
        route_dir = sys.argv[2]
    except:
        route_dir = './route_imgs/'

    App(video_src, route_dir).run()
