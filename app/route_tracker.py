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

        if p2k:
            if p2k.is_close:
                self.update_keyframe()

        return p2k, self.did_finish

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
    def __init__(self, src, route_dir):
        self.cap = cv.VideoCapture(src)
        self.frame = None
        self.routeTracker = RouteTracker(route_dir)

        self.route_dir = route_dir
        self.save_frame_num = 0

    def run(self, web=False):
        cv_vis_images = []
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            self.frame = frame.copy()

            vis = self.frame.copy()
            p2k, did_finish = self.routeTracker.get_cmd(self.frame)
            if p2k:
                r = R.from_matrix(p2k.pose[0])

                for (x0, y0), (x1, y1) in zip(np.int32(p2k.p0), np.int32(p2k.p1)):
                    cv.circle(vis, (x1, y1), 2, (255, 255, 255))
                    cv.line(vis, (x0, y0), (x1, y1), (255, 255, 255))

            if web:
                cv_vis_file_name = 'cv_vis.png'
                cv.imwrite(cv_vis_file_name, vis)
                cv_vis_images.append(cv.imread(cv_vis_file_name))
                os.remove(cv_vis_file_name)
                if len(cv_vis_images) == 30:
                    break
            else:
                cv.imshow('plane', vis)
                ch = cv.waitKey(1)
                if ch == 27:
                    break
                if did_finish:
                    print('finish')
                    break

        if web:
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
        video_src = 0
    try:
        route_dir = sys.argv[2]
    except:
        route_dir = './route_imgs/'

    App(video_src, route_dir).run()
