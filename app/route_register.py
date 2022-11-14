import os
import cv2 as cv

from submodule.mono_vslam_py_prototype.app.keyframe_on_route import KeyframeOnRoute

PIX_DISTNCE_THRESHOLD = 40

class RouteRegister:

    def __init__(self, route_dir, path_to_camera_mat):
        self.img_dir = route_dir
        self.img_num = 0

        self.last_frame = None
        self.path_to_camera_mat = path_to_camera_mat

    def update_frame(self, frame):

        if self.last_frame is None:
            self.regist_frame(frame)
            return

        self.last_frame.update(frame)

        if self.last_frame.value_available:
            if self.last_frame.pixel_distance.value() > PIX_DISTNCE_THRESHOLD:
                self.regist_frame(frame)

    def regist_frame(self, frame):
        path = '{}frame_{}.png'.format(self.img_dir, self.img_num)
        print('save frame to {}'.format(path))
        cv.imwrite(path, frame)
        self.img_num += 1

        self.last_frame = KeyframeOnRoute(frame, self.path_to_camera_mat)
     

