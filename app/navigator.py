import os
import cv2 as cv

from submodule.mono_vslam_py_prototype.app.keyframe_on_route import KeyframeOnRoute
from submodule.mono_vslam_py_prototype.app.route_viewer import RouteViewer

MINIMUN_AVAILABLE_KEYFRAMES_NUM = 3

class Navigator:

    def __init__(self, route_dir):
        self.img_dir = route_dir
        self.next_img_num = 0
        self.did_finish = False

        self.keyframes_on_route = []
        self.routeViewer = RouteViewer()

        self.add_next_keyframe()

    def get_velocity_and_rotation(self, frame):
        self.update_keyframes_on_route(frame)

        self.routeViewer.update( self.keyframes_on_route )

        velocity = 0.0
        rotation = 0.0
        return velocity, rotation

    def update_keyframes_on_route(self, frame):

        # add new keyframe
        if self.keyframes_on_route[-1].keyframe_available:
            self.add_next_keyframe()

        [ keyframe.update(frame) for keyframe in self.keyframes_on_route ]
        is_keyframe_available = [ keyframe.keyframe_available for keyframe in self.keyframes_on_route ]

        if sum( is_keyframe_available ) > MINIMUN_AVAILABLE_KEYFRAMES_NUM\
        and not is_keyframe_available[0]:
            self.keyframes_on_route.pop(0)


    def add_next_keyframe(self):
        new_keyframe = self.get_next_keyframe()
        if new_keyframe is not None:
            self.keyframes_on_route.append( KeyframeOnRoute( new_keyframe ) )

    def get_next_keyframe(self):
        next_frame_str = '{}frame_{}.png'.format(self.img_dir, self.next_img_num)

        if not os.path.exists(next_frame_str):
            print('Not find next keyframe.')
            self.did_finish = True
            return

        print('update kfm to {}'.format(next_frame_str))

        try:
            self.next_img_num += 1
            return cv.imread(next_frame_str)
        except: #TODO: add error type
            print('cv.imread({}) has error.'.format(next_frame_str))
            return None

