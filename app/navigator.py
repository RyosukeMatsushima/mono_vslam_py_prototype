import os
import cv2 as cv

from submodule.mono_vslam_py_prototype.app.keyframe_on_route import KeyframeOnRoute
from submodule.mono_vslam_py_prototype.app.route_viewer import RouteViewer

MINIMUN_AVAILABLE_KEYFRAMES_NUM = 2
MAXMUN_AVAILABLE_KEYFRAMES_NUM = 3

ANGLE_THRESHOLD = 3.1415926 / 180 * 30
MAX_VELOCITY = 2.0

MAX_ROTATION = 0.5
ROTATION_GAIN = 0.5

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
        for keyframe in self.keyframes_on_route:
            if not keyframe.value_available:
                continue

            need_stop = abs(keyframe.yaw_to_keyframe) > ANGLE_THRESHOLD

            rotation = keyframe.keyframe_yaw - keyframe.yaw_to_keyframe
            velocity = MAX_VELOCITY * max( 0.0, ( 1 - abs( rotation ) / ANGLE_THRESHOLD ) )

            if need_stop:
                rotation = keyframe.keyframe_yaw
                velocity = 0.0

            rotation = max( -MAX_ROTATION, min( MAX_ROTATION, rotation * ROTATION_GAIN ) )
            break

        return velocity, rotation

    def update_keyframes_on_route(self, frame):

        # add new keyframe
        if len(self.keyframes_on_route) < MAXMUN_AVAILABLE_KEYFRAMES_NUM:
            self.add_next_keyframe()

        [ keyframe.update(frame) for keyframe in self.keyframes_on_route ]
        is_keyframe_available = [ keyframe.keyframe_available for keyframe in self.keyframes_on_route ]

        if sum( is_keyframe_available ) > MINIMUN_AVAILABLE_KEYFRAMES_NUM:
            if not is_keyframe_available[0]:
                self.keyframes_on_route.pop(0)
                print('pop')
            elif abs( self.keyframes_on_route[0].yaw_to_keyframe ) > ANGLE_THRESHOLD:
                self.keyframes_on_route.pop(0)
                print('pop')

    def add_next_keyframe(self):
        new_keyframe = self.get_next_keyframe()
        if new_keyframe is not None:
            self.keyframes_on_route.append( KeyframeOnRoute( new_keyframe ) )

    def get_next_keyframe(self):
        next_frame_str = '{}frame_{}.png'.format(self.img_dir, self.next_img_num)

        if not os.path.exists(next_frame_str):
            #print('Not find next keyframe.')
            self.did_finish = True
            return

        print('update kfm to {}'.format(next_frame_str))

        try:
            self.next_img_num += 1
            return cv.imread(next_frame_str)
        except: #TODO: add error type
            print('cv.imread({}) has error.'.format(next_frame_str))
            return None

