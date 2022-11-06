import os
import cv2 as cv

from submodule.mono_vslam_py_prototype.app.keyframe_on_route import KeyframeOnRoute
from submodule.mono_vslam_py_prototype.app.route_viewer import RouteViewer
from submodule.mono_vslam_py_prototype.app.average_value import AverageValue

MINIMUN_AVAILABLE_KEYFRAMES_NUM = 1
MAXMUN_AVAILABLE_KEYFRAMES_NUM = 2
PIXEL_DISTANCE_THRESHOLD = 20

ANGLE_THRESHOLD = 3.1415926 / 180 * 45
MAX_VELOCITY = 0.3

VELOCITY_FOR_ROTATION = 0.10
ROTATION_GAIN = -0.5
MAX_ROTATION = 0.15

class Navigator:

    def __init__(self, route_dir, debug=False):
        self.img_dir = route_dir
        self.next_img_num = 0
        self.did_finish = False

        self.keyframes_on_route = []

        self.debug = debug
        if self.debug:
            self.routeViewer = RouteViewer()

        self.velocity = AverageValue(5)

        for _ in range(MAXMUN_AVAILABLE_KEYFRAMES_NUM):
            self.add_next_keyframe()

    def is_keyframe_available(self, keyframe):
        return keyframe.keyframe_available\
               and keyframe.pixel_distance.value() > PIXEL_DISTANCE_THRESHOLD\
               and keyframe.yaw_to_keyframe.value() < ANGLE_THRESHOLD\
               and keyframe.keyframe_yaw.value() < ANGLE_THRESHOLD

    def get_velocity_and_rotation(self, frame):

        [ keyframe.update(frame) for keyframe in self.keyframes_on_route ]

        if self.debug:
            self.routeViewer.update( self.keyframes_on_route )

        velocity = 0.0
        rotation = 0.0

        first_keyframe_available = self.is_keyframe_available( self.keyframes_on_route[0] )

        need_stop = False
        if not first_keyframe_available:
            second_keyframe_available = self.is_keyframe_available( self.keyframes_on_route[1] )
            if second_keyframe_available:
                self.set_next_keyframe()
            else:
                need_stop = True

        if need_stop:
            if self.keyframes_on_route[0].value_available:
                rotation = self.keyframes_on_route[0].keyframe_yaw.value()
            self.velocity.reset()
            self.velocity.update( VELOCITY_FOR_ROTATION )
            velocity = self.velocity.value()
        else:
            rotation = self.keyframes_on_route[0].yaw_to_keyframe.value()
            self.velocity.update( MAX_VELOCITY )
            velocity = self.velocity.value()

        rotation = max( -MAX_ROTATION, min( MAX_ROTATION, rotation * ROTATION_GAIN ) )

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
            elif abs( self.keyframes_on_route[0].yaw_to_keyframe.value() ) > ANGLE_THRESHOLD:
                self.keyframes_on_route.pop(0)
                print('pop')

    def set_next_keyframe(self):
        if self.add_next_keyframe():
            self.keyframes_on_route.pop(0)

    def add_next_keyframe(self):
        new_keyframe = self.get_next_keyframe()
        if new_keyframe is not None:
            self.keyframes_on_route.append( KeyframeOnRoute( new_keyframe ) )
            return True
        else:
            return False

    def get_next_keyframe(self):
        next_frame_str = '{}frame_{}.png'.format(self.img_dir, self.next_img_num)

        if not os.path.exists(next_frame_str):
            #print('Not find next keyframe.')
            self.did_finish = True
            return None

        print('update kfm to {}'.format(next_frame_str))

        try:
            self.next_img_num += 1
            return cv.imread(next_frame_str)
        except: #TODO: add error type
            print('cv.imread({}) has error.'.format(next_frame_str))
            return None

