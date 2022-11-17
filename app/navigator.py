import os
import cv2 as cv

from submodule.mono_vslam_py_prototype.app.keyframe_on_route import KeyframeOnRoute
from submodule.mono_vslam_py_prototype.app.route_viewer import RouteViewer
from submodule.mono_vslam_py_prototype.app.average_value import AverageValue

MINIMUN_AVAILABLE_KEYFRAMES_NUM = 1
MAXMUN_AVAILABLE_KEYFRAMES_NUM = 2
PIXEL_DISTANCE_THRESHOLD = 15

ANGLE_THRESHOLD = 3.1415926 / 180 * 10
MAX_VELOCITY = 0.3

VELOCITY_FOR_ROTATION = 0.0
ROTATION_GAIN = -0.5
MAX_ROTATION = 0.15
ROTATION_ON_THE_STOP = 0.15

class Navigator:

    def __init__(self, route_dir, path_to_camera_mat, debug=False):
        self.img_dir = route_dir
        self.path_to_camera_mat = path_to_camera_mat
        self.next_img_num = 0
        self.did_finish = False

        self.keyframes_on_route = []

        self.debug = debug
        if self.debug:
            self.routeViewer = RouteViewer()

        self.velocity = AverageValue(1)

        for _ in range(MAXMUN_AVAILABLE_KEYFRAMES_NUM):
            self.add_next_keyframe()

    def is_reached(self, keyframe):
        return keyframe.pixel_distance.value() < PIXEL_DISTANCE_THRESHOLD

    def is_keyframe_available(self, keyframe):
        valid_yaw_to_keyframe = abs(keyframe.yaw_to_keyframe.value()) < ANGLE_THRESHOLD
        valid_keyframe_yaw = abs(keyframe.keyframe_yaw.value()) < ANGLE_THRESHOLD

        if not keyframe.keyframe_available:
            print('not keyframe_available')
        if not valid_yaw_to_keyframe:
            print('not valid_yaw_to_keyframe: {}'.format(keyframe.yaw_to_keyframe.value()))
        if not valid_keyframe_yaw:
            print('not valid_keyframe_yaw: {}'.format(keyframe.keyframe_yaw.value()))

        return keyframe.keyframe_available\
               and valid_yaw_to_keyframe\
               and valid_keyframe_yaw

    def get_velocity_and_rotation(self, frame):

        [ keyframe.update(frame) for keyframe in self.keyframes_on_route ]

        if self.debug:
            self.routeViewer.update( self.keyframes_on_route )

        velocity = 0.0
        rotation = 0.0

        first_keyframe_available = self.is_keyframe_available( self.keyframes_on_route[0] )\
                                   and not self.is_reached( self.keyframes_on_route[0])

        need_stop = False
        if not first_keyframe_available:
            second_keyframe_available = self.keyframes_on_route[1].keyframe_available
            if second_keyframe_available:
                self.set_next_keyframe()
            else:
                need_stop = True

        if need_stop:
            self.velocity.reset()
            if self.keyframes_on_route[0].value_available:
                rotation = -ROTATION_ON_THE_STOP * self.keyframes_on_route[0].keyframe_yaw.value() / abs(self.keyframes_on_route[0].keyframe_yaw.value())
                self.velocity.update( VELOCITY_FOR_ROTATION )
                velocity = self.velocity.value()
        else:
            rotation = self.keyframes_on_route[0].yaw_to_keyframe.value()
            self.velocity.update( MAX_VELOCITY )
            velocity = self.velocity.value()
            rotation = max( -MAX_ROTATION, min( MAX_ROTATION, rotation * ROTATION_GAIN ) )


        return velocity, rotation

    def set_next_keyframe(self):
        if self.add_next_keyframe():
            self.keyframes_on_route.pop(0)

    def add_next_keyframe(self):
        new_keyframe = self.get_next_keyframe()
        if new_keyframe is not None:
            self.keyframes_on_route.append( KeyframeOnRoute( new_keyframe, self.path_to_camera_mat ) )
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

