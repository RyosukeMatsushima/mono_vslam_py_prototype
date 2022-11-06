import numpy as np

from submodule.mono_vslam_py_prototype.app.local_pose_estimator import LocalPoseEstimator
from submodule.mono_vslam_py_prototype.app.average_value import AverageValue

AVAILABLE_FRAME_THRESHOLD = 3
AVERAGE_RANGE = 3

class KeyframeOnRoute:

    def __init__(self, keyframe):

        self.keyframe = keyframe

        self.keyframe_yaw = AverageValue(AVERAGE_RANGE)
        self.yaw_to_keyframe = AverageValue(AVERAGE_RANGE)
        self.pixel_distance = AverageValue(AVERAGE_RANGE)

        self.localPoseEstimator = LocalPoseEstimator()
        self.localPoseEstimator.set_keyframe(self.keyframe)

        self.keyframe_available = False
        self.value_available = False
        self.available_count = 0

        self.last_p2k = None


    def update(self, frame):

        p2k = self.localPoseEstimator.get_pose(frame)

        if p2k:
            self.update_available_status(True)
        else:
            self.update_available_status(False)
            return

        self.calculate_values(p2k)
        self.last_p2k = p2k


    def calculate_values(self, p2k):
        rotation_matrix = np.array(p2k.pose[0])
        frame_Z = rotation_matrix @ np.array([0, 0, 1])
        keyframe_direction = np.array(p2k.pose[1]).T[0]

        keyframe_yaw = self.vector2d_to_angle(frame_Z[2], frame_Z[0])
        self.keyframe_yaw.update( keyframe_yaw )
        self.yaw_to_keyframe.update( self.vector2d_to_angle( keyframe_direction[2], keyframe_direction[0] ) + keyframe_yaw )
        self.pixel_distance.update( p2k.distance )


    def vector2d_to_angle(self, x, y):
        alpha  = np.sqrt( 1 / (x**2 + y**2) )
        sign = 1.0 if y > 0.0 else -1.0
        return np.arccos(x * alpha) * sign


    def update_available_status(self, frame_available):

        self.value_available = frame_available

        v = 1 if frame_available else -1
        self.available_count += v

        if self.available_count > AVAILABLE_FRAME_THRESHOLD:
            self.keyframe_available = True
            self.available_count = AVAILABLE_FRAME_THRESHOLD
            return

        if self.available_count < 0:
            self.keyframe_available = False
            self.available_count = 0

