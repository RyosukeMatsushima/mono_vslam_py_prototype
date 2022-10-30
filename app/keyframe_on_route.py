
from submodule.mono_vslam_py_prototype.app.local_pose_estimator import LocalPoseEstimator

AVAILABLE_FRAME_THRESHOLD = 10

class KeyframeOnRoute:

    def __init__(self, keyframe):

        self.keyframe_yaw = None
        self.yaw_to_keyframe = None
        self.pixel_distance = None

        self.localPoseEstimator = LocalPoseEstimator()
        self.localPoseEstimator.set_keyframe(keyframe)

        self.keyframe_available = False
        self.value_available = False
        self.available_count = 0


    def update(self, frame):

        p2k = self.localPoseEstimator.get_pose(frame)

        if p2k:
            self.update_available_status(True)
        else:
            self.update_available_status(False)
            return

        self.calculate_values(self, p2k)


    def calculate_values(self, p2k):
        rotation_matrix = np.array(p2k.pose[0])
        frame_Z = rotation_matrix @ np.array([0, 0, 1])
        self.keyframe_yaw = self.vector2d_to_angle(frame_Z[2], frame_Z[0])

        keyframe_direction = np.array(p2k.pose[1]).T[0]
        self.yaw_to_keyframe = self.vector2d_to_angle( keyframe_direction[2], keyframe_direction[0] )
        self.pixel_distance = p2k.distance


    def update_available_status(self, frame_available):

        self.value_available = frame_available

        v = 1 if frame_available else -1
        self.available_count += v

        if self.available_count > AVAILABLE_FRAME_THRESHOLD:
            self.available = True
            self.available_count = AVAILABLE_FRAME_THRESHOLD

        if self.available_count < 0:
            self.keyframe_available = False
            self.available_count = 0

