import unittest
import numpy as np
from scipy.spatial.transform import Rotation as scipy_R
import matplotlib.pyplot as plt

from points_2d_to_3d import *

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

class Points2Dto3DTest(unittest.TestCase):

    def setUp(self):
        xy_scale = 3
        z_scale = 3
        z_offset = 4
        self.fp_3d_org = [ np.random.rand(3) for i in range(100) ]
        self.fp_3d_org = [ np.array([(p[0] - 0.5) * xy_scale,
                                     (p[1] - 0.5) * xy_scale,
                                      p[2] * z_scale + z_offset])
                           for p in self.fp_3d_org ]

        self.camera_R_mat = scipy_R.from_euler('xyz', [10, 3, 0]).as_matrix()
        self.camera_t_mat = np.array([0.1, 0.0, 0.1])
 
        self.fp_3d_from_camera2 = [ np.dot(self.camera_R_mat, point) + self.camera_t_mat for point in self.fp_3d_org ]

        self.f = 3000 # focal length
        self.camera1_2d_points = [ [int(point[0]/point[2] * self.f),
                                    int(point[1]/point[2] * self.f)]
                                   for point in self.fp_3d_org ]
        self.camera2_2d_points = [ [int(point[0]/point[2] * self.f),
                                    int(point[1]/point[2] * self.f)]
                                   for point in self.fp_3d_from_camera2 ]

        self.camera1_2d_points = np.array(self.camera1_2d_points)
        self.camera2_2d_points = np.array(self.camera2_2d_points)

        self.d_cameras = np.linalg.norm(self.camera_t_mat)

        #show cameras img
        size = 20

        x, y = zip(*self.camera1_2d_points)
        _, _, z = zip(*self.fp_3d_org)
        s = [size] * len(z)
        c = [0.2] * len(z)
        plt.scatter(x, y, s=s, c=c, alpha=0.5)
        plt.show()

        x, y = zip(*self.camera2_2d_points)
        _, _, z = zip(*self.fp_3d_from_camera2)
        s = [size] * len(z)
        c = [0.3] * len(z)
        plt.scatter(x, y, s=s, c=c, alpha=0.5)
        plt.show()

    def tearDown(self):
        print("finish Points2Dto3DTest")

    def test_get_R_t_from_2d_points(self):
        print("test_get_R_t_from_2d_points")
        R, t, F = get_R_t_from_2d_points(self.camera1_2d_points, self.camera2_2d_points, self.f)
        F_ans = np.dot(skew(-self.camera_t_mat), self.camera_R_mat)
        print("R_ans")
        print(self.camera_R_mat)
        print("R")
        print(R)
        print("t_ans")
        print(self.camera_t_mat)
        print("t")
        print(t)
        print("F_ans")
        print(F_ans)
        print("F")
        print(F)
        print("check essential mat")
        print(np.dot(np.append(self.camera1_2d_points[0], 1.0), np.dot(F_ans, np.append(self.camera2_2d_points[0], 1.0))))
        print(self.camera_t_mat.T)

        #print(points_2d_to_3d(self.camera1_2d_points, self.camera2_2d_points, self.d_cameras, self.f).T)

if __name__ == "__main__":
    unittest.main()
