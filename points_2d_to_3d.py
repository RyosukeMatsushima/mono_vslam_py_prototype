import numpy as np
import cv2 as cv

#TODO: remove
from scipy.spatial.transform import Rotation as scipy_R

# input: mached points in 2d image and distance between each camera pose
# points1, points2: 2*N, int32, same_size
# distance
def points_2d_to_3d(pts1, pts2, distance, focal):

    R, t = get_R_t_from_2d_points(pts1, pts2, focal)

    print('R')
    print(scipy_R.from_dcm(R).as_euler('zyx', degrees=True))

    projection_mat1 = np.eye(3, 4)
    projection_mat2 = np.hstack((R, t * distance))

    pts1 = pts1.astype(np.float)
    pts2 = pts2.astype(np.float)
    points_3d = cv.triangulatePoints(projection_mat1, projection_mat2, pts1.T, pts2.T)

    #TODO: modify point_3d data shape
    return points_3d

def get_R_t_from_2d_points(pts1, pts2, focal):
    F, mask = cv.findEssentialMat(pts1, pts2, focal=focal, pp=(0.0, 0.0), method=cv.RANSAC, prob=0.999, threshold=1.0)

    pts1 = pts1[mask.ravel()==1]
    pts2 = pts2[mask.ravel()==1]

    retval, R, t, mask = cv.recoverPose(F, pts1, pts2, focal=focal);

    return R, t, F


