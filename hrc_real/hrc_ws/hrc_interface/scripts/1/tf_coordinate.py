import numpy as np
import cv2

def pixel_to_3d(u, v, depth=1.223):
    pixels = np.array([[u, v]], dtype=np.float32)
    K = [913.650390625, 0.0, 955.0496215820312, 0.0, 913.8749389648438, 550.6069946289062, 0.0, 0.0, 1.0]
    D = [0.18453791737556458, -2.423478603363037, 0.00043306572479195893, -0.0002455342037137598, 1.5398979187011719, 0.0690656453371048, -2.238345146179199, 1.4565629959106445]
    camera_matrix = np.array(K).reshape((3, 3))
    dist_coeffs = np.array(D)
    undistorted_points = cv2.undistortPoints(pixels, camera_matrix, dist_coeffs)
    X = depth * undistorted_points[0][0][0] - 0.02
    Y = depth * undistorted_points[0][0][1]
    return X, Y

def transform_coords(x, y):
    tf_x = -y + (0.766 - 0.09) 
    tf_y = -x + (1.036 - 0.812)    
    sf = 0.07 if tf_y < 0 else 0
    tf_y -= (sf * tf_y)
    return tf_x, tf_y
