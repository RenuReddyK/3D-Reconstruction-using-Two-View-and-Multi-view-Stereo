import numpy as np
import cv2


EPS = 1e-8


def backproject_corners(K, width, height, depth, Rt):
    """
    Backproject 4 corner points in image plane to the imaginary depth plane using intrinsics and extrinsics
    
    Hint:
    depth * corners = K @ T @ y, where y is the output world coordinates and T is the 4x4 matrix of Rt (3x4)

    Input:
        K -- camera intrinsics calibration matrix
        Rt -- 3 x 4 camera extrinsics calibration matrix
        width -- width of the image
        heigh -- height of the image
        depth -- depth value of the imaginary plane
    Output:
        points -- 2 x 2 x 3 array of 3D coordinates of backprojected points, here 2x2 correspinds to 4 corners
    """

    points = np.array(
        (
            (0, 0, 1),
            (width, 0, 1),
            (0, height, 1),
            (width, height, 1),
        ),
        dtype=np.float32,
    ).reshape(2, 2, 3)

    """ YOUR CODE HERE
    """
    R = Rt[:3,:3]
    t = Rt[:3,3]
    print(R.shape)
    t = t.reshape((3,1))
    points = points.reshape((4,3))
    print(t.shape)
    
    points = np.matmul( depth*np.linalg.inv(K) , points.T)    
    #points = points.reshape((3,1))                 
    points = np.matmul(R.T, (points - t)).T
    points = points.reshape((2,2,3))


    """ END YOUR CODE
    """
    return points


def project_points(K, Rt, points):
    """
    Project 3D points into a calibrated camera.
    
    Hint:
    Z * projections = K @ T @ p, where p is the input points and projections is the output, T is the 4x4 matrix of Rt (3x4)
    
    Input:
        K -- camera intrinsics calibration matrix
        Rt -- 3 x 4 camera extrinsics calibration matrix
        points -- points_height x points_width x 3 array of 3D points
    Output:
        projections -- points_height x points_width x 2 array of 2D projections
    """
    """ YOUR CODE HERE
    """
    R = Rt[:3,:3]
    t = Rt[:3,3].reshape((3,1))

    pointsX = np.zeros((points.shape[0], points.shape[1], 2))
    for i in range(pointsX.shape[0]):
        for j in range(pointsX.shape[1]):
            x = points[i,j]
            point = np.matmul(R,x).reshape((3,1)) + t
            point = np.matmul(K,point)
            pointsX[i,j,0] = point[0]/point[2]
            pointsX[i,j,1] = point[1]/point[2]

    
    """ END YOUR CODE
    """
    return pointsX


def warp_neighbor_to_ref(
    backproject_fn, project_fn, depth, neighbor_rgb, K_ref, Rt_ref, K_neighbor, Rt_neighbor
):
    """
    Warp the neighbor view into the reference view
    via the homography induced by the imaginary depth plane at the specified depth

    Make use of the functions you've implemented in the same file (which are passed in as arguments):
    - backproject_corners
    - project_points

    Also make use of the cv2 functions:
    - cv2.findHomography
    - cv2.warpPerspective
    
    ! Note, when you use cv2.warpPerspective, you should use the shape (width, height), NOT (height, width)
    
    Hint: you should do the follows:
    1.) apply backproject_corners on ref view to get the virtual 3D corner points in the virtual plane
    2.) apply project_fn to project these virtual 3D corner points back to ref and neighbor views
    3.) use findHomography to get teh H between neighbor and ref
    4.) warp the neighbor view into the reference view

    Input:
        backproject_fn -- backproject_corners function
        project_fn -- project_points function
        depth -- scalar value of the depth at the imaginary depth plane
        neighbor_rgb -- height x width x 3 array of neighbor rgb image
        K_ref -- 3 x 3 camera intrinsics calibration matrix of reference view
        Rt_ref -- 3 x 4 camera extrinsics calibration matrix of reference view
        K_neighbor -- 3 x 3 camera intrinsics calibration matrix of neighbor view
        Rt_neighbor -- 3 x 4 camera extrinsics calibration matrix of neighbor view
    Output:
        warped_neighbor -- height x width x 3 array of the warped neighbor RGB image
    """

    height, width = neighbor_rgb.shape[:2]
    

    """ YOUR CODE HERE
    """
    bp_c = backproject_fn(K_ref, width, height, depth, Rt_ref)
    p_nei = project_fn(K_neighbor, Rt_neighbor, bp_c).reshape((-1,2))
    p_ref = project_fn(K_ref, Rt_ref, bp_c).reshape((-1,2))
    H, x = cv2.findHomography(p_nei, p_ref)
    warped_neighbor = cv2.warpPerspective(neighbor_rgb, H, (width, height))


    """ END YOUR CODE
    """
    return warped_neighbor


def zncc_kernel_2D(src, dst):
    """
    Compute the cost map between src and dst patchified images via the ZNCC metric

    IMPORTANT NOTES:
    - Treat each RGB channel separately but sum the 3 different zncc scores at each pixel

    - When normalizing by the standard deviation, add the provided small epsilon value,
    EPS which is included in this file, to both sigma_src and sigma_dst to avoid divide-by-zero issues

    Input:
        src -- height x width x K**2 x 3
        dst -- height x width x K**2 x 3
    Output:
        zncc -- height x width array of zncc metric computed at each pixel
    """
    assert src.ndim == 4 and dst.ndim == 4
    assert src.shape[:] == dst.shape[:]

    """ YOUR CODE HERE
    """
    W_1 = np.mean(src, axis =2)
    W_2 = np.mean(dst, axis =2)
    sigma_W_1 = np.std(src, axis = 2)
    sigma_W_2 = np.std(dst, axis = 2)
    ZNCC = np.zeros((src.shape[0],src.shape[1],src.shape[3]))
    
    for i in range(src.shape[0]):
        for j in range(src.shape[1]):
            numerator0 = np.sum(np.multiply(src[i,j,:,0]-W_1[i,j,0], dst[i,j,:,0]-W_2[i,j,0]))
            denominator0 = (np.multiply(sigma_W_1[i,j, 0], sigma_W_2[i,j, 0]) + EPS)
            ZNCC[i,j,0] = numerator0/denominator0
            numerator1 = np.sum(np.multiply(src[i,j,:,1]-W_1[i,j,1], dst[i,j,:,1]-W_2[i,j,1]))
            denominator1 = (np.multiply(sigma_W_1[i,j, 1], sigma_W_2[i,j, 1]) + EPS)
            ZNCC[i,j,1] = numerator1/denominator1
            numerator2 = np.sum(np.multiply(src[i,j,:,2]-W_1[i,j,2], dst[i,j,:,2]-W_2[i,j,2]))
            denominator2 = (np.multiply(sigma_W_1[i,j, 2], sigma_W_2[i,j, 2]) + EPS)
            ZNCC[i,j,2] = numerator2/denominator2

    zncc = np.sum(ZNCC, axis = 2)
    """ END YOUR CODE
    """

    return zncc  # height x width


def backproject(dep_map, K):
    """
    Backproject image points to 3D coordinates wrt the camera frame according to the depth map

    Input:
        K -- camera intrinsics calibration matrix
        dep_map -- height x width array of depth values
    Output:
        points -- height x width x 3 array of 3D coordinates of backprojected points
    """
    _u, _v = np.meshgrid(np.arange(dep_map.shape[1]), np.arange(dep_map.shape[0]))

    """ YOUR CODE HERE
    """
    f = K[1,1] 
    xyz_cam = np.zeros((dep_map.shape[0], dep_map.shape[1], 3))
    for i in range(dep_map.shape[0]):
        for j in range(dep_map.shape[1]):
            caliberated_coord = np.linalg.inv(K)@np.array([j,i,1])
            xyz_cam[i,j,0] =  caliberated_coord[0] * dep_map[i,j]
            xyz_cam[i,j,1] = caliberated_coord[1] * dep_map[i,j]
            xyz_cam[i,j,2] = caliberated_coord[2] * dep_map[i,j]
           




    """ END YOUR CODE
    """
    return xyz_cam
