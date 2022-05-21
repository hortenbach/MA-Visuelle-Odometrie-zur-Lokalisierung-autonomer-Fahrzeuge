# Rapid VO

from socket import gaierror
import cv2 as cv
import numpy as np
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

# import DataHandler


# Initiate ORB detector
# max features
nFeatures = 100
orb = cv.ORB_create(nFeatures, scoreType=cv.ORB_FAST_SCORE)

def visualize_trajectory(trajectory):
    # Unpack X Y Z each trajectory point
    locX = []
    locY = []
    locZ = []
    # This values are required for keeping equal scale on each plot.
    # matplotlib equal axis may be somewhat confusing in some situations because of its various scale on
    # different axis on multiple plots
    max = -math.inf
    min = math.inf

    # Needed for better visualisation
    maxY = -math.inf
    minY = math.inf

    for i in range(0, trajectory.shape[1]):
        current_pos = trajectory[:, i]
        
        locX.append(current_pos.item(0))
        locY.append(current_pos.item(1))
        locZ.append(current_pos.item(2))
        if np.amax(current_pos) > max:
            max = np.amax(current_pos)
        if np.amin(current_pos) < min:
            min = np.amin(current_pos)

        if current_pos.item(1) > maxY:
            maxY = current_pos.item(1)
        if current_pos.item(1) < minY:
            minY = current_pos.item(1)

    auxY_line = locY[0] + locY[-1]
    if max > 0 and min > 0:
        minY = auxY_line - (max - min) / 2
        maxY = auxY_line + (max - min) / 2
    elif max < 0 and min < 0:
        minY = auxY_line + (min - max) / 2
        maxY = auxY_line - (min - max) / 2
    else:
        minY = auxY_line - (max - min) / 2
        maxY = auxY_line + (max - min) / 2

    # Set styles
    mpl.rc("figure", facecolor="white")
    plt.style.use("seaborn-whitegrid")

    # Plot the figure
    fig = plt.figure(figsize=(8, 6), dpi=100)
    gspec = gridspec.GridSpec(3, 3)
    ZY_plt = plt.subplot(gspec[0, 1:])
    YX_plt = plt.subplot(gspec[1:, 0])
    traj_main_plt = plt.subplot(gspec[1:, 1:])
    D3_plt = plt.subplot(gspec[0, 0], projection='3d')

    # Actual trajectory plotting ZX
    toffset = 1.06
    traj_main_plt.set_title("Autonomous vehicle trajectory (Z, X)", y=toffset)
    traj_main_plt.set_title("Trajectory (Z, X)", y=1)
    traj_main_plt.plot(locZ, locX, ".-", label="Trajectory", zorder=1, linewidth=1, markersize=4)
    traj_main_plt.set_xlabel("Z")
    # traj_main_plt.axes.yaxis.set_ticklabels([])
    # Plot reference lines
    traj_main_plt.plot([locZ[0], locZ[-1]], [locX[0], locX[-1]], "--", label="Auxiliary line", zorder=0, linewidth=1)
    # Plot camera initial location
    traj_main_plt.scatter([0], [0], s=8, c="red", label="Start location", zorder=2)
    traj_main_plt.set_xlim([min, max])
    traj_main_plt.set_ylim([min, max])
    traj_main_plt.legend(loc=1, title="Legend", borderaxespad=0., fontsize="medium", frameon=True)

    # Plot ZY
    # ZY_plt.set_title("Z Y", y=toffset)
    ZY_plt.set_ylabel("Y", labelpad=-4)
    ZY_plt.axes.xaxis.set_ticklabels([])
    ZY_plt.plot(locZ, locY, ".-", linewidth=1, markersize=4, zorder=0)
    ZY_plt.plot([locZ[0], locZ[-1]], [(locY[0] + locY[-1]) / 2, (locY[0] + locY[-1]) / 2], "--", linewidth=1, zorder=1)
    ZY_plt.scatter([0], [0], s=8, c="red", label="Start location", zorder=2)
    ZY_plt.set_xlim([min, max])
    ZY_plt.set_ylim([minY, maxY])

    # Plot YX
    # YX_plt.set_title("Y X", y=toffset)
    YX_plt.set_ylabel("X")
    YX_plt.set_xlabel("Y")
    YX_plt.plot(locY, locX, ".-", linewidth=1, markersize=4, zorder=0)
    YX_plt.plot([(locY[0] + locY[-1]) / 2, (locY[0] + locY[-1]) / 2], [locX[0], locX[-1]], "--", linewidth=1, zorder=1)
    YX_plt.scatter([0], [0], s=8, c="red", label="Start location", zorder=2)
    YX_plt.set_xlim([minY, maxY])
    YX_plt.set_ylim([min, max])

    # Plot 3D
    D3_plt.set_title("3D trajectory", y=toffset)
    D3_plt.plot3D(locX, locZ, locY, zorder=0)
    D3_plt.scatter(0, 0, 0, s=8, c="red", zorder=1)
    D3_plt.set_xlim3d(min, max)
    D3_plt.set_ylim3d(min, max)
    D3_plt.set_zlim3d(min, max)
    D3_plt.tick_params(direction='out', pad=-2)
    D3_plt.set_xlabel("X", labelpad=0)
    D3_plt.set_ylabel("Z", labelpad=0)
    D3_plt.set_zlabel("Y", labelpad=-2)
    
    # plt.axis('equal')
    D3_plt.view_init(45, azim=30)
    plt.tight_layout()
    plt.show()

class DataHandler:

    def __init__(self, start, stop):
        self.gray_img_A = []
        self.gray_img_B = []

        self.DIR = "/home/hortenbach/workspace/code_ws/pictures"

        for i in range(start, stop+0):
            print(i)
            self.gray_img_A.append(self.read_img_gray(i, 'left'))
            self.gray_img_B.append(self.read_img_gray(i, 'right'))

    def read_img(self, num, cam):
        img = cv.imread(
            f"/home/hortenbach/workspace/MA_v2/py/pic/image_{cam}_{num}.jpg")

        return img

    def read_img_gray(self, num, cam):
        gray = cv.imread(
            f"/home/hortenbach/workspace/MA_v2/py/pic/image_{cam}_{num}.jpg", 0)
        # gray = np.float31(gray)
        return gray


def feature_detection(img, detector):
    """ Detect Features """
    if detector == 'ORB':
        # find the keypoints with ORB
        kp = orb.detect(img, None)
    elif detector == 'GFTT':
        corners = cv.goodFeaturesToTrack(img, 25, 0.01, 10)
        kp = [cv.KeyPoint(pt[0][0], pt[0][1], 3) for pt in corners]
    return kp


def feature_extraction(img, kp):
    # compute the descriptors with ORB
    kp, des = orb.compute(img, kp)
    return [kp, des]


def feature_matching(img1, img2, kp1, kp2, des1, des2):
    # Using Flann with ORB Descriptors
    FLANN_INDEX_LSH = 6
    index_params = dict(algorithm=FLANN_INDEX_LSH,
                        table_number=6,  # 12
                        key_size=12,     # 20
                        multi_probe_level=1)  # 2
    search_params = dict(checks=50)   # or pass empty dictionary

    # flann = cv.FlannBasedMatcher(index_params,search_params)

    des1 = np.float32(des1)
    des2 = np.float32(des2)
    # matches = flann.knnMatch(des1,des2,k=2)

    # -- Step 2: Matching descriptor vectors with a FLANN based matcher
    # Since SURF is a floating-point descriptor NORM_L2 is used
    matcher = cv.DescriptorMatcher_create(cv.DescriptorMatcher_FLANNBASED)
    matches = matcher.knnMatch(des1, des2, 2)

    return matches


def outlier_removal(matches, kp1 ,kp2):
    pts1 = []
    pts2 = []
    print(f"Original {len(matches)} matches.")
    # ratio test as per Lowe's paper
    ratio_thresh = 0.8
    good_matches = []
    for m, n in matches:
        if m.distance < ratio_thresh * n.distance:
            good_matches.append([m])
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)
    print(f"good {len(good_matches)} matches.")
    return good_matches, pts1, pts2


def draw_matches(img1, img2, kp1, kp2, matches, good_matches, num=0):
    DIR = '/home/hortenbach/workspace/MA_v2/py/pic/matching'
    # -- Draw matches
    # all matches
    img_matches = cv.drawMatchesKnn(img1, kp1, img2, kp2, matches, None, flags=cv.DrawMatchesFlags_DEFAULT)
    cv.imwrite(f"{DIR}/image_flann_matches_{num}.jpg", img_matches)
    # good matches
    good_matches = cv.drawMatchesKnn(img1, kp1, img2, kp2, good_matches,
                      None, flags=cv.DrawMatchesFlags_DEFAULT)
    cv.imwrite(f"{DIR}/image_flann_good_matches_{num}.jpg", good_matches)

def calc_depthmap(img_A, img_B):
  DIR = '/home/hortenbach/workspace/MA_v2/py/pic/disparity'
  # E =  K'^T F K
  #Set disparity parameters
  #Note: disparity range is tuned according to specific parameters obtained through trial and error. 
  win_size = 5
  min_disp = -1
  max_disp = 31 #min_disp * 9
  num_disp = max_disp - min_disp # Needs to be divisible by 16
  #Create Block matching object. 
  stereo = cv.StereoSGBM_create(minDisparity= min_disp,
                numDisparities = num_disp,
                blockSize = 5,
                uniquenessRatio = 5,
                speckleWindowSize = 5,
                speckleRange = 5,
                disp12MaxDiff = 1,
                P1 = 8*3*win_size**2,#8*3*win_size**2,
                P2 =32*3*win_size**2) #32*3*win_size**2)
  #Compute disparity map
  disparity_map = stereo.compute(img_A, img_B)

#   #Show disparity map 
#   cv.imwrite(f"{DIR}/image_disparity_map_02.jpg", disparity_map)
  plt.imshow(disparity_map,'gray')
  plt.show()
  return disparity_map


def estimate_motion(match, kp1, kp2, k, depth1=None):
    """
    Estimate camera motion from a pair of subsequent image frames

    Arguments:
    match -- list of matched features from the pair of images
    kp1 -- list of the keypoints in the first image
    kp2 -- list of the keypoints in the second image
    k -- camera calibration matrix 
    
    Optional arguments:
    depth1 -- a depth map of the first frame. This argument is not needed if you use Essential Matrix Decomposition

    Returns:
    rmat -- recovered 3x3 rotation numpy matrix
    tvec -- recovered 3x1 translation numpy vector
    image1_points -- a list of selected match coordinates in the first image. image1_points[i] = [u, v], where u and v are 
                     coordinates of the i-th match in the image coordinate system
    image2_points -- a list of selected match coordinates in the second image. image1_points[i] = [u, v], where u and v are 
                     coordinates of the i-th match in the image coordinate system
               
    """
    rmat = np.eye(3)
    tvec = np.zeros((3, 1))
    image1_points = []
    image2_points = []
    
    objectpoints = []
    
    # Iterate through the matched features
    for m in match:
        m = m[0]
        # Get the pixel coordinates of features f[k - 1] and f[k]
        u1, v1 = kp1[m.queryIdx].pt
        u2, v2 = kp2[m.trainIdx].pt
        
        # Get the scale of features f[k - 1] from the depth map
        s = depth1[int(v1), int(u1)]
        
        # Check for valid scale values
        if s < 1000:
            # Transform pixel coordinates to camera coordinates using the pinhole camera model
            p_c = np.linalg.inv(k) @ (s * np.array([u1, v1, 1]))
            
            # Save the results
            image1_points.append([u1, v1])
            image2_points.append([u2, v2])
            objectpoints.append(p_c)
        
    # Convert lists to numpy arrays
    objectpoints = np.vstack(objectpoints)
    imagepoints = np.array(image2_points)
    
    # Determine the camera pose from the Perspective-n-Point solution using the RANSAC scheme
    _, rvec, tvec, _ = cv.solvePnPRansac(objectpoints, imagepoints, k, None)
    
    # Convert rotation vector to rotation matrix
    rmat, _ = cv.Rodrigues(rvec)
    
    return rmat, tvec, image1_points, image2_points


def pose_update():
    pass


def motion_prediction():
    pass


def keyframe():
    pass


if __name__ == "__main__":

    ## acquire images
    # handle = DataHandler.DataHandler()
    handle = DataHandler(350, 354)
    # handle.read(354,354)
    cv.waitKey(0)
    num = 0
    gray_A = iter(handle.gray_img_A)
    gray_B = iter(handle.gray_img_B)
    img_A_t0 = next(gray_A)
    img_A_t1 = next(gray_A)
    # cv.imshow('t0', img_A_t0)
    # cv.imshow('t1', img_A_t1)
    # cv.waitKey(0)
    img_B = next(gray_B)

    ## detect features in t0 and t+1 of left camera
    gftt_kp1 = feature_detection(img_A_t0, 'GFTT')
    gftt_kp2 = feature_detection(img_A_t1, 'GFTT')
    #orb_kp = feature_detection(img, 'ORB')

    # draw only keypoints location,not size and orientation
    # img2 = cv.drawKeypoints(img_A_t0, gftt_kp1, None, color=(0,255,0), flags=cv.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
    # img3 = cv.drawKeypoints(img_A_t1, gftt_kp2, None, color=(0,255,0), flags=cv.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
    # cv.imwrite(f"/home/hortenbach/workspace/MA_v2/py/pic/detection/image_gftt1_01.jpg", img2)
    # cv.imwrite(f"/home/hortenbach/workspace/MA_v2/py/pic/detection/image_gfft2_01.jpg", img3)

    ## extract gftt features into orb descriptors
    orb_kp1, des1 = feature_extraction(img_A_t0, gftt_kp1)
    orb_kp2, des2 = feature_extraction(img_A_t0, gftt_kp2)

    ## match feature descitpors of image t0  with image t+1
    matches = feature_matching(img_A_t0, img_A_t1, orb_kp1, orb_kp2, des1, des2)

    ## remove outliers and keep only good matches
    good_matches, pts1, pts2 = outlier_removal(matches, orb_kp1, orb_kp2)
    # draw_matches(img_A_t0, img_A_t1, orb_kp1, orb_kp2, matches, good_matches, num=2)

    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    F, mask = cv.findFundamentalMat(pts1,pts2,cv.FM_LMEDS)
    print("Fundamental Matrix F: \n", F)

    k = np.array([[640, 0, 640],
                  [0, 480, 480],
                  [0,   0,   1]], dtype=np.float32)

    ## for stereo camera images compute disparity map
    disparity_map = calc_depthmap(img_A_t0, img_A_t1) 

    rmat, tvec, image1_points, image2_points = estimate_motion(good_matches, orb_kp1, orb_kp2, k, depth1=disparity_map)

    print("R \n", rmat)
    print("t \n", tvec)

    R = rmat.transpose()
    pos = -R * tvec

    print("pos \n", pos)

    # roll = np.atan2(-R[2][1], R[2][2])
    # pitch = np.asin(R[2][0])
    # yaw = np.atan2(-R[1][0], R[0][0])
    i = 0
    # Create variables for computation
    trajectory = np.zeros((3, len(matches) + 1))
    robot_pose = np.zeros((len(matches) + 1, 4, 4))
    
    # Initialize camera pose
    robot_pose[0] = np.eye(4)

    # Determine current pose from rotation and translation matrices
    current_pose = np.eye(4)
    current_pose[0:3, 0:3] = rmat
    current_pose[0:3, 3] = tvec.T
    
    # Build the robot's pose from the initial position by multiplying previous and current poses
    robot_pose[i + 1] = robot_pose[i] @ np.linalg.inv(current_pose)
    
    # Calculate current camera position from origin
    position = robot_pose[i + 1] @ np.array([0., 0., 0., 1.])
    
    # Build trajectory
    trajectory[:, i + 1] = position[0:3]

    # visualize_trajectory(trajectory)