from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rtabmap_ros",
                executable="rgbd_odometry",
                output="screen",
                parameters=[
                    {
                        "frame_id": "ego_vehicle",      # our base_link
                        "publish_tf": True,             # ~publish_tf (bool, default: "true") Publish TF from /odom to /base_link
                        "use_sim_time": False,
                        "Odom/Strategy": "1",           # 1= Frame 2 Frame
                        "wait_for_transform": 2.0,
                        "Reg/Force3DoF": "true",        # [Force 3 degrees-of-freedom transform (3Dof: x,y and yaw). Parameters z, roll and pitch will be set to 0.]
                        # "ground_truth_frame_id": "carla/ego_vehicle/odometry",  # (string, default: "")
                        "approx_sync": False,
                        "Odom/AlignWithGround": "true",  # [Align odometry with the ground on initialization.]
                        "queue_size": 15,
                        "Odom/Holonomic": "false",      # [If the robot is holonomic (strafing commands can be issued). If not, y value will be estimated from x and yaw values (y=x*tan(yaw)).]
                        "Odom/ResetCountdown": "1",     # [Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset).]
                        "ORB/EdgeThreshold": "19",      # [This is size of the border where the features are not detected. It should roughly match the patchSize parameter.]
                        "ORB/PatchSize": "31",          # [size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.]
                        "ORB/ScoreType": "0",           # [The default HARRIS_SCORE=0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.]
                        "ORB/WTA_K": "2",               # [The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).]
                        "Odom/GuessMotion": "true",     # [Guess next transformation from the last motion computed.]
                        "Stereo/Iterations": "30",      # 30 [Maximum iterations.]
                        "Stereo/MaxLevel": "3",         # [Maximum pyramid level.]
                        "Stereo/OpticalFlow": "false",  # true[Use optical flow to find stereo correspondences, otherwise a simple block matching approach is used.]
                        "Stereo/SSD": "false",          # [[Stereo/OpticalFlow = false] Use Sum of Squared Differences (SSD) window, otherwise Sum of Absolute Differences (SAD) window is used.]
                        "Stereo/WinHeight": "3",        # 3[Window height.]
                        "Stereo/WinWidth": "15",        # 15[Window width.]
                        "Stereo/MinDisparity": "1",     # [Minimum disparity.]
                        "Stereo/MaxDisparity": "256.0", # [Maximum disparity.]
                        # "Optimizer/Strategy" : "0",           # [Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres.]
                        # "Reg/Strategy" : "0",                 # [0=Vis, 1=Icp, 2=VisIcp]
                        "Vis/EstimationType": "1",      # [Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)]
                        "Vis/CorFlowMaxLevel": "5",
                        "Vis/BundleAdjustment": "0",    # [Optimization with bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.]
                        "Vis/FeatureType": "8",         # 2=ORB, 8=GFTT/ORB
                        "Stereo/DenseStrategy": "1",    # [0=cv::StereoBM, 1=cv::StereoSGBM]
                        "Vis/PnPFlags": "1",            # [[Vis/EstimationType = 1] PnP flags: 0=Iterative, 1=EPNP, 2=P3P]
                        "Vis/PnPRefineIterations": "1", # [[Vis/EstimationType = 1] Refine iterations. Set to 0 if "Vis/BundleAdjustment" is also used.]
                        "Vis/PnPReprojError": "1",      # [[Vis/EstimationType = 1] PnP reprojection error.]
                        "Vis/RefineIterations": "5",    # [[Vis/EstimationType = 0] Number of iterations used to refine the transformation found by RANSAC. 0 means that the transformation is not refined.]
                    }
                ],
                remappings=[
                    ("rgb/image", "/carla/ego_vehicle/rgb_left/image"),
                    ("depth/image", "/carla/ego_vehicle/depth_front/image"),
                    ("rgb/camera_info", "/carla/ego_vehicle/rgb_left/camera_info"),
                    ("/odom", "/vo/rgbd"),
                ],
                arguments=["--delete_db_on_start"],
            ),
        ]
    )
