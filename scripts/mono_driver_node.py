#!/usr/bin/env python3


"""
Python node for the MonocularMode cpp node.

Author: viswa
Date: 31/01/2026

Requirements
* Dataset must be configured in EuRoC MAV format
* Paths to dataset must be set before bulding (or running) this node
* Make sure to set path to your workspace in common.hpp

Command line arguments
-- settings_name: EuRoC, TUM2, KITTI etc; the name of the .yaml file containing camera intrinsics and other configurations
-- image_seq: MH01, V102, etc; the name of the image sequence you want to run

"""

# Imports
#* Import Python modules
import sys # System specific modules
import os # Operating specific functions
import glob
import time # Python timing module
import copy # For deepcopying arrays
import shutil # High level folder operation tool
from pathlib import Path # To find the "home" directory location
import argparse # To accept user arguments from commandline
import natsort # To ensure all images are chosen loaded in the correct order
import yaml # To manipulate YAML files for reading configuration files
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import numpy as np # Python Linear Algebra module
import cv2 # OpenCV

#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention

# Import ROS2 message templates
from sensor_msgs.msg import Image 
from std_msgs.msg import String, Float64 
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError 
import pandas as pd

#* Class definition
class MonoDriver(Node):
    def __init__(self, node_name = "mono_py_node"):
        super().__init__(node_name) # Initializes the rclpy.Node class. It expects the name of the node

        # Initialize parameters to be passed from the command line (or launch file)
        self.declare_parameter("settings_name","EuRoC")
        self.declare_parameter("image_seq","NULL")

        #* Parse values sent by command line
        self.settings_name = str(self.get_parameter('settings_name').value) 
        self.image_seq = str(self.get_parameter('image_seq').value)

        # DEBUG
        print(f"-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print()

        # Global path definitions
        # Try to find the package directory dynamically
        try:
            package_path = ament_index_python.packages.get_package_share_directory('ros2_orb_slam3')
            # Assuming the layout is .../install/ros2_orb_slam3/share/ros2_orb_slam3
            # We want the root of the workspace or where TEST_DATASET is.
            # In this case, TEST_DATASET seems to be in the root of the workspace.
            self.home_dir = os.path.abspath(os.path.join(package_path, "../../../.."))
        except ament_index_python.packages.PackageNotFoundError:
            self.home_dir = "/home/viswa/task_01_visual_odometry" 
        self.parent_dir = "TEST_DATASET" #! Change or provide path to the parent directory where data for all image sequences are stored
        self.image_sequence_dir = self.home_dir + "/" + self.parent_dir + "/" + self.image_seq # Full path to the image sequence folder

        print(f"self.image_sequence_dir: {self.image_sequence_dir}\n")

        # Global variables
        self.node_name = "mono_py_driver"
        self.image_seq_dir = ""
        self.imgz_seqz = []
        self.time_seqz = [] # Maybe redundant

        # Define a CvBridge object
        self.br = CvBridge()

        # Read images from the chosen dataset, order them in ascending order and prepare timestep data as well
        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = self.get_image_dataset_asl(self.image_sequence_dir, "mav0") 

        print(self.image_seq_dir)
        print(len(self.imgz_seqz))

        #* ROS2 publisher/subscriber variables [HARDCODED]
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings" 
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.send_config = True # Set False once handshake is completed with the cpp node
        
        #* Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1) # Publish configs to the ORB-SLAM3 C++ node

        #* Build the configuration string to be sent out
        #self.exp_config_msg = self.settings_name + "/" + self.image_seq # Example EuRoC/sample_euroc_MH05
        self.exp_config_msg = self.settings_name # Example EuRoC
        print(f"Configuration to be sent: {self.exp_config_msg}")


        #* Subscriber to get acknowledgement from CPP node that it received experimetn settings
        self.subscribe_exp_ack_ = self.create_subscription(String, 
                                                           self.sub_exp_ack_name, 
                                                           self.ack_callback ,10)
        self.subscribe_exp_ack_

        # Publisher to send RGB image
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)


        # Initialize work variables for main logic
        self.start_frame = 0 # Default 0
        self.end_frame = -1 # Default -1
        self.frame_stop = -1 # Set -1 to use the whole sequence, some positive integer to force sequence to stop, 350 test2, 736 test3
        self.show_imgz = False # Default, False, set True to see the output directly from this node
        self.frame_id = 0 # Integer id of an image frame
        self.frame_count = 0 # Ensure we are consistent with the count number of the frame
        self.inference_time = [] # List to compute average time

        # Ground truth related
        self.gt_path_publisher = self.create_publisher(Path, "/mono_py_driver/gt_path", 10)
        self.gt_path_msg = Path()
        self.gt_path_msg.header.frame_id = "map"
        self.gt_data = None
        self.T_gt_start_inv = None # To store the inverse of the starting GT pose for alignment
        self.load_ground_truth()

        print()
        print(f"MonoDriver initialized, attempting handshake with CPP node")
    # ****************************************************************************************

    # ****************************************************************************************
    def get_image_dataset_asl(self, exp_dir, agent_name = "mav0"):
        """
            Returns images and list of timesteps in ascending order from a ASL formatted dataset
        """
        
        # Define work variables
        imgz_file_list = []
        time_list = []

        #* Only works for EuRoC MAV format
        agent_cam0_fld = exp_dir + "/" + agent_name + "/" + "cam0"
        imgz_file_dir = agent_cam0_fld + "/" + "data" + "/"
        imgz_file_list = natsort.natsorted(os.listdir(imgz_file_dir),reverse=False)
        # print(len(img_file_list)) # Debug, checks the number of rgb images

        # Extract timesteps from image names
        for iox in imgz_file_list:
            time_step = iox.split(".")[0]
            time_list.append(time_step)
            #print(time_step)

        return imgz_file_dir, imgz_file_list, time_list
    # ****************************************************************************************

    # ****************************************************************************************
    def load_ground_truth(self):
        """
            Loads ground truth data from EuRoC format CSV
        """
        gt_file = os.path.join(self.image_sequence_dir, "mav0", "state_groundtruth_estimate0", "data.csv")
        if not os.path.exists(gt_file):
            print(f"Ground truth file not found: {gt_file}")
            return

        print(f"Loading ground truth from: {gt_file}")
        self.gt_data = pd.read_csv(gt_file, comment='#', header=None)
        # 0: timestamp, 1-3: px,py,pz, 4-7: qw,qx,qy,qz
        self.gt_data.columns = ['timestamp', 'px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz'] + list(self.gt_data.columns[8:])
        
        # Convert timestamps to seconds (EuRoC is in ns)
        self.gt_data['timestamp'] = self.gt_data['timestamp'] / 1e9

    def get_relative_gt_pose(self, timestamp):
        """
            Returns the GT pose relative to the start frame
        """
        if self.gt_data is None:
            return None

        # Find closest GT pose to current timestamp
        idx = (self.gt_data['timestamp'] - timestamp).abs().idxmin()
        row = self.gt_data.iloc[idx]

        # Construct transformation matrix T
        from scipy.spatial.transform import Rotation as R
        pos = np.array([row['px'], row['py'], row['pz']])
        quat = [row['qx'], row['qy'], row['qz'], row['qw']] # scipy uses x,y,z,w
        
        rot = R.from_quat(quat).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos

        # If this is the first frame, store its inverse for alignment
        if self.T_gt_start_inv is None:
            self.T_gt_start_inv = np.linalg.inv(T)

        # Align: T_rel = T_start_inv * T_now
        T_rel = self.T_gt_start_inv @ T
        return T_rel

    # ****************************************************************************************
    def ack_callback(self, msg):
        """
            Callback function
        """
        print(f"Got ack: {msg.data}")
        
        if(msg.data == "ACK"):
            self.send_config = False
            # self.subscribe_exp_ack_.destory() # TODO doesn't work 
    # ****************************************************************************************
    
    # ****************************************************************************************
    def handshake_with_cpp_node(self):
        """
            Send and receive acknowledge of sent configuration settings
        """
        if (self.send_config == True):
            # print(f"Sent mesasge: {self.exp_config_msg}")
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    # ****************************************************************************************
    
    # ****************************************************************************************
    def run_py_node(self, idx, imgz_name):
        """
            Master function that sends the RGB image message to the CPP node
        """

        # Initialize work variables
        img_msg = None # sensor_msgs image object

        # Path to this image
        img_look_up_path = self.imgz_seqz_dir  + imgz_name
        timestep = float(imgz_name.split(".")[0]) / 1e9
        self.frame_id = self.frame_id + 1  
        #print(img_look_up_path)
        # print(f"Frame ID: {frame_id}")

        # Based on the tutorials
        img_msg = self.br.cv2_to_imgmsg(cv2.imread(img_look_up_path), encoding="passthrough")
        timestep_msg = Float64()
        timestep_msg.data = timestep

        # Publish RGB image and timestep, must be in the order shown below. I know not very optimum, you can use a custom message interface to send both
        try:
            self.publish_timestep_msg_.publish(timestep_msg) 
            self.publish_img_msg_.publish(img_msg)
        except CvBridgeError as e:
            print(e)

        # Ground Truth Publishing
        gt_pose_mat = self.get_relative_gt_pose(timestep)
        if gt_pose_mat is not None:
            from scipy.spatial.transform import Rotation as R
            gt_pose_msg = PoseStamped()
            gt_pose_msg.header.stamp = self.get_clock().now().to_msg()
            gt_pose_msg.header.frame_id = "map"
            
            # Alignment logic:
            # EuRoC Body frame: Z-Forward, Y-Right, X-Up
            # ROS frame: X-Forward, Y-Left, Z-Up
            # We must map: Z_body -> X_ros, Y_body -> -Y_ros, X_body -> Z_ros
            R_body_to_ros = np.array([
                [0, 0, 1],
                [0, -1, 0],
                [1, 0, 0]
            ])
            
            # Rotate GT translation and rotation into ROS frame
            pos_ros = R_body_to_ros @ gt_pose_mat[:3, 3]
            rot_ros = R_body_to_ros @ gt_pose_mat[:3, :3] @ R_body_to_ros.T
            
            # Position
            gt_pose_msg.pose.position.x = pos_ros[0]
            gt_pose_msg.pose.position.y = pos_ros[1]
            gt_pose_msg.pose.position.z = pos_ros[2]
            
            # Orientation
            quat = R.from_matrix(rot_ros).as_quat() # x,y,z,w
            gt_pose_msg.pose.orientation.x = quat[0]
            gt_pose_msg.pose.orientation.y = quat[1]
            gt_pose_msg.pose.orientation.z = quat[2]
            gt_pose_msg.pose.orientation.w = quat[3]
            
            self.gt_path_msg.poses.append(gt_pose_msg)
            self.gt_path_msg.header.stamp = gt_pose_msg.header.stamp
            self.gt_path_publisher.publish(self.gt_path_msg)

    # ****************************************************************************************
        

# main function
def main(args = None):
    rclpy.init(args=args) # Initialize node
    n = MonoDriver("mono_py_node") #* Initialize the node
    rate = n.create_rate(20) # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    
    #* Blocking loop to initialize handshake
    while(n.send_config == True):
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        #self.rate.sleep(10) # Potential bug, breaks code

        if(n.send_config == False):
            break
        
    print(f"Handshake complete")

    #* Blocking loop to send RGB image and timestep message
    for idx, imgz_name in enumerate(n.imgz_seqz[n.start_frame:n.end_frame]):
        try:
            rclpy.spin_once(n) # Blocking we need a non blocking take care of callbacks
            n.run_py_node(idx, imgz_name)
            rate.sleep()

            # DEBUG, if you want to halt sending images after a certain Frame is reached
            if (n.frame_id>n.frame_stop and n.frame_stop != -1):
                print(f"BREAK!")
                break
        
        except KeyboardInterrupt:
            break

    # Cleanup
    cv2.destroyAllWindows() # Close all image windows
    n.destroy_node() # Release all resource related to this node
    rclpy.shutdown()

# Dunders, this .py is the main file
if __name__=="__main__":
    main()
