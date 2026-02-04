/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: viswa
Date: 31/01/2026

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("camera_link_frame_id", "camera_link");
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    map_frame_id_ = this->get_parameter("map_frame_id").as_string();
    camera_frame_id_ = this->get_parameter("camera_link_frame_id").as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        packagePath = "task_01_visual_odometry/"; //! Change to match path to your workspace
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    this->declare_parameter("visual_scale", 1.0);
    this->visual_scale_ = this->get_parameter("visual_scale").as_double();

    // Setup dynamic parameter callback
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : parameters) {
                if (param.get_name() == "visual_scale") {
                    try {
                        this->visual_scale_ = param.as_double();
                        RCLCPP_INFO(this->get_logger(), "Visual scale updated to: %f. Rescaling path...", this->visual_scale_);
                        
                        // Rescale existing path
                        trajectory_path_.poses.clear();
                        for (const auto& raw_pose : raw_poses_) {
                            geometry_msgs::msg::PoseStamped scaled_pose = raw_pose;
                            scaled_pose.pose.position.x *= this->visual_scale_;
                            scaled_pose.pose.position.y *= this->visual_scale_;
                            scaled_pose.pose.position.z *= this->visual_scale_;
                            trajectory_path_.poses.push_back(scaled_pose);
                        }
                        path_publisher_->publish(trajectory_path_);
                    } catch (const rclcpp::ParameterTypeException& e) {
                        result.successful = false;
                        result.reason = "visual_scale must be a double";
                    }
                }
            }
            return result;
        });

    // EuRoC T_BS (Body to Sensor) from sensor.yaml
    R_bs_ << 0.0148655429818, -0.999880929698, 0.00414029679422,
             0.999557249008, 0.0149672133247, 0.025715529948,
            -0.0257744366974, 0.00375618835797, 0.999660727178;
    t_bs_ << -0.0216401454975, -0.064676986768, 0.00981073058949;

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //* Initialize publishers
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    map_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/map_points", 10);

    //* Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    
    //* Publish Pose and TF
    if (pAgent->GetTrackingState() == ORB_SLAM3::Tracking::OK)
    {
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();

        // 0. Rotation from SLAM world to ROS world
        // SLAM default: Z-Forward, X-Right, Y-Down
        // ROS default: X-Forward, Y-Left, Z-Up
        // Mapping: Z_slam -> X_ros, X_slam -> -Y_ros, Y_slam -> -Z_ros
        Eigen::Matrix3f R_orb_to_ros;
        R_orb_to_ros << 0, 0, 1,
                       -1, 0, 0,
                        0,-1, 0;

        // Resulting Pose in ROS frame (Unscaled for storage)
        Eigen::Vector3f t_ros_raw = R_orb_to_ros * t;
        Eigen::Quaternionf q_ros = Eigen::Quaternionf(R_orb_to_ros * q.toRotationMatrix() * R_orb_to_ros.transpose());

        auto now = this->get_clock()->now();

        // 1. Publish PoseStamped (Scaled)
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = map_frame_id_;
        pose_msg.pose.position.x = t_ros_raw.x() * visual_scale_;
        pose_msg.pose.position.y = t_ros_raw.y() * visual_scale_;
        pose_msg.pose.position.z = t_ros_raw.z() * visual_scale_;
        pose_msg.pose.orientation.x = q_ros.x();
        pose_msg.pose.orientation.y = q_ros.y();
        pose_msg.pose.orientation.z = q_ros.z();
        pose_msg.pose.orientation.w = q_ros.w();
        pose_publisher_->publish(pose_msg);

        // Store Raw Pose for live rescaling
        geometry_msgs::msg::PoseStamped raw_pose = pose_msg;
        raw_pose.pose.position.x = t_ros_raw.x();
        raw_pose.pose.position.y = t_ros_raw.y();
        raw_pose.pose.position.z = t_ros_raw.z();
        raw_poses_.push_back(raw_pose);

        // 2. Publish Odometry (optional but good for consistency)
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = "base_link"; 
        odom_msg.pose.pose = pose_msg.pose;
        odom_publisher_->publish(odom_msg);

        // 3. Update and Publish Path
        trajectory_path_.header.stamp = now;
        trajectory_path_.header.frame_id = map_frame_id_;
        trajectory_path_.poses.push_back(pose_msg);
        path_publisher_->publish(trajectory_path_);

        // 3. Broadcast TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = map_frame_id_;
        tf_msg.child_frame_id = camera_frame_id_;
        tf_msg.transform.translation.x = pose_msg.pose.position.x;
        tf_msg.transform.translation.y = pose_msg.pose.position.y;
        tf_msg.transform.translation.z = pose_msg.pose.position.z;
        tf_msg.transform.rotation.x = q_ros.x();
        tf_msg.transform.rotation.y = q_ros.y();
        tf_msg.transform.rotation.z = q_ros.z();
        tf_msg.transform.rotation.w = q_ros.w();
        tf_broadcaster_->sendTransform(tf_msg);
    }
}


