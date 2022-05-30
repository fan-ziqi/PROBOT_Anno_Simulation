#ifndef PROBOT_GRASPING_DEMO
#define PROBOT_GRASPING_DEMO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include "probot_grasping/vision_manager.h"



class GraspingDemo
{
  private:

	ros::NodeHandle nh_;

	geometry_msgs::Pose target_pose1;

	moveit::planning_interface::MoveGroupInterface arm_group;
	moveit::planning_interface::MoveGroupInterface gripper_group;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	image_transport::ImageTransport image_transport_;
	image_transport::Subscriber image_sub_;
	cv_bridge::CvImagePtr cv_ptr;
	VisionManager vision_manager_;

	tf::StampedTransform camera_to_robot_;
	tf::TransformListener tf_camera_to_robot;
	tf::Vector3 obj_camera_frame, obj_robot_frame;
	geometry_msgs::PoseStamped homePose;

	bool grasp_running;
	float initpos_x, initpos_y, initpos_z;

	/**
	 * @brief 到达指定位置
	 * 
	 * @param x 机器人坐标系下的x坐标
	 * @param y 机器人坐标系下的y坐标
	 * @param z 机器人坐标系下的z坐标
	 */
	void attainPosition(float x, float y, float z);
	/**
	 * @brief 靠近物体
	 */
	void AttainObject();
	/**
	 * @brief 夹取物体
	 */
	void grasp();
	/**
	 * @brief 移动一小段距离
	 * 
	 * @param color 
	 */
	void move(int color);

  public:
	 /**
	  * @brief 构造函数
	  * 
	  * @param n_ 			ros_NodeHandle
	  * @param initpos_x 	初始位置x
	  * @param initpos_y 	初始位置y
	  * @param initpos_z 	初始位置z
	  * @param table_length 桌子长度
	  * @param table_width 	桌子宽度
	  * @param color 		颜色
	  */
	GraspingDemo(ros::NodeHandle n_, float initpos_x, float initpos_y, float initpos_z, float table_length, float table_width, int color);
	/**
	 * @brief      imageCb is called when a new image is received from the camera
	 *
	 * @param[in]  msg   Image received as a message
	 */

	void imageCb(const int color, const sensor_msgs::ImageConstPtr &msg);


	/**
	 * @brief 执行抓取动作
	 * 
	 * @param color 颜色
	 */
	void ProcessGrasping(int color);
	/**
	 * @brief 机械臂回到初始位置
	 */
	void goHome();
};

#endif
