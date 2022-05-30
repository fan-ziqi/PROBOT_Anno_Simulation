#include "probot_grasping/grasping_demo.h"

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float initpos_x, float initpos_y, float initpos_z, float table_length, float table_width, int color) :
    image_transport_(n_), 
    arm_group("manipulator"), 
    gripper_group("gripper"), 
    vision_manager_(table_length, table_width)
{
  this->nh_ = n_;

 //获取base_link和camera_link之间的关系，也就是手眼标定的结果
  try
  {
    this->tf_camera_to_robot.waitForTransform("/base_link", 
                                              "/camera_link", 
                                              ros::Time(0), 
                                              ros::Duration(50.0));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(0.5).sleep();
  }

//如果查询得到的话，就将结果保存到camera_to_robot_，保存x,y,z和四元数一共7个值
  try
  {
    this->tf_camera_to_robot.lookupTransform("/base_link", 
                                             "/camera_link", 
                                             ros::Time(0), 
                                             this->camera_to_robot_);
  }

  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }

  grasp_running = false;
  
  this->initpos_x = initpos_x;
  this->initpos_y = initpos_y;
  this->initpos_z = initpos_z;


  //让机械臂运动到初始的位置
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //调用该函数控制机械臂运动到设定的位置
  ROS_INFO_STREAM("运动到初始位置");
  attainPosition(initpos_x, initpos_y, initpos_z);

  //订阅图像话题，一旦收到图像信息，就会进入到callback当中(依据需要识别的颜色进行划分)
  image_sub_ = image_transport_.subscribe("/probot_anno/camera/image_raw", 1, 
                                          std::bind(&GraspingDemo::imageCb, 
                                                    this, 
                                                    color, 
                                                    std::placeholders::_1)
                                          );
}

void GraspingDemo::imageCb(const int color,const sensor_msgs::ImageConstPtr &msg)
{
  if (!grasp_running)
  {
    ROS_INFO_STREAM("正在处理图像以定位物体");
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge 错误: %s", e.what());
      return;
    }

    float obj_x, obj_y;

    vision_manager_.GetLocation(cv_ptr->image, obj_x, obj_y, color);

    std::cout<< "相机坐标系中的物体坐标-X:" << obj_x << std::endl;
    std::cout<< "相机坐标系中的物体坐标-Y:" << obj_y << std::endl;

    obj_camera_frame.setZ(-obj_y);
    obj_camera_frame.setY(-obj_x);
    obj_camera_frame.setX(0.45);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    std::cout<< "机器人坐标系下的物体坐标-X:" << obj_robot_frame.getX() << std::endl;
    std::cout<< "机器人坐标系下的物体坐标-Y:" << obj_robot_frame.getY() << std::endl;
    std::cout<< "机器人坐标系下的物体坐标-Z:" << obj_robot_frame.getZ() << std::endl;
  }
}

void GraspingDemo::attainPosition(float x, float y, float z)
{
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link_2");
  visual_tools.deleteAllMarkers();

  // 获取当前位姿
  geometry_msgs::PoseStamped currPose = arm_group.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;

  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  arm_group.setPoseTarget(target_pose1);
  arm_group.move();
}

void GraspingDemo::AttainObject()
{
  // 到达物体上方0.05m的位置
  attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(), obj_robot_frame.getZ() + 0.05);

  // 张开夹爪
  ros::WallDuration(0.5).sleep();
  gripper_group.setNamedTarget("open");
  gripper_group.move();

  // 下落夹爪0.03m
  geometry_msgs::PoseStamped currPose = arm_group.getCurrentPose();
  geometry_msgs::Pose target_pose1;

  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  target_pose1.position.z = obj_robot_frame.getZ() - 0.03;
  arm_group.setPoseTarget(target_pose1);
  arm_group.move();
}

void GraspingDemo::grasp()
{
  // 闭合夹爪
  ros::WallDuration(0.5).sleep();
  gripper_group.setNamedTarget("close");
  gripper_group.move();
}

void GraspingDemo::move(int color)
{
  ros::WallDuration(0.5).sleep();

  geometry_msgs::PoseStamped currPose = arm_group.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  if(color == my_GREEN)
  {
    target_pose1.position.y = obj_robot_frame.getY() - 0.2;
  }
  else if (color == my_BLUE)
  {
    target_pose1.position.y = obj_robot_frame.getY() + 0.2;
  }
  else if (color == my_RED)
  {
    target_pose1.position.y = obj_robot_frame.getY() - 0.2;
  }

  arm_group.setPoseTarget(target_pose1);
  arm_group.move();

  // 张开夹爪
  ros::WallDuration(0.5).sleep();
  gripper_group.setNamedTarget("open");
  gripper_group.move();

  // 抬起夹爪0.08m
  target_pose1.position.z = target_pose1.position.z + 0.08;
  arm_group.setPoseTarget(target_pose1);
  arm_group.move();
}

void GraspingDemo::goHome()
{
  geometry_msgs::PoseStamped currPose = arm_group.getCurrentPose();

  // 闭合夹爪
  gripper_group.setNamedTarget("close");
  gripper_group.move();

  // 回到初始位置
  attainPosition(homePose.pose.position.x, homePose.pose.position.y, homePose.pose.position.z);
}

void GraspingDemo::ProcessGrasping(int color)
{
  //开启新的线程
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(0.5).sleep();

  //获取当前的位置
  homePose = arm_group.getCurrentPose();
  
  //机械臂靠近目标
  ROS_INFO_STREAM("机械臂靠近目标");
  AttainObject();

  //夹取物体
  ROS_INFO_STREAM("抓取物体");
  grasp();

  //夹住物体做一个小范围移动
  ROS_INFO_STREAM("移动物体");
  move(color);

  //机械臂返回到初始状态
  ROS_INFO_STREAM("返回初始状态");
  goHome();

  grasp_running = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_grasping");
  float table_length, table_width, initpos_x, initpos_y, initpos_z;
  ros::NodeHandle n;

  if (!n.getParam("probot_grasping/table_length", table_length))
    table_length = 0.3;
  if (!n.getParam("probot_grasping/table_width", table_width))
    table_width = 0.3;
  if (!n.getParam("probot_grasping/initpos_x", initpos_x))
    initpos_x = 0;
  if (!n.getParam("probot_grasping/initpos_y", initpos_y))
    initpos_y = 0;
  if (!n.getParam("probot_grasping/initpos_z", initpos_z))
    initpos_z = 0.28;

  GraspingDemo Grasp_Green(n, initpos_x, initpos_y, initpos_z, table_length, table_width,my_GREEN);
  GraspingDemo Grasp_Red(n, initpos_x, initpos_y, initpos_z, table_length, table_width,my_RED);
  GraspingDemo Grasp_Blue(n, initpos_x, initpos_y, initpos_z, table_length, table_width,my_BLUE);

  while (ros::ok())
  {
    ros::spinOnce();

    Grasp_Blue.ProcessGrasping(my_BLUE);

    Grasp_Green.ProcessGrasping(my_GREEN);

    Grasp_Red.ProcessGrasping(my_RED);

    ROS_INFO_STREAM("执行完毕");

    return 0;
  }
}
