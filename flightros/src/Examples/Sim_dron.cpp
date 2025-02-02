
// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"

#include <quadrotor_common/parameter_helper.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

SceneID scene_id{UnityScene::WAREHOUSE};
int scene_id_int;
std::shared_ptr<UnityBridge> unity_bridge_ptr;
bool unity_ready{false};
bool unity_render_{false};

std::shared_ptr<Quadrotor> quad_ptr;
QuadState quad_state;
QuadState quad_state_;

std::shared_ptr<RGBCamera> rgb_camera;

// publisher
image_transport::Publisher rgb_pub;
image_transport::Publisher depth_pub;
image_transport::Publisher segmentation_pub;
image_transport::Publisher opticalflow_pub;

FrameID frame_id = 0;



void ActiveCallback(const std_msgs::Bool::ConstPtr& msg);
bool setUnity(const bool render);
bool connectUnity();
void mainLoopCallback(const ros::TimerEvent &event);
void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
bool loadParams(void);

int main(int argc, char *argv[]) {
  // initialize ROS
  ros::init(argc, argv, "Sim_dron");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");  
  ros::Rate(50.0);

  // load parameters
  quadrotor_common::getParam("scene_id", scene_id_int, pnh);



  // subscriber
  ros::Subscriber active_sub = nh.subscribe("active_node/active", 1, ActiveCallback);
  ros::Subscriber odom_sub = nh.subscribe("odometry", 1, OdometryCallback);

  ros::Timer timer_main_loop_ = nh.createTimer(ros::Duration(0.02), mainLoopCallback);

  // unity quadrotor
  quad_ptr = std::make_shared<Quadrotor>();
  // define quadsize scale (for unity visualization only)
  Vector<3> quad_size(0.5, 0.5, 0.5);
  quad_ptr->setSize(quad_size);


  //
  rgb_camera = std::make_shared<RGBCamera>();
ROS_INFO("AQUII");
  // Flightmare(Unity3D)
  unity_bridge_ptr = UnityBridge::getInstance();
  //SceneID scene_id{UnityScene::WAREHOUSE};
  //unity_ready{false};

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  rgb_pub = it.advertise("/rgb", 1);
  depth_pub = it.advertise("/depth", 1);
  segmentation_pub = it.advertise("/segmentation", 1);
  opticalflow_pub = it.advertise("/opticalflow", 1);

  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera->setFOV(90);
  rgb_camera->setWidth(640);
  rgb_camera->setHeight(360);
  rgb_camera->setRelPose(B_r_BC, R_BC);
  rgb_camera->setPostProcesscing(
    std::vector<bool>{true, true, true});  // depth, segmentation, optical flow
  quad_ptr->addRGBCamera(rgb_camera);

ROS_INFO("ANTES INICIALIZAR");
  // initialization
  quad_state.setZero();
  quad_ptr->reset(quad_state);

  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();
ROS_INFO("ANTES CONECT");
  // connect unity
  unity_bridge_ptr->addQuadrotor(quad_ptr);
  unity_ready = unity_bridge_ptr->connectUnity(scene_id_int);

  ros::spin();

  return 0;
}


void ActiveCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (ros::ok() && unity_ready) {
          // connect unity
          //ROS_INFO("ACTIVE");
	  
	  


	  if (msg->data == true) {
	    

	    //quad_ptr->setState(quad_state_);

	    unity_bridge_ptr->getRender(0);
	    unity_bridge_ptr->handleOutput();

	    cv::Mat img;

	    ros::Time timestamp = ros::Time::now();

	    rgb_camera->getRGBImage(img);
	    sensor_msgs::ImagePtr rgb_msg =
	      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	    rgb_msg->header.stamp = timestamp;
	    rgb_pub.publish(rgb_msg);

	    rgb_camera->getDepthMap(img);
	    sensor_msgs::ImagePtr depth_msg =
	      cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
	    depth_msg->header.stamp = timestamp;
	    depth_pub.publish(depth_msg);

	    rgb_camera->getSegmentation(img);
	    sensor_msgs::ImagePtr segmentation_msg =
	      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	    segmentation_msg->header.stamp = timestamp;
	    segmentation_pub.publish(segmentation_msg);

	    rgb_camera->getOpticalFlow(img);
	    sensor_msgs::ImagePtr opticflow_msg =
	      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	    opticflow_msg->header.stamp = timestamp;
	    opticalflow_pub.publish(opticflow_msg);

	    frame_id += 1;
	  }
  }
}

void mainLoopCallback(const ros::TimerEvent &event) {

}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr->setState(quad_state_);
}

bool setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr == nullptr) {
    // create unity bridge
    unity_bridge_ptr = UnityBridge::getInstance();
    unity_bridge_ptr->addQuadrotor(quad_ptr);
    //ROS_INFO("[%s] Unity Bridge is created.", pnh.getNamespace().c_str());
  }
  return true;
}

bool connectUnity() {
  if (!unity_render_ || unity_bridge_ptr == nullptr) return false;
  unity_ready = unity_bridge_ptr->connectUnity(scene_id_int);
  return unity_ready;
}



