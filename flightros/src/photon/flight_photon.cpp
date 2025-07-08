#include "flightros/photon/flight_photon.hpp"

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(30.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

// add front left camera 
front_left_camera = std::make_shared<RGBCamera>();
front_left_camera->setWidth(640);
front_left_camera->setHeight(400);
front_left_camera->setFOV(128); // HFOV
Vector<3> front_camera_left_position(-0.0373, 0.45, 0.0); // Left lens at x = -74.6/2 = -37.3 mm
Matrix<3, 3> front_camera_left_rotation = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
front_left_camera->setRelPose(front_camera_left_position, front_camera_left_rotation);
quad_ptr_->addRGBCamera(front_left_camera);

// add front right camera
front_right_camera = std::make_shared<RGBCamera>();
front_right_camera->setWidth(640);
front_right_camera->setHeight(400);
front_right_camera->setFOV(128); // HFOV
Vector<3> front_camera_right_position(0.0373, 0.45, 0.0); // Right lens at x = 74.6/2 = 37.3 mm
Matrix<3, 3> front_camera_right_rotation = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
front_right_camera->setRelPose(front_camera_right_position, front_camera_right_rotation);
quad_ptr_->addRGBCamera(front_right_camera);

// add bottom left camera
bottom_left_camera = std::make_shared<RGBCamera>();
bottom_left_camera->setWidth(640);
bottom_left_camera->setHeight(400);
bottom_left_camera->setFOV(128); // HFOV
Vector<3> bottom_left_position(0.03445 ,0.45, -0.5); // Half of 68.9 mm = 34.45 mm
Matrix<3, 3> bottom_left_rotation = Quaternionf(AngleAxisf(-M_PI/2, Vector3f::UnitX())).toRotationMatrix();
bottom_left_camera->setRelPose(bottom_left_position, bottom_left_rotation);
quad_ptr_->addRGBCamera(bottom_left_camera);

// add bottom right camera
bottom_right_camera = std::make_shared<RGBCamera>();
bottom_right_camera->setWidth(640);
bottom_right_camera->setHeight(400);
bottom_right_camera->setFOV(128); // HFOV
Vector<3> bottom_right_position(0.03445 , 0.45, -0.5 );
Matrix<3, 3> bottom_right_rotation = Quaternionf(AngleAxisf(-M_PI/2, Vector3f::UnitX())).toRotationMatrix();
bottom_right_camera->setRelPose(bottom_right_position, bottom_right_rotation);
quad_ptr_->addRGBCamera(bottom_right_camera);

// add left camera (IMX586 tilted 5° downward in horizontal plane)
left_camera = std::make_shared<RGBCamera>();
left_camera->setWidth(640);
left_camera->setHeight(480);
left_camera->setFOV(79);
// Tilt downward 5° in horizontal plane = rotate around Z axis
float deg_to_rad = M_PI / 180.0f;
Quaternionf qz_left(AngleAxisf(+90.0f * deg_to_rad, Vector3f::UnitZ()));
Quaternionf qy_left(AngleAxisf(-5.0f * deg_to_rad, Vector3f::UnitX()));
Eigen::Matrix3f left_camera_rotation = (qz_left * qy_left).toRotationMatrix();
Vector<3> left_camera_position(-0.4, 0.0, 0.0); // ปรับตำแหน่งตามการติดตั้งจริง
left_camera->setRelPose(left_camera_position, left_camera_rotation);
quad_ptr_->addRGBCamera(left_camera);

// add right camera (IMX586 tilted 5° downward in horizontal plane)
right_camera = std::make_shared<RGBCamera>();
right_camera->setWidth(640);
right_camera->setHeight(480);
right_camera->setFOV(79);
Quaternionf qz_right(AngleAxisf(-90.0f * deg_to_rad, Vector3f::UnitZ()));
Quaternionf qy_right(AngleAxisf(-5.0f * deg_to_rad, Vector3f::UnitX()));
Eigen::Matrix3f right_camera_rotation = (qz_right * qy_right).toRotationMatrix();
Vector<3> right_camera_position(0.4, 0.0, 0.0); // Adjust position according to actual installation
right_camera->setRelPose(right_camera_position, right_camera_rotation);
quad_ptr_->addRGBCamera(right_camera);


  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);

  it = std::make_shared<image_transport::ImageTransport>(nh_);

  front_left_camera_pub = it->advertise("front_left/image_raw", 1);
  front_right_camera_pub = it->advertise("front_right/image_raw", 1);
  right_camera_pub = it->advertise("right/image_raw", 1);
  left_camera_pub = it->advertise("left/image_raw", 1);
  bottom_left_camera_pub = it->advertise("bottom_left/image_raw", 1);
  bottom_right_camera_pub = it->advertise("bottom_right/image_raw", 1);

  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    if (quad_ptr_->getCollision()) {
      // collision happened
      ROS_INFO("COLLISION");
    }

    // publish front left camera image
    cv::Mat front_left_image;
    front_left_camera->getRGBImage(front_left_image);
    if (!front_left_image.empty()) {
      sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", front_left_image).toImageMsg();
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "front_left_camera_link";
      front_left_camera_pub.publish(msg);
    }

    cv::Mat front_right_image;
    front_right_camera->getRGBImage(front_right_image);
    if (!front_right_image.empty()) {
      sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", front_right_image).toImageMsg();
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "front_right_camera_link";
      front_right_camera_pub.publish(msg);
    }

    cv::Mat right_image;
    right_camera->getRGBImage(right_image);
    if (!right_image.empty()) {
      sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_image).toImageMsg();
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "right_camera_link";
      right_camera_pub.publish(msg);
    }

    cv::Mat left_image;
    left_camera->getRGBImage(left_image);
    if (!left_image.empty()) {
      sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_image).toImageMsg();
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "left_camera_link";
      left_camera_pub.publish(msg);
    } 

    cv::Mat bottom_left_image;
    bottom_left_camera->getRGBImage(bottom_left_image); 
    if (!bottom_left_image.empty()) {
      sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", bottom_left_image).toImageMsg();
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "bottom_left_camera_link";
      bottom_left_camera_pub.publish(msg);
    } 
    cv::Mat bottom_right_image;
    bottom_right_camera->getRGBImage(bottom_right_image);
    if (!bottom_right_image.empty()) {
      sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", bottom_right_image).toImageMsg();
      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "bottom_right_camera_link";
      bottom_right_camera_pub.publish(msg);
    }
    
  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // empty
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros