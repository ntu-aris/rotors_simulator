/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#ifndef ROTORS_GAZEBO_PLUGINS_PPCOM_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_PPCOM_PLUGIN_H

#include <random>

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// #include <ignition/transport.hh>

#include <ros/ros.h>
// #include <ros/callback_queue.h>
// #include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// #include "Imu.pb.h"

#include "rotors_gazebo_plugins/common.h"
// #include "rotors_gazebo_plugins/gazebo_ros_interface_plugin.h"

#include "utility.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include "utility_tm.h"
#include "geometry_msgs/Twist.h"

using namespace std;

namespace gazebo {

// Object to store the node info
struct PPComNode
{
    PPComNode();
    PPComNode(const string &name_, const string &role_, const double &offset_);
    PPComNode(const string &name_, const string &role_, const double &offset_,
              const double &hfov, const double &vfov, const double &cam_x,
              const double &cam_y, const double &cam_z);   
   ~PPComNode();
    
    // Name of the node
    string name = "";

    string role = "";

    // Offset used to do ray tracing
    double offset = 0.0;

    // ray tracing object
    gazebo::physics::RayShapePtr ray_topo;

    // ray tracing object
    gazebo::physics::RayShapePtr ray_camera;

    // Subsribed odometry subscriber
    ros::Subscriber odom_sub;
    ros::Subscriber gimbal_sub;
    // Subsribed odometry subscriber
    ros::Publisher topo_pub;
    ros::Publisher camera_pyramid_pub;
    ros::Timer timer_update;

    // Saved odometry message
    nav_msgs::Odometry odom_msg;

    bool odom_msg_received = false;

    Eigen::Vector3d Cam_rel_Uav;
    Eigen::Vector3d cam_rpy;
    Eigen::Vector3d cam_rpy_rate;

    double visible_radius = 10.0;
    double fov_h = 90.0;
    double fov_v = 60.0;
    double focal_length = 0.016; //in meter
    double exposure = 0.001; //in s
    double pixel_size = 4.35e-6; //in m
    Eigen::VectorXd gimbal_cmd;
    ros::Time gimbal_cmd_last_update;
};

class GazeboPPComPlugin : public ModelPlugin {
 public:

  GazeboPPComPlugin();
  ~GazeboPPComPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief  	This gets called by the world update start event.
  /// \details	Calculates IMU parameters and then publishes one IMU message.
  void OnUpdate(const common::UpdateInfo&);

 private:

  void OdomCallback(const nav_msgs::OdometryConstPtr &msg, int node_idx);
  void UpdateTopo();
  void UpdateInterestPoint();
  bool CheckTopoLOS(const Vector3d &pi, double bi, const Vector3d &pj, double bj, gazebo::physics::RayShapePtr &ray);
  bool CheckInterestPointLOS(const Eigen::Vector3d &pi, const Eigen::Vector3d &pj, gazebo::physics::RayShapePtr &ray);  
  void readPCloud(std::string filename);
  void TimerCallback(const ros::TimerEvent &, int node_idx);
  void GimbalCallback(const geometry_msgs::TwistConstPtr &msg, int node_idx);
  
  string namespace_;
  string ppcom_topic_;

  /// \brief  Handle for the Gazebo node.
  transport::NodePtr gz_node_handle_;

  /// \brief  Handle for the ROS node.
  ros::NodeHandle* ros_node_handle_;

  /// \brief  Topic to publish the comm result
  transport::PublisherPtr ppcom_pub_;

  /// \brief  Topic to publish the comm result
  string self_link_name_;

  /// \brief  Pointer to the world.
  physics::WorldPtr world_;

  /// \brief    Pointer to the model.
  physics::ModelPtr model_;

  /// \brief  Pointer to the engine.
  physics::PhysicsEnginePtr physics_;
  
  /// \brief  Id of the ppcom node
  string ppcom_id_;

  /// \brief  Update time for the topology
  double ppcom_hz_;  

  /// \brief  Idx of the ppcom node
  int ppcom_slf_idx_;

  /// \brief  Path to the configuration of the network
  string ppcom_config_;

  /// \brief  Number of node in the network
  int Nnodes_;

  /// \brief  Name of the nodes read from the ppcom_config_
  vector<PPComNode> ppcom_nodes_;

  // /// \brief  Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  // default_random_engine random_generator_;
  // normal_distribution<double> standard_normal_distribution_;

  /// \brief  Update time for the topology
  double cam_evaluate_hz_ = 10;

  common::Time last_time_topo_, last_time_cam_;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_;
  pcl::search::KdTree<pcl::PointXYZINormal> kdTreeInterestPts_;
  ros::Publisher cloud_pub_;
  double gimbal_update_hz_ = 10.0;
  double gimbal_rate_max_ = 45.0/180.0*M_PI;
  double gimbal_pitch_max_ = 70.0/180.0*M_PI;
  double gimbal_yaw_max_ = 80.0/180.0*M_PI;
};

}  // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_PPCOM_PLUGIN_H
