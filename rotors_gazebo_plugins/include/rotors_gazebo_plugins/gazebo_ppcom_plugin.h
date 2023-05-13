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
#include "rotors_gazebo_plugins/common.h"
// #include "rotors_gazebo_plugins/gazebo_ros_interface_plugin.h"

#include "utility.h"

using namespace std;

namespace gazebo {

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

  /// \brief  Idx of the ppcom node
  int ppcom_slf_idx_;

  /// \brief  Path to the configuration of the network
  string ppcom_config_;


  // Object to store the node info
  struct PPComNode
  {
    //   PPComNode();
    //   PPComNode(const string &name_, const double &offset_) : name(name_), offset(offset_) {};
    //  ~PPComNode();
     
      // Name of the node
      string name = "";

      string role = "";

      // Offset used to do ray tracing
      double offset = 0.0;

      // ray tracing object
      gazebo::physics::RayShapePtr ray;

      // Subsribed odometry subscriber
      ros::Subscriber odom_sub;

      // Saved odometry message
      nav_msgs::Odometry odom_msg;

      bool odom_msg_received = false;
  };

  /// \brief  Number of node in the network
  int Nnodes_;

  /// \brief  Name of the nodes read from the ppcom_config_
  vector<PPComNode> ppcom_nodes_;

  // /// \brief  Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  // default_random_engine random_generator_;
  // normal_distribution<double> standard_normal_distribution_;

  common::Time last_time_;
};

}  // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_PPCOM_PLUGIN_H
