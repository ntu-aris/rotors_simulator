/*
 * Copyright 2023 Muqing Cao <caomuqing@163.com>
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

#ifndef ROTORS_GAZEBO_PLUGINS_CARIC_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_CARIC_PLUGIN_H

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

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <rotors_comm/BoolStamped.h>
#include <deque>
#include <math.h>       
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

#include <Eigen/Geometry>
#include <stdio.h>
#include <random>
#include <tf/transform_datatypes.h>

#include <CGAL/Tetrahedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
// typedef CGAL::Convex_hull_traits_3<K> Traits;
typedef K::Tetrahedron_3 CGAL_Tetra_3;
typedef K::Point_3 Point_3;

using namespace std;

namespace gazebo {

class GazeboCaricPlugin : public ModelPlugin {
 public:

  GazeboCaricPlugin();
  ~GazeboCaricPlugin();

  void InitializeParams();
  void Publish();
  bool CheckLOS(const Vector3d &pi, const Vector3d &pj, gazebo::physics::RayShapePtr &ray);
  bool CheckLOS(const tf::Vector3 &pi, const tf::Vector3 &pj);
  pcl::search::KdTree<pcl::PointXYZINormal> getktree()
  {
    return kdTreeInterestPts_;
  }
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr getcloud()
  {
    return cloud_;
  }
  double gethfov() {return H_fov_; }
  double getvfov() {return V_fov_; }

 protected:
 
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief  	This gets called by the world update start event.
  /// \details	Calculates IMU parameters and then publishes one IMU message.
  void OnUpdate(const common::UpdateInfo&);

 private:

    class Listener
    {
    private:
        GazeboCaricPlugin& plugin_;
        int agent_id;
        std::deque<nav_msgs::Odometry> odom_deque;
        std::deque<rotors_comm::BoolStamped> trigger_deque;
        int odom_save_size_ = 600;
        ros::Time last_vis_view_pyra_time_;
        ros::Publisher visPub;
        double visible_radius = 5.0;
        tf::Vector3 Cam_rel_Uav;
        bool visualize =true;
    public:
    Listener(int id, GazeboCaricPlugin& plugin):plugin_(plugin)
    {
        // plugin_ = plugin;
        agent_id = id;
        for (size_t i = 0; i < odom_save_size_; i++)
        {
            nav_msgs::Odometry msg_tmp;
            odom_deque.push_back(msg_tmp);
        }
        
    }
    void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
    void triggerCB(const rotors_comm::BoolStamped::ConstPtr& msg);
    bool findClosestOdom(rotors_comm::BoolStamped trigger, nav_msgs::Odometry& odom);
    void checkEvaluateCaptures();
    void getPointInWorld(tf::Vector3 &point_in_cam, const nav_msgs::Odometry& uav_odom, 
                        tf::Vector3& Cam_rel_Uav, tf::Vector3& Cam_angle, tf::Vector3& point_in_world);
    void getPointInWorld2(tf::Vector3 &point_in_cam, tf::Vector3& cam_in_world, tf::Vector3& Cam_angle, tf::Vector3& point_in_world);                        
    bool point_in_viewPyramid(tf::Vector3 cam_world, std::vector<tf::Vector3> base_points, tf::Vector3 interestPt);
    void setId(int id)
    {
        agent_id = id;
    }
    int getId()
    {
        return agent_id;
    }
    void setVisTime(ros::Time time)
    {
        last_vis_view_pyra_time_ = time;
    }
    ros::Time getVisTime()
    {
        return last_vis_view_pyra_time_;
    }

    void setPublisher(ros::Publisher pub)
    {
        visPub = pub;
    }
    void setVisibleRad(double rad)
    {
        visible_radius = rad;
    }    
    void setCam_rel_Uav(tf::Vector3 input)
    {
        Cam_rel_Uav = input;
    }    
    };

  void OdomCallback(const nav_msgs::OdometryConstPtr &msg);
  void readPCloud(std::string filename);
  geometry_msgs::Point geometricMsgFromTf(tf::Vector3 input);
  void pubTimerCallback(const ros::TimerEvent& e);

  string namespace_;

  /// \brief  Handle for the Gazebo node.
  transport::NodePtr gz_node_handle_;

  /// \brief  Handle for the ROS node.
  ros::NodeHandle *ros_node_handle1_, *ros_node_handle2_, *ros_node_handle3_;

  /// \brief  Topic to publish the comm result
  string self_link_name_;

  /// \brief  Pointer to the world.
  physics::WorldPtr world_;

  /// \brief    Pointer to the model.
  physics::ModelPtr model_;

  /// \brief  Pointer to the engine.
  physics::PhysicsEnginePtr physics_;

  /// \brief  Update time for the topology
  double ppcom_hz_;  

  /// \brief  Path to the configuration of the network
  string ppcom_config_;


  // /// \brief  Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  // default_random_engine random_generator_;
  // normal_distribution<double> standard_normal_distribution_;

  common::Time last_time_;
  gazebo::physics::RayShapePtr ray_;
  pcl::search::KdTree<pcl::PointXYZINormal> kdTreeInterestPts_;
  std::vector<Listener> idListener_;  
  int number_of_robots_ = 3;
  std::string fstring_ = "/home/to/Desktop/file.pcd";
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_;
  double H_fov_ = 80; //in degrees
  double V_fov_ = 60;

  std::vector<ros::Publisher> viewPub_list_;
  ros::Publisher int_points_cloud_;
  std::vector<ros::Subscriber> subscriber_list;
  std::vector<ros::Timer> timer_list;
  ros::Timer pubTimer;  
};


}  // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_CARIC_PLUGIN_H
