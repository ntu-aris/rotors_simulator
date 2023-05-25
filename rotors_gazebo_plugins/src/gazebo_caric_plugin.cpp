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

// MODULE HEADER
#include "rotors_gazebo_plugins/gazebo_caric_plugin.h"

// SYSTEM LIBS
#include <stdio.h>
#include <boost/bind.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>

#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"

// Physics
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>

// 3RD PARTY
#include "mav_msgs/default_topics.h"

// // USER HEADERS
// #include "ConnectGazeboToRosTopic.pb.h"

using namespace std;

namespace gazebo
{

    // Plugin definition
    GazeboCaricPlugin::GazeboCaricPlugin()
        : ModelPlugin(), gz_node_handle_(0) {}

    GazeboCaricPlugin::~GazeboCaricPlugin() {}

    void GazeboCaricPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (kPrintOnPluginLoad)
            gzdbg << __FUNCTION__ << "() called." << endl;

        gzdbg << "_model = " << _model->GetName() << endl;

        // Store the pointer to the model, world, and physics
        model_ = _model;
        world_ = model_->GetWorld();
        physics_ = world_->Physics();

        physics_->InitForThread();

        // default params
        namespace_.clear();

        //==============================================//
        //========== READ IN PARAMS FROM SDF ===========//
        //==============================================//

        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<string>();
        else
            gzerr << "[gazebo_caric_plugin] Please specify a robotNamespace.\n";

        if (_sdf->HasElement("linkName"))
            self_link_name_ = _sdf->GetElement("linkName")->Get<string>();
        else
            gzerr << "[gazebo_caric_plugin] Please specify a linkName.\n";

        if (_sdf->HasElement("ppcomConfig"))
            ppcom_config_ = _sdf->GetElement("ppcomConfig")->Get<string>();
        else
            gzerr << "[gazebo_caric_plugin] Please specify ppcomConfig.\n";

        if (_sdf->HasElement("ppcomHz"))
            ppcom_hz_ = _sdf->GetElement("ppcomHz")->Get<double>();
        else
            gzerr << "[gazebo_caric_plugin] Please specify a ppcomId.\n";

        if (_sdf->HasElement("interestPcd"))
            fstring_ = _sdf->GetElement("interestPcd")->Get<string>();
        else
            gzerr << "[gazebo_caric_plugin] Please specify input interest points.\n";

        double cam_x, cam_y, cam_z;
        if (_sdf->HasElement("cam_x") && _sdf->HasElement("cam_y") && _sdf->HasElement("cam_z"))
        {
            cam_x = _sdf->GetElement("cam_x")->Get<double>();
            cam_y = _sdf->GetElement("cam_y")->Get<double>();
            cam_z = _sdf->GetElement("cam_z")->Get<double>();
        }
        else
            gzerr << "[gazebo_caric_plugin] Please specify cam position relative to drone.\n";

        readPCloud(fstring_);
        // Cam_rel_Uav_ = tf::Vector3(cam_x, cam_y, cam_z);

        //==============================================//
        //========== CREATE THE TRANSPORT STRUCTURES ===//
        //==============================================//

        // Create a gazebo node handle and initialize with the namespace
        gz_node_handle_ = transport::NodePtr(new transport::Node());
        gz_node_handle_->Init();

        // Create a ros node
        ros_node_handle1_ = new ros::NodeHandle("/carid_node1");
        ros_node_handle2_ = new ros::NodeHandle("/carid_node2");
        ros_node_handle3_ = new ros::NodeHandle("/carid_node3");

        // Open the config file and read the links
        std::ifstream ppcom_config_file(ppcom_config_.c_str());

        subscriber_list.clear();
        timer_list.clear();
        viewPub_list_.clear();
        string line;
        int i = 0;
        while (getline(ppcom_config_file, line))
        {
            // Process the line here
            cout << "Reading " << line << endl;

            // Remove spaces in the line
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());

            // Skip the line of its commented out
            if (line[0] != 'f')
                continue;

            std::cout << "caric_plugin: adding drones!!" << std::endl;
            // Decode the line and construct the node
            Listener idListener(i, *this);
            idListener.setVisibleRad(5.0);
            idListener.setCam_rel_Uav(tf::Vector3(cam_x, cam_y, cam_z));
            idListener_.push_back(idListener);
            i++;
        }
        for (int i = 0; i < idListener_.size(); i++)
        {
            ros::Subscriber odometry_sub = ros_node_handle1_->subscribe<nav_msgs::Odometry>("/firefly" + std::to_string(i + 1) + "/ground_truth/odometry",
                                                                                            1, &Listener::odomCB, &idListener_[i]);
            ros::Subscriber trigger_cmd_sub = ros_node_handle1_->subscribe<rotors_comm::BoolStamped>("/firefly" + std::to_string(i + 1) + "/trigger",
                                                                                                     1, &Listener::triggerCB, &idListener_[i]);
            // ros::Timer evaluateTimer = ros_node_handle3_->createTimer(ros::Duration(0.05), &Listener::checkEvaluateCaptures, &idListener_[i]);
            idListener_[i].setVisTime(ros::Time::now());
            ros::Publisher view_pub = ros_node_handle2_->advertise<visualization_msgs::Marker>("/firefly" + std::to_string(i + 1) + "/visualize", 1);
            // viewPub_list_.push_back(view_pub);
            idListener_[i].setPublisher(view_pub);
            subscriber_list.push_back(odometry_sub);
            subscriber_list.push_back(trigger_cmd_sub);
            // timer_list.push_back(evaluateTimer);
            ROS_INFO("finish setup for robot %d!!", i);
        }
        // ros::Subscriber odometry_sub = ros_node_handle1_->subscribe<nav_msgs::Odometry>("/firefly"+std::to_string(0+1)+"/ground_truth/odometry",
        //     1, &GazeboCaricPlugin::OdomCallback, this);
        int_points_cloud_ = ros_node_handle2_->advertise<sensor_msgs::PointCloud2>("/interest_cloud", 100);
        pubTimer = ros_node_handle2_->createTimer(ros::Duration(0.1), &GazeboCaricPlugin::pubTimerCallback, this);

        // Create rayshape object
        ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

        // Listen to the update event. This event is broadcast every simulation iteration.
        last_time_ = world_->SimTime();
        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCaricPlugin::OnUpdate, this, _1));
    }

    void GazeboCaricPlugin::readPCloud(std::string filename)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        cloud_ = cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(filename, *cloud_) == -1) // load point cloud file
        {
            PCL_ERROR("Could not read the file");
            return;
        }
        std::cout << "Loaded" << cloud_->width * cloud_->height
                  << "data points with the following fields: "
                  << std::endl;

        // for(size_t i = 0; i < cloud_->points.size(); ++i)
        //     std::cout << "    " << cloud_->points[i].x
        //               << " "    << cloud_->points[i].y
        //               << " "    << cloud_->points[i].z
        //               << " "    << cloud_->points[i].normal_x
        //               << " "    << cloud_->points[i].normal_y
        //               << " "    << cloud_->points[i].normal_z << std::endl;
        kdTreeInterestPts_.setInputCloud(cloud_);
    }

    void GazeboCaricPlugin::OdomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        ROS_INFO("visualizing GazeboCaricPlugin!!");
    }

    void GazeboCaricPlugin::Listener::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // ROS_INFO("start odomCB for robot %d!!", getId());
        odom_deque.push_back(*msg);
        //   ROS_INFO("start odomCB 1 for robot %d!!", getId());
        if (odom_deque.size() > odom_save_size_)
            odom_deque.pop_front();
        //   ROS_INFO("start odomCB 1.5!!");
        if (visualize && (msg->header.stamp - last_vis_view_pyra_time_).toSec() > 0.1) // visualize at 10hz
        {
            tf::Vector3 Cam_angle_tmp(0, 0, 0); // use all zero for now

            tf::Quaternion q_uav_world(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                       msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            tf::Matrix3x3 R_uav_world(q_uav_world);
            tf::Vector3 p_uav_world(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
            tf::Vector3 p_cam_world = R_uav_world.inverse() * Cam_rel_Uav + p_uav_world;

            tf::Vector3 p0(p_cam_world.getX(), p_cam_world.getY(), p_cam_world.getZ());
            std::vector<tf::Vector3> point_list_in_world;
            point_list_in_world.push_back(p0);
            for (double k : {1.0, -1.0})
            {
                for (double l : {1.0, -1.0})
                {
                    tf::Vector3 point_in_cam(visible_radius,
                                             k * tan(plugin_.gethfov() * 0.008726646) * visible_radius,
                                             l * tan(plugin_.getvfov() * 0.008726646) * visible_radius);
                    tf::Vector3 point_in_world;
                    getPointInWorld2(point_in_cam, p_cam_world, Cam_angle_tmp, point_in_world);
                    point_list_in_world.push_back(point_in_world);
                }
                point_list_in_world.push_back(p0);
            }
            point_list_in_world.push_back(point_list_in_world[1]);
            point_list_in_world.push_back(point_list_in_world[4]);
            point_list_in_world.push_back(point_list_in_world[5]);
            point_list_in_world.push_back(point_list_in_world[2]);
            visualization_msgs::Marker m;
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::DELETEALL;
            m.id = agent_id; // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
            m.ns = "view_agent_" + std::to_string(agent_id);

            m.color.a = 1.0; // Don't forget to set the alpha!
            m.color.r = 0.0;
            m.color.g = 1.0;
            m.color.b = 0.0;

            m.scale.x = 0.15;
            m.scale.y = 0.0001;
            m.scale.z = 0.0001;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = msg->header.frame_id;
            // viewPub_list_[agent_id].publish(m); //DELETE last line first
            visPub.publish(m);
            m.action = visualization_msgs::Marker::ADD;
            // pose is actually not used in the marker, but if not RVIZ complains about the quaternion
            m.pose.position.x = 0.0;
            m.pose.position.y = 0.0;
            m.pose.position.z = 0.0;
            m.pose.orientation.x = 0.0;
            m.pose.orientation.y = 0.0;
            m.pose.orientation.z = 0.0;
            m.pose.orientation.w = 1.0;

            for (auto point : point_list_in_world)
            {
                m.points.push_back(plugin_.geometricMsgFromTf(point));
            }
            // viewPub_list_[agent_id].publish(m);
            visPub.publish(m);
            last_vis_view_pyra_time_ = msg->header.stamp;
        }
    }

    void GazeboCaricPlugin::Listener::triggerCB(const rotors_comm::BoolStamped::ConstPtr &msg)
    {
        trigger_deque.push_back(*msg);
        if (trigger_deque.size() > 10)
        {
            ROS_WARN("Many camera capture commands in the buffer!!");
        }
    }

    void GazeboCaricPlugin::pubTimerCallback(const ros::TimerEvent &e)
    {
        // ROS_INFO("start pubTimerCallback!!");
        sensor_msgs::PointCloud2 msg1;
        pcl::toROSMsg(*cloud_, msg1);
        msg1.header.frame_id = "world";
        int_points_cloud_.publish(msg1);
        //    ROS_INFO("end pubTimerCallback!!");
    }

    void GazeboCaricPlugin::Listener::checkEvaluateCaptures()
    {
        tf::Vector3 Cam_angle_tmp(0, 0, 0); // use all zero for now
        for (int i = 0; i < trigger_deque.size(); i++)
        {
            nav_msgs::Odometry odom;
            if (!findClosestOdom(trigger_deque[i], odom))
                continue;
            // getPointInWorld(point_body, )

            tf::Quaternion q_uav_world(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                       odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
            tf::Matrix3x3 R_uav_world(q_uav_world);
            tf::Vector3 p_uav_world(odom.pose.pose.position.x, odom.pose.pose.position.y,
                                    odom.pose.pose.position.z);
            tf::Vector3 p_cam_world = R_uav_world.inverse() * Cam_rel_Uav + p_uav_world;

            pcl::PointXYZINormal position;
            position.x = p_cam_world.getX();
            position.y = p_cam_world.getY();
            position.z = p_cam_world.getZ();

            std::vector<int> k_idx;
            std::vector<float> k_distances;
            plugin_.getktree().radiusSearch(position, visible_radius, k_idx, k_distances);
            for (int j = 0; j < k_idx.size(); j++) //
            {
                pcl::PointXYZINormal &kpoint = plugin_.getcloud()->points[k_idx[j]];
                tf::Vector3 InPoint_world(kpoint.x, kpoint.y, kpoint.z);
                std::vector<tf::Vector3> point_list_in_world;
                for (double k : {1.0, -1.0})
                {
                    for (double l : {1.0, -1.0})
                    {
                        tf::Vector3 point_in_cam(visible_radius,
                                                 k * tan(plugin_.gethfov() * 0.008726646) * visible_radius,
                                                 l * tan(plugin_.getvfov() * 0.008726646) * visible_radius);
                        tf::Vector3 point_in_world;
                        getPointInWorld2(point_in_cam, p_cam_world, Cam_angle_tmp, point_in_world);
                        point_list_in_world.push_back(point_in_world);
                    }
                }
                if (point_in_viewPyramid(p_cam_world, point_list_in_world, InPoint_world))
                {
                    // ROS_INFO("checking LOS!!");
                    if (plugin_.CheckLOS(p_cam_world, InPoint_world))
                    {
                        plugin_.getcloud()->points[k_idx[j]].intensity = 1.0;
                        // ROS_INFO("Setting points to high intensity!!");
                    }
                }
            }
        }
    }

    void GazeboCaricPlugin::Listener::getPointInWorld2(tf::Vector3 &point_in_cam, tf::Vector3 &cam_in_world, tf::Vector3 &Cam_angle, tf::Vector3 &point_in_world)
    {
        tf::Matrix3x3 point_to_world;
        point_to_world.setRPY(Cam_angle.getX(), Cam_angle.getY(), Cam_angle.getZ());
        tf::Vector3 point_rel_to_gimbal_in_world = point_to_world.inverse() * point_in_cam;

        point_in_world = cam_in_world + point_rel_to_gimbal_in_world;
    }

    void GazeboCaricPlugin::Listener::getPointInWorld(tf::Vector3 &point_in_cam, const nav_msgs::Odometry &uav_odom,
                                                      tf::Vector3 &Cam_rel_Uav, tf::Vector3 &Cam_angle, tf::Vector3 &point_in_world)
    {
        tf::Matrix3x3 point_to_world;
        point_to_world.setRPY(Cam_angle.getX(), Cam_angle.getY(), Cam_angle.getZ());
        tf::Vector3 point_rel_to_gimbal_in_world = point_to_world.inverse() * point_in_cam;
        tf::Quaternion q_uav_world(uav_odom.pose.pose.orientation.x, uav_odom.pose.pose.orientation.y,
                                   uav_odom.pose.pose.orientation.z, uav_odom.pose.pose.orientation.w);
        tf::Matrix3x3 R_uav_world(q_uav_world);
        tf::Vector3 p_uav_world(uav_odom.pose.pose.position.x, uav_odom.pose.pose.position.y,
                                uav_odom.pose.pose.position.z);
        point_in_world = R_uav_world.inverse() * Cam_rel_Uav + point_rel_to_gimbal_in_world + p_uav_world;
    }

    bool GazeboCaricPlugin::Listener::point_in_viewPyramid(tf::Vector3 cam_world, std::vector<tf::Vector3> base_points, tf::Vector3 interestPt)
    {
        Point_3 interestP(interestPt.getX(), interestPt.getY(), interestPt.getZ());
        Point_3 p0(cam_world.getX(), cam_world.getY(), cam_world.getZ());
        std::vector<Point_3> list_points;
        for (int i = 0; i < base_points.size(); i++)
        {
            Point_3 p1(base_points[i].getX(), base_points[i].getY(), base_points[i].getZ());
            list_points.push_back(p1);
        }

        CGAL_Tetra_3 tetra(p0, list_points[0], list_points[1], list_points[2]);
        if (CGAL::do_intersect(tetra, interestP))
        {
            return true;
        }

        CGAL_Tetra_3 tetra1(p0, list_points[1], list_points[2], list_points[3]);
        if (CGAL::do_intersect(tetra1, interestP))
        {
            return true;
        }
    }

    bool GazeboCaricPlugin::Listener::findClosestOdom(rotors_comm::BoolStamped trigger, nav_msgs::Odometry &odom)
    {
        if (odom_deque.back().header.stamp - trigger.header.stamp < ros::Duration(0) ||
            odom_deque.front().header.stamp - trigger.header.stamp > ros::Duration(0))
        {
            // ROS_WARN("trigger time out of the range of the buffer! leave if first!!");
            return false;
        }
        else
        {
            // ROS_WARN("processing points!!!");
        }

        int oldest = 0;
        int newest = odom_deque.size() - 1;
        if (newest <= 0)
        {
            ROS_WARN("findClosestOdom: not many odom received!!");
            return false;
        }
        int mid;
        while (newest - oldest > 1)
        {
            mid = oldest + (newest - oldest) / 2;
            if (odom_deque[mid].header.stamp - trigger.header.stamp < ros::Duration(0))
                oldest = mid;

            else
                newest = mid;
        }

        if (trigger.header.stamp - odom_deque[oldest].header.stamp >
            odom_deque[newest].header.stamp - trigger.header.stamp)
        {
            odom = odom_deque[newest];
        }
        else
        {
            odom = odom_deque[oldest];
        }

        if (fabs((odom.header.stamp - trigger.header.stamp).toSec()) > 0.01)
            ROS_WARN("findClosestOdom: found odom is too far from the trigger time!!");

        return true;
    }

    geometry_msgs::Point GazeboCaricPlugin::geometricMsgFromTf(tf::Vector3 input)
    {
        geometry_msgs::Point point;
        point.x = input.getX();
        point.y = input.getY();
        point.z = input.getZ();
        return point;
    }

    void GazeboCaricPlugin::OnUpdate(const common::UpdateInfo &_info)
    {
        if (kPrintOnUpdates)
            gzdbg << __FUNCTION__ << "() called." << endl;

        common::Time current_time = world_->SimTime();
        double dt = (current_time - last_time_).Double();

        // Update the ray casting every 0.1s
        if (dt > 1.0 / 1.0)
        {
            last_time_ = current_time;
            for (auto listener : idListener_)
            {
                listener.checkEvaluateCaptures();
            }
        }
    }

    bool GazeboCaricPlugin::CheckLOS(const Vector3d &pi, const Vector3d &pj, gazebo::physics::RayShapePtr &ray)
    {

        // Ray tracing from the slf node to each of the nbr node
        double rtDist;      // Raytracing distance
        double ppDist = 0;  // Peer to peer distance
        string entity_name; // Name of intersected object
        ignition::math::Vector3d start_point;
        ignition::math::Vector3d end_point;
        bool los = false;
        start_point = ignition::math::Vector3d(pi.x(), pi.y(), pi.z());
        end_point = ignition::math::Vector3d(pj.x(), pj.y(), pj.z());

        ray->SetPoints(start_point, end_point);
        ray->GetIntersection(rtDist, entity_name);

        ppDist = (pi - pj).norm();
        if (rtDist >= ppDist - 0.1)
            return true;
        return los;
    }

    bool GazeboCaricPlugin::CheckLOS(const tf::Vector3 &pi, const tf::Vector3 &pj)
    {

        // Ray tracing from the slf node to each of the nbr node
        double rtDist;      // Raytracing distance
        double ppDist = 0;  // Peer to peer distance
        string entity_name; // Name of intersected object
        ignition::math::Vector3d start_point;
        ignition::math::Vector3d end_point;
        bool los = false;
        start_point = ignition::math::Vector3d(pi.getX(), pi.getY(), pi.getZ());
        end_point = ignition::math::Vector3d(pj.getX(), pj.getY(), pj.getZ());

        ray_->SetPoints(start_point, end_point);
        ray_->GetIntersection(rtDist, entity_name);

        ppDist = (pi - pj).length();
        if (rtDist >= ppDist - 0.1)
            return true;
        return los;
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboCaricPlugin);

} // namespace gazebo
