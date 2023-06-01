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
#include "rotors_gazebo_plugins/gazebo_ppcom_plugin.h"

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
#include "rotors_comm/PPComTopology.h"

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
    // Constructors
    IndexedInterestPoint::IndexedInterestPoint()
    {
        detected_order = -1;
        scored_point.x = 0;
        scored_point.y = 0;
        scored_point.z = 0;
        scored_point.intensity = 0;
        scored_point.normal_x  = 0;
        scored_point.normal_y  = 0;
        scored_point.normal_z  = 0;
        scored_point.curvature = 0;
    }
    IndexedInterestPoint::IndexedInterestPoint(const int &detected_order_, const PointXYZIN &scored_point_):
        detected_order(detected_order_), scored_point(scored_point_) {}
    // Simple destructor
    IndexedInterestPoint::~IndexedInterestPoint() {};    

    // PPCom class, helper of the plugin
    PPComNode::PPComNode()
    {
        // Set cov diagonal to -1 to indicate no reception yet
        for (auto &c : this->odom_msg.pose.covariance)
            c = -1;
    }

    PPComNode::PPComNode(const string &name_, const string &role_, const double &offset_)
        : name(name_), role(role_), offset(offset_)
    {
        // Set cov diagonal to -1 to indicate no reception yet
        for (auto &c : this->odom_msg.pose.covariance)
            c = -1;
    }

    PPComNode::PPComNode(const string &name_, const string &role_, const double &offset_,
                         const double &hfov, const double &vfov, const double &cam_x,
                         const double &cam_y, const double &cam_z)
        : name(name_), role(role_), offset(offset_), fov_h(hfov), fov_v(vfov)
    {
        // Set cov diagonal this to -1 to indicate no reception yet
        for (auto &c : this->odom_msg.pose.covariance)
            c = -1;

        topology_mtx = boost::shared_ptr<std::mutex>(new std::mutex());

        Cam_rel_Uav = Vector3d(cam_x, cam_y, cam_z);
    }

    PPComNode::~PPComNode() {}

    Eigen::MatrixXd PPComNode::GetTopology()
    {
       std::lock_guard<std::mutex> lock(*topology_mtx);
       return topology;
    }

    void PPComNode::SetTopology(Eigen::MatrixXd topology_)
    {
       std::lock_guard<std::mutex> lock(*topology_mtx);
       topology = topology_;
    }

    // Plugin definition
    GazeboPPComPlugin::GazeboPPComPlugin()
        : ModelPlugin(), gz_node_handle_(0) {}

    GazeboPPComPlugin::~GazeboPPComPlugin() {}

    void GazeboPPComPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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

        // // Create a rayshape object for communication links
        // ray_topo_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
        //                     (physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

        // // Create rayshape object for camera field of view check
        // ray_inpo_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
        //                     (physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

        //==============================================//
        //========== READ IN PARAMS FROM SDF ===========//
        //==============================================//

        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a robotNamespace.\n";

        if (_sdf->HasElement("linkName"))
            self_link_name_ = _sdf->GetElement("linkName")->Get<string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a linkName.\n";

        if (_sdf->HasElement("ppcomId"))
            ppcom_id_ = _sdf->GetElement("ppcomId")->Get<string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a ppcomId.\n";

        if (_sdf->HasElement("ppcomConfig"))
            ppcom_config_ = _sdf->GetElement("ppcomConfig")->Get<string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify ppcomConfig.\n";

        if (_sdf->HasElement("ppcomHz"))
            ppcom_hz_ = _sdf->GetElement("ppcomHz")->Get<double>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a ppcomId.\n";

        std::string fstring_;
        if (_sdf->HasElement("interestPcd"))
            fstring_ = _sdf->GetElement("interestPcd")->Get<string>();
        else
            gzerr << "[gazebo_caric_plugin] Please specify input interest points.\n";

        // Get the ppcom topic where data is published to
        getSdfParam<string>(_sdf, "ppcomTopic", ppcom_topic_, "ppcom");

        // Report on params obtained from sdf
        printf(KGRN "PPCom Id %s is set. Linkname %s. Config %s!\n" RESET,
               ppcom_id_.c_str(), self_link_name_.c_str(), ppcom_config_.c_str());

        // Open the config file and read the links
        std::ifstream ppcom_config_file(ppcom_config_.c_str());

        readPCloud(fstring_);

        // Read the declared nodes
        ppcom_nodes_.clear();
        string line;
        while (getline(ppcom_config_file, line))
        {
            // Process the line here
            cout << "Reading " << line << endl;

            // Remove spaces in the line
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());

            // Skip the line of its commented out
            if (line[0] == '#')
                continue;

            // Decode the line and construct the node
            vector<string> parts = Util::split(line, ",");
            ppcom_nodes_.push_back(PPComNode(parts[0], parts[1], stod(parts[2]), stod(parts[3]),
                                             stod(parts[4]), stod(parts[5]), stod(parts[6]), stod(parts[7])));
        }

        // Assert that ppcom_id_ is found in the network
        bool ppcom_id_in_network = false;
        for (int idx = 0; idx < ppcom_nodes_.size(); idx++)
            if (ppcom_id_ == ppcom_nodes_[idx].name)
            {
                ppcom_id_in_network = true;
                ppcom_slf_idx_ = idx;
                break;
            }

        assert(ppcom_id_in_network);

        // Number of nodes
        Nnodes_ = ppcom_nodes_.size();

        //==============================================//
        //========== CREATE THE TRANSPORT STRUCTURES ===//
        //==============================================//

        // Create a gazebo node handle and initialize with the namespace
        gz_node_handle_ = transport::NodePtr(new transport::Node());
        gz_node_handle_->Init();

        // Create a ros node
        ros_node_handle_ = new ros::NodeHandle("/firefly" + ppcom_id_ + "rosnode");

        // Subscribe to the odom topics
        int node_idx = -1;
        for (PPComNode &node : ppcom_nodes_)
        {
            node_idx++;

            // Create the subscriber to each nodes
            node.odom_sub = ros_node_handle_->subscribe<nav_msgs::Odometry>("/" + node.name + "/ground_truth/odometry", 1,
                                                                            boost::bind(&GazeboPPComPlugin::OdomCallback, this, _1, node_idx));
            // Publisher for the topology
            if (node.role == "manager")
                node.topo_pub = ros_node_handle_->advertise<rotors_comm::PPComTopology>("/" + node.name + "/ppcom_topology", 1);

            // Publisher for camera pyramid visuals
            node.camera_pyramid_pub = ros_node_handle_->advertise<visualization_msgs::Marker>(
                "/" + node.name + "/visualize", 1);

            // Create the storage of nodes to each object
            node.odom_msg_received = false;

            // Knowlege on the topology
            node.SetTopology(-1 * MatrixXd::Ones(Nnodes_, Nnodes_));

            // Create a rayshape object for communication link
            node.ray_topo = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

            // Create rayshape object for camera field of view check
            node.ray_inpo = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));

            // Create a pointcloud of detected points
            node.InPoLog = {};

            node.CloudDetectedInpoPub = ros_node_handle_->advertise<sensor_msgs::PointCloud2>("/" + node.name + "/detected_interest_points", 1);

            // Preload the interest points for manager
            if (node.role == "manager")
            {

                for(int i = 0; i < cloud_inpo_->size(); i++)
                {
                    node.InPoLog[i] = IndexedInterestPoint(i, cloud_inpo_->points[i]);
                    node.InPoLog[i].scored_point.intensity = 0;
                    assert(node.InPoLog[i].scored_point.curvature == i);
                }
            }

            // GCS no need gimbal
            if (node.name == "gcs")
                continue;

            node.timer_update = ros_node_handle_->createTimer(ros::Duration(1.0 / gimbal_update_hz_),
                                                              boost::bind(&GazeboPPComPlugin::TimerCallback, this, _1, node_idx));

            node.gimbal_sub = ros_node_handle_->subscribe<geometry_msgs::Twist>("/" + node.name + "/command/gimbal", 1,
                                                                                boost::bind(&GazeboPPComPlugin::GimbalCallback, this, _1, node_idx));

            node.gimbal_cmd = Eigen::VectorXd(6);
            node.gimbal_cmd.setZero();
            node.gimbal_cmd_last_update = ros::Time::now();
        }

        cloud_inpo_pub_ = ros_node_handle_->advertise<sensor_msgs::PointCloud2>("/interest_cloud", 100);

        // Listen to the update event. This event is broadcast every simulation iteration.
        last_time_topo_ = world_->SimTime();
        last_time_inpo_ = last_time_topo_;
        this->updateConnection_topology_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPPComPlugin::OnUpdateCheckTopology, this, _1));
        this->updateConnection_interestpoints_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPPComPlugin::OnUpdateCheckInterestPoints, this, _1));
    }

    void GazeboPPComPlugin::GimbalCallback(const geometry_msgs::TwistConstPtr &msg, int node_idx)
    {
        ppcom_nodes_[node_idx].gimbal_cmd_last_update = ros::Time::now();
        ppcom_nodes_[node_idx].gimbal_cmd(0) = msg->linear.x;
        ppcom_nodes_[node_idx].gimbal_cmd(1) = msg->linear.y;
        ppcom_nodes_[node_idx].gimbal_cmd(2) = msg->linear.z;
        ppcom_nodes_[node_idx].gimbal_cmd(3) = msg->angular.x;
        ppcom_nodes_[node_idx].gimbal_cmd(4) = msg->angular.y;
        ppcom_nodes_[node_idx].gimbal_cmd(5) = msg->angular.z;
    }

    void GazeboPPComPlugin::OdomCallback(const nav_msgs::OdometryConstPtr &msg, int node_idx)
    {
        ppcom_nodes_[node_idx].odom_msg = *msg;
        ppcom_nodes_[node_idx].odom_msg_received = true;
    }

    void GazeboPPComPlugin::OnUpdateCheckTopology(const common::UpdateInfo &_info)
    {
        if (kPrintOnUpdates)
            gzdbg << __FUNCTION__ << "() called." << endl;

        // common::Time current_time = world_->SimTime();

        if (ppcom_nodes_[ppcom_slf_idx_].role != "manager")
            return;

        TicToc tt_topo;

        // Update the topology
        UpdateTopology();

        tt_topo.Toc();

        // printf("Topo Time: %.3f. Ip Time: %.3f.\n", tt_topo.GetLastStop());
    }

    void GazeboPPComPlugin::UpdateTopology()
    {
        gazebo::common::Time current_time = world_->SimTime();
        double dt_topo = (current_time - last_time_topo_).Double();
        // Update the ray casting every 0.1s
        if (dt_topo > 1.0 / ppcom_hz_)
        {
            last_time_topo_ = current_time;

            Eigen::MatrixXd distMat = -1 * Eigen::MatrixXd::Ones(Nnodes_, Nnodes_);
            vector<vector<bool>> los_check(Nnodes_, vector<bool>(Nnodes_, false));

            for (int i = 0; i < Nnodes_; i++)
            {
                PPComNode &node_i = ppcom_nodes_[i];

                if (!node_i.odom_msg_received)
                    continue;

                // Create the start points
                Vector3d pi(node_i.odom_msg.pose.pose.position.x,
                            node_i.odom_msg.pose.pose.position.y,
                            node_i.odom_msg.pose.pose.position.z);

                for (int j = i + 1; j < Nnodes_; j++)
                {
                    PPComNode &node_j = ppcom_nodes_[j];

                    if (!node_j.odom_msg_received)
                        continue;

                    assert(node_i.name != node_j.name);

                    // Find the position of the neighbour
                    Vector3d pj(node_j.odom_msg.pose.pose.position.x,
                                node_j.odom_msg.pose.pose.position.y,
                                node_j.odom_msg.pose.pose.position.z);

                    los_check[i][j] = los_check[j][i] = CheckTopoLOS(pi, node_i.offset, pj, node_j.offset, node_i.ray_topo);

                    // Assign the distance if there is line of sight
                    if (los_check[i][j])
                    {
                        distMat(i, j) = (pi - pj).norm();
                        distMat(j, i) = distMat(i, j);
                    }
                }

                node_i.SetTopology(distMat);
            }

            // Publish the information of the topology
            rotors_comm::PPComTopology topo_msg;
            topo_msg.header.frame_id = "world";
            topo_msg.header.stamp = ros::Time::now();

            topo_msg.node_id.clear();
            for (PPComNode &node : ppcom_nodes_)
            {
                topo_msg.node_id.push_back(node.name);
                topo_msg.node_role.push_back(node.role);
                topo_msg.node_odom.push_back(node.odom_msg);
                // printf("Node %s. OdomCov: %f\n", node.name.c_str(), node.odom_msg.pose.covariance[0]);
            }

            topo_msg.range.clear();
            for (int i = 0; i < Nnodes_; i++)
                for (int j = i + 1; j < Nnodes_; j++)
                    topo_msg.range.push_back(distMat(i, j));

            ppcom_nodes_[ppcom_slf_idx_].topo_pub.publish(topo_msg);
        }
    }

    void GazeboPPComPlugin::OnUpdateCheckInterestPoints(const common::UpdateInfo &_info)
    {
        if (kPrintOnUpdates)
            gzdbg << __FUNCTION__ << "() called." << endl;

        // common::Time current_time = world_->SimTime();

        if (ppcom_nodes_[ppcom_slf_idx_].role != "manager")
            return;

        TicToc tt_ip;

        // Update the interest point
        UpdateInterestPoints();

        tt_ip.Toc();

        // Calculate and publish the score
        TallyScore();

        // printf("Ip Time: %.3f.\n", tt_ip.GetLastStop());
    }

    void GazeboPPComPlugin::UpdateInterestPoints()
    {
        gazebo::common::Time current_time = world_->SimTime();
        double dt_inspection = (current_time - last_time_inpo_).Double();
        if (dt_inspection > 1.0 / cam_evaluate_hz_)
        {
            last_time_inpo_ = current_time;
            for (int i = 0; i < Nnodes_; i++)
            {
                PPComNode &node_i = ppcom_nodes_[i];
                std::map<int, IndexedInterestPoint> &nodeIPLog = node_i.InPoLog;

                // GCS no need to check detection
                if (node_i.name == "gcs")
                    continue;

                if (!node_i.odom_msg_received)
                    continue;

                visualization_msgs::Marker m;
                m.type = visualization_msgs::Marker::LINE_STRIP;
                m.action = visualization_msgs::Marker::DELETEALL;
                m.id = 1; // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
                m.ns = "view_agent_1";

                m.color.a = 1.0; // Don't forget to set the alpha!
                m.color.r = 0.0;
                m.color.g = 1.0;
                m.color.b = 0.0;

                m.scale.x = 0.15;
                m.scale.y = 0.0001;
                m.scale.z = 0.0001;
                m.header.stamp = ros::Time::now();
                m.header.frame_id = "world";
                node_i.camera_pyramid_pub.publish(m);

                // Create the start points
                myTf<double> tf_uav(node_i.odom_msg);
                // Vector3d pi(node_i.odom_msg.pose.pose.position.x,
                //             node_i.odom_msg.pose.pose.position.y,
                //             node_i.odom_msg.pose.pose.position.z);
                Vector3d p_cam_world = tf_uav * node_i.Cam_rel_Uav;
                PointXYZIN pos_cam;
                pos_cam.x = p_cam_world(0);
                pos_cam.y = p_cam_world(1);
                pos_cam.z = p_cam_world(2);
                // node_i.cam_ypr = Vector3d(0.0, node_i., tf_uav.yaw());
                Vector3d ypr(tf_uav.yaw() + node_i.cam_rpy(2) / M_PI * 180.0, node_i.cam_rpy(1) / M_PI * 180.0, 0.0);
                myTf<double> tf_cam(Util::YPR2Rot(ypr), p_cam_world);

                std::vector<int> k_idx; std::vector<float> k_distances;
                kdTreeInterestPts_.radiusSearch(pos_cam, node_i.visible_radius, k_idx, k_distances);

                // odometry message gives velocity in body frame
                Vector3d v_uav_world = tf_uav.rot * Vector3d(node_i.odom_msg.twist.twist.linear.x,
                                                             node_i.odom_msg.twist.twist.linear.y,
                                                             node_i.odom_msg.twist.twist.linear.z);

                // angvel uav is expressed in body frame
                Vector3d angv_uav(node_i.odom_msg.twist.twist.angular.x,
                                  node_i.odom_msg.twist.twist.angular.y,
                                  node_i.odom_msg.twist.twist.angular.z);

                Vector3d v_cam_world = v_uav_world + tf_uav.rot * Util::skewSymmetric(angv_uav) * node_i.Cam_rel_Uav;

                for (int j = 0; j < k_idx.size(); j++) //
                {
                    PointXYZIN &kpoint = cloud_inpo_->points[k_idx[j]];
                    assert(kpoint.curvature == k_idx[j]);
                    Vector3d InPoint_world(kpoint.x, kpoint.y, kpoint.z);
                    Vector3d InPoint_cam = tf_cam.inverse() * InPoint_world;

                    if (InPoint_cam(0) <= 0.0)
                        continue;

                    double horz_angle = atan(InPoint_cam(1) / InPoint_cam(0)) / M_PI * 180.0;
                    double vert_angle = atan(InPoint_cam(2) / InPoint_cam(0)) / M_PI * 180.0;

                    if (horz_angle > node_i.fov_h / 2 || horz_angle < -node_i.fov_h / 2 ||
                        vert_angle > node_i.fov_v / 2 || vert_angle < -node_i.fov_v / 2)
                        continue;

                    double visualized = false;
                    if (CheckInterestPointLOS(p_cam_world, InPoint_world, node_i.ray_inpo))
                    {
                        // Temporary point to be evaluated for actual detection
                        PointXYZIN detected_point = kpoint; detected_point.intensity = 0;

                        Vector3d rpy_rate = Vector3d(0.0, node_i.cam_rpy_rate(1), node_i.cam_rpy_rate(2) + node_i.odom_msg.twist.twist.angular.z);
                        Vector3d v_point_in_cam = -Util::skewSymmetric(rpy_rate) * InPoint_cam -
                                                   tf_cam.rot.inverse() * v_cam_world;
                        // horizontal camera pixels
                        double pixel_move_v = node_i.focal_length / InPoint_cam(0) * v_point_in_cam(1) *
                                              node_i.exposure / node_i.pixel_size;
                        // vertical camera pixels
                        double pixel_move_u = node_i.focal_length / InPoint_cam(0) * v_point_in_cam(2) *
                                              node_i.exposure / node_i.pixel_size;

                        double score1 = 1.0 - min(max(fabs(pixel_move_v), fabs(pixel_move_u)), 1.0);

                        // compute resolution requirement mm per pixel
                        Vector3d Normal_world(kpoint.normal_x, kpoint.normal_y, kpoint.normal_z);
                        Vector3d Normal_cam = (tf_cam.rot.inverse() * Normal_world).normalized();
                        // Normal_cam = Vector3d(-1.0, 0.0, 0.0); //for testing only
                        Vector3d x_disp(-Normal_cam(2), 0.0, Normal_cam(0)); // the gradient projected on the x-z plane
                        Vector3d inPoint_xplus = InPoint_cam + 0.0005 * x_disp;
                        Vector3d inPoint_xminus = InPoint_cam - 0.0005 * x_disp;
                        double v_plus = inPoint_xplus(2) * node_i.focal_length / inPoint_xplus(0);
                        double v_minus = inPoint_xminus(2) * node_i.focal_length / inPoint_xminus(0);
                        double mm_per_pixel_v = node_i.pixel_size / fabs(v_plus - v_minus);

                        Vector3d y_disp(-Normal_cam(1), Normal_cam(0), 0.0); // the gradient projected on the x-y plane
                        Vector3d inPoint_yplus = InPoint_cam + 0.0005 * y_disp;
                        Vector3d inPoint_yminus = InPoint_cam - 0.0005 * y_disp;
                        double u_plus = inPoint_yplus(1) * node_i.focal_length / inPoint_yplus(0);
                        double u_minus = inPoint_yminus(1) * node_i.focal_length / inPoint_yminus(0);
                        double mm_per_pixel_u = node_i.pixel_size / fabs(u_plus - u_minus);

                        double score2 = max(0.0, 1.0 - max(max(mm_per_pixel_v, mm_per_pixel_u) - 6.0, 0.0) / 10.0);
                        double score = score1 * score2;
                        if (inPoint_yplus(0) < 0 || inPoint_yminus(0) < 0 ||
                            inPoint_xplus(0) < 0 || inPoint_xminus(0) < 0)
                        {
                            ROS_INFO("NOT RIGHT! X smaller than zero!!");
                            continue;
                        }

                        // std::cout<<"mm_per pixel u is "<<mm_per_pixel_u<<"mm_per pixel v is "<<mm_per_pixel_v<<std::endl;
                        if (score > cloud_inpo_->points[k_idx[j]].intensity)
                            cloud_inpo_->points[k_idx[j]].intensity = score;

                        detected_point.intensity = score;
                        int point_idx = (int)detected_point.curvature;

                        if (nodeIPLog.find(point_idx) == nodeIPLog.end())
                            nodeIPLog[point_idx] = IndexedInterestPoint(nodeIPLog.size(), detected_point);
                        else if (nodeIPLog[point_idx].scored_point.intensity < detected_point.intensity)
                            nodeIPLog[point_idx] = IndexedInterestPoint(nodeIPLog[point_idx].detected_order, detected_point); 

                        // CloudDtectedInPo->clear();
                        // for(map< int, pair<int, PointXYZIN> >::iterator itr = nodeIPLog.begin(); itr != nodeIPLog.end(); itr++)
                        //     CloudDtectedInPo->push_back(itr->second.second);
                        
                        // // if (visualized) continue;
                        // visualized = true;
                        // visualization_msgs::Marker m;
                        // m.type   = visualization_msgs::Marker::ARROW;
                        // m.action = visualization_msgs::Marker::ADD;
                        // m.id     = 1; // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
                        // m.ns     = "view_agent_1";

                        // m.color.a = 1.0; // Don't forget to set the alpha!
                        // m.color.r = 0.0;
                        // m.color.g = 1.0;
                        // m.color.b = 0.0;

                        // m.scale.x = 0.15;
                        // m.scale.y = 0.25;
                        // m.scale.z = 0.4;
                        // m.header.stamp = ros::Time::now();
                        // m.header.frame_id = "world";
                        // geometry_msgs::Point point1;
                        // point1.x = InPoint_world(0);
                        // point1.y = InPoint_world(1);
                        // point1.z = InPoint_world(2);
                        // m.points.push_back(point1);
                        // Vector3d v_point_world = tf_cam.rot* v_point_in_cam*1.0;
                        // point1.x = InPoint_world(0) + v_point_world(0);
                        // point1.y = InPoint_world(1) + v_point_world(1);
                        // point1.z = InPoint_world(2) + v_point_world(2);
                        // m.points.push_back(point1);
                        // node_i.camera_pyramid_pub.publish(m);

                        // ROS_INFO("Setting points to high intensity!!");
                    }
                }

                // Copy the detected points to the cloud and publish it
                CloudXYZINPtr CloudDtectedInPo(new CloudXYZIN());
                CloudDtectedInPo->resize(nodeIPLog.size());

                // Copy the points into the pointcloud structure
                #pragma omp parallel for num_threads(MAX_THREADS)
                for(map< int, IndexedInterestPoint >::iterator itr = nodeIPLog.begin(); itr != nodeIPLog.end(); itr++)
                {
                    CloudDtectedInPo->points[itr->second.detected_order] = itr->second.scored_point;
                    CloudDtectedInPo->points[itr->second.detected_order].curvature = itr->second.detected_order;
                }

                // Publish the pointcloud
                Util::publishCloud(node_i.CloudDetectedInpoPub, *CloudDtectedInPo, ros::Time::now(), "world");   

                // Visualize the frustrum
                std::vector<Vector3d> point_list_in_world;
                point_list_in_world.push_back(p_cam_world);
                for (double k : {1.0, -1.0})
                {
                    for (double l : {1.0, -1.0})
                    {
                        Vector3d point_in_cam(node_i.visible_radius,
                                              k * tan(node_i.fov_h * 0.008726646) * node_i.visible_radius,
                                              l * tan(node_i.fov_v * 0.008726646) * node_i.visible_radius);
                        Vector3d point_in_world = tf_cam * point_in_cam;
                        point_list_in_world.push_back(point_in_world);
                    }
                    point_list_in_world.push_back(p_cam_world);
                }
                point_list_in_world.push_back(point_list_in_world[1]);
                point_list_in_world.push_back(point_list_in_world[4]);
                point_list_in_world.push_back(point_list_in_world[5]);
                point_list_in_world.push_back(point_list_in_world[2]);

                // visualization_msgs::Marker m;
                // m.type   = visualization_msgs::Marker::LINE_STRIP;
                // m.action = visualization_msgs::Marker::DELETEALL;
                // m.id     = 1; // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
                // m.ns     = "view_agent_1";

                // m.color.a = 1.0; // Don't forget to set the alpha!
                // m.color.r = 0.0;
                // m.color.g = 1.0;
                // m.color.b = 0.0;

                // m.scale.x = 0.15;
                // m.scale.y = 0.0001;
                // m.scale.z = 0.0001;
                // m.header.stamp = ros::Time::now();
                // m.header.frame_id = "world";
                // node_i.camera_pyramid_pub.publish(m);

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
                    geometry_msgs::Point point1;
                    point1.x = point(0);
                    point1.y = point(1);
                    point1.z = point(2);
                    m.points.push_back(point1);
                }
                node_i.camera_pyramid_pub.publish(m);
            }

            Util::publishCloud(cloud_inpo_pub_, *cloud_inpo_, ros::Time::now(), "world");
        }
    }
    
    void GazeboPPComPlugin::TallyScore()
    {
        // Go through the log of each node in line of sight of GCS to tally the score
        for (int i = 0; i < Nnodes_; i++)
        {
            PPComNode &node_i = ppcom_nodes_[i];
            std::map<int, IndexedInterestPoint> &node_i_iplog = node_i.InPoLog;

            // Only job of the GCS
            if (node_i.name != "gcs")
                continue;
            
            Eigen::MatrixXd topology = node_i.GetTopology();

            // printf("Topo: \n");
            // cout << topology << endl;

            // Check for other nodes in line of sight
            for(int j = i+1; j < Nnodes_; j++)
            {
                // If no link, continue
                if(topology(i, j) <= 0.0)
                    continue;

                PPComNode &node_j = ppcom_nodes_[j];
                std::map<int, IndexedInterestPoint> &node_j_iplog = node_j.InPoLog;    

                // Check the IPLog of the neigbour and update the score
                #pragma omp parallel for num_threads(MAX_THREADS)
                for(map< int, IndexedInterestPoint >::iterator itr = node_j_iplog.begin(); itr != node_j_iplog.end(); itr++)
                {
                    int global_idx = itr->second.scored_point.curvature;
                    float ip_score_from_nbr = itr->second.scored_point.intensity;

                    assert(node_i_iplog[global_idx].scored_point.curvature == global_idx);

                    if (node_i_iplog[global_idx].scored_point.intensity < ip_score_from_nbr)
                        node_i_iplog[global_idx].scored_point.intensity = ip_score_from_nbr;
                }
            }

            sensor_msgs::PointCloud DetectedIP;
            sensor_msgs::ChannelFloat32 normal_x; normal_x.name = "normal_x"; DetectedIP.channels.push_back(normal_x);
            sensor_msgs::ChannelFloat32 normal_y; normal_y.name = "normal_y"; DetectedIP.channels.push_back(normal_y);
            sensor_msgs::ChannelFloat32 normal_z; normal_z.name = "normal_z"; DetectedIP.channels.push_back(normal_z);
            sensor_msgs::ChannelFloat32 score; score.name = "score"; DetectedIP.channels.push_back(score);

            int total_detected = 0;
            double total_score = 0;
            for(map< int, IndexedInterestPoint >::iterator itr = node_i_iplog.begin(); itr != node_i_iplog.end(); itr++)
            {
                if (itr->second.scored_point.intensity > 0.0)
                {
                    total_detected++;
                    total_score += itr->second.scored_point.intensity;

                    geometry_msgs::Point32 detected_point;
                    detected_point.x = itr->second.scored_point.x;
                    detected_point.y = itr->second.scored_point.y;
                    detected_point.z = itr->second.scored_point.z;

                    DetectedIP.points.push_back(detected_point);
                    DetectedIP.channels[0].values.push_back(itr->second.scored_point.normal_x);
                    DetectedIP.channels[1].values.push_back(itr->second.scored_point.normal_y);
                    DetectedIP.channels[2].values.push_back(itr->second.scored_point.normal_z);
                    DetectedIP.channels[3].values.push_back(itr->second.scored_point.intensity);
                }
            }
            string report = myprintf("Detected: %4d / %4d. Score: %6.3f\n",
                                     total_detected, node_i_iplog.size(), total_score);

            // Visualize the score on rviz
            visualization_msgs::Marker marker;
            marker.header.frame_id = "score_report";
            marker.header.stamp = ros::Time::now();
            marker.ns = node_i.name;
            marker.id = 0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = node_i.odom_msg.pose.pose.position.x;
            marker.pose.position.y = node_i.odom_msg.pose.pose.position.y;
            marker.pose.position.z = node_i.odom_msg.pose.pose.position.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 2.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.text = report;

            static ros::Publisher score_viz_pub = ros_node_handle_->advertise<visualization_msgs::Marker>("/" + node_i.name + "/score_text", 10);
            score_viz_pub.publish(marker);

            static ros::Publisher score_pub = ros_node_handle_->advertise<sensor_msgs::PointCloud>("/" + node_i.name + "/score", 10);
            score_pub.publish(DetectedIP);
        }
    }
    
    void GazeboPPComPlugin::TimerCallback(const ros::TimerEvent &, int node_idx)
    {
        PPComNode &node_i = ppcom_nodes_[node_idx];
        if (node_i.name == "gcs")
            return;
        if (node_i.gimbal_cmd(0) < -1e-7 &&
            (ros::Time::now() - node_i.gimbal_cmd_last_update).toSec() > 2.0 / gimbal_update_hz_)
        {
            // change from rate control to angle control
            node_i.gimbal_cmd(0) = 0.1;
            node_i.gimbal_cmd(1) = node_i.cam_rpy(1);
            node_i.gimbal_cmd(2) = node_i.cam_rpy(2);
        }
        if (node_i.gimbal_cmd(0) > 0.0) // angle control mode
        {
            double pitch_cmd = min(gimbal_pitch_max_, max(-gimbal_pitch_max_, Util::wrapToPi(node_i.gimbal_cmd(1))));
            double yaw_cmd = min(gimbal_yaw_max_, max(-gimbal_yaw_max_, Util::wrapToPi(node_i.gimbal_cmd(2))));
            // double pitch_cmd = min(gimbal_pitch_max_, max(-gimbal_pitch_max_, (node_i.gimbal_cmd(1))));
            // double yaw_cmd = min(gimbal_yaw_max_, max(-gimbal_yaw_max_, (node_i.gimbal_cmd(2))));

            node_i.cam_rpy_rate(1) = max(-gimbal_rate_max_, min(gimbal_rate_max_,
                                                                (pitch_cmd - node_i.cam_rpy(1)) * gimbal_update_hz_));
            node_i.cam_rpy_rate(2) = max(-gimbal_rate_max_, min(gimbal_rate_max_,
                                                                (yaw_cmd - node_i.cam_rpy(2)) * gimbal_update_hz_));
            node_i.cam_rpy(1) = node_i.cam_rpy(1) + node_i.cam_rpy_rate(1) * 1.0 / gimbal_update_hz_;
            node_i.cam_rpy(2) = node_i.cam_rpy(2) + node_i.cam_rpy_rate(2) * 1.0 / gimbal_update_hz_;
        }
        if (node_i.gimbal_cmd(0) < -1e-7) // rate control mode
        {
            node_i.cam_rpy_rate(1) = max(-gimbal_rate_max_, min(gimbal_rate_max_, node_i.gimbal_cmd(4)));
            double pitch = node_i.cam_rpy(1) + node_i.cam_rpy_rate(1) * 1.0 / gimbal_update_hz_;
            node_i.cam_rpy(1) = max(-gimbal_pitch_max_, min(gimbal_pitch_max_, pitch));

            node_i.cam_rpy_rate(2) = max(-gimbal_rate_max_, min(gimbal_rate_max_, node_i.gimbal_cmd(5)));
            double yaw = node_i.cam_rpy(2) + node_i.cam_rpy_rate(2) * 1.0 / gimbal_update_hz_;
            node_i.cam_rpy(2) = max(-gimbal_yaw_max_, min(gimbal_yaw_max_, yaw));
        }
    }

    void GazeboPPComPlugin::readPCloud(std::string filename)
    {
        cloud_inpo_ = CloudXYZINPtr(new CloudXYZIN);
        if (pcl::io::loadPCDFile<PointXYZIN>(filename, *cloud_inpo_) == -1) // load point cloud file
        {
            PCL_ERROR("Could not read the file");
            return;
        }

        // Add the point index into the curvature field
        #pragma omp parallel for num_threads(MAX_THREADS)
        for(int i = 0; i < cloud_inpo_->size(); i++)
            cloud_inpo_->points[i].curvature = i;

        std::cout << "Loaded" << cloud_inpo_->width * cloud_inpo_->height
                  << "data points with the following fields: "
                  << std::endl;

        // for(size_t i = 0; i < cloud_inpo_->points.size(); ++i)
        //     std::cout << "    " << cloud_inpo_->points[i].x
        //               << " "    << cloud_inpo_->points[i].y
        //               << " "    << cloud_inpo_->points[i].z
        //               << " "    << cloud_inpo_->points[i].normal_x
        //               << " "    << cloud_inpo_->points[i].normal_y
        //               << " "    << cloud_inpo_->points[i].normal_z << std::endl;

        kdTreeInterestPts_.setInputCloud(cloud_inpo_);
    }

    bool GazeboPPComPlugin::CheckTopoLOS(const Vector3d &pi, double bi, const Vector3d &pj, double bj, gazebo::physics::RayShapePtr &ray)
    {
        vector<Vector3d> Pi;
        Pi.push_back(pi + Vector3d(bi, 0, 0));
        Pi.push_back(pi - Vector3d(bi, 0, 0));
        Pi.push_back(pi + Vector3d(0, bi, 0));
        Pi.push_back(pi - Vector3d(0, bi, 0));
        Pi.push_back(pi + Vector3d(0, 0, bi));
        Pi.push_back(pi - Vector3d(0, 0, bi));
        // Make sure the virtual antenna does not go below the ground
        Pi.back().z() = max(0.1, Pi.back().z());

        vector<Vector3d> Pj;
        Pj.push_back(pj + Vector3d(bj, 0, 0));
        Pj.push_back(pj - Vector3d(bj, 0, 0));
        Pj.push_back(pj + Vector3d(0, bj, 0));
        Pj.push_back(pj - Vector3d(0, bj, 0));
        Pj.push_back(pj + Vector3d(0, 0, bj));
        Pj.push_back(pj - Vector3d(0, 0, bj));
        // Make sure the virtual antenna does not go below the ground
        Pj.back().z() = max(0.1, Pj.back().z());

        // Ray tracing from the slf node to each of the nbr node
        double rtDist;      // Raytracing distance
        double ppDist = 0;  // Peer to peer distance
        string entity_name; // Name of intersected object
        ignition::math::Vector3d start_point;
        ignition::math::Vector3d end_point;
        bool los = false;
        for (Vector3d &pa : Pi)
        {
            start_point = ignition::math::Vector3d(pa.x(), pa.y(), pa.z());

            for (Vector3d &pb : Pj)
            {
                end_point = ignition::math::Vector3d(pb.x(), pb.y(), pb.z());

                ray->SetPoints(start_point, end_point);
                ray->GetIntersection(rtDist, entity_name);

                ppDist = (pa - pb).norm();
                if (entity_name == "" || (rtDist >= ppDist - 0.1))
                    return true;
            }
        }
        return los;
    }

    bool GazeboPPComPlugin::CheckInterestPointLOS(const Eigen::Vector3d &pi, const Eigen::Vector3d &pj, gazebo::physics::RayShapePtr &ray)
    {
        // Ray tracing from the optical origin to the interestpoint
        double rtDist;      // Raytracing distance
        double ppDist = 0;  // Peer to peer distance
        string entity_name; // Name of intersected object
        ignition::math::Vector3d start_point;
        ignition::math::Vector3d end_point;
        bool los = false;
        start_point = ignition::math::Vector3d(pi(0), pi(1), pi(2));
        end_point = ignition::math::Vector3d(pj(0), pj(1), pj(2));

        ray->SetPoints(start_point, end_point);
        ray->GetIntersection(rtDist, entity_name);

        ppDist = (pi - pj).norm();
        if (fabs(rtDist - ppDist) <= 0.025)
            return true;
        return los;
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboPPComPlugin);

} // namespace gazebo
