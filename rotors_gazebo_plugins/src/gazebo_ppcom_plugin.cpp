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
    // PPCom class, helper of the plugin
    PPComNode::PPComNode()
    {
        // Set cov diagonal to -1 to indicate no reception yet
        for(auto &c : this->odom_msg.pose.covariance)
            c = -1;
    }
    PPComNode::PPComNode(const string &name_, const string &role_, const double &offset_)
        : name(name_), role(role_), offset(offset_)
    {
        // Set cov diagonal this to -1 to indicate no reception yet
        for(auto &c : this->odom_msg.pose.covariance)
            c = -1;
    }

    PPComNode::PPComNode(const string &name_, const string &role_, const double &offset_,
                         const double &hfov, const double &vfov, const double &cam_x,
                         const double &cam_y, const double &cam_z)
        : name(name_), role(role_), offset(offset_), fov_h(hfov), fov_v(vfov)
    {
        // Set cov diagonal this to -1 to indicate no reception yet
        for(auto &c : this->odom_msg.pose.covariance)
            c = -1;
        Cam_rel_Uav = Vector3d(cam_x, cam_y, cam_z);
    }

    PPComNode::~PPComNode() {}

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
        model_   = _model;
        world_   = model_->GetWorld();
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
                                            stod(parts[4]), stod(parts[5]), stod(parts[6]),
                                            stod(parts[7])));
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
            node.odom_sub
                = ros_node_handle_->subscribe<nav_msgs::Odometry>("/" + node.name + "/ground_truth/odometry", 1,
                                                                  boost::bind(&GazeboPPComPlugin::OdomCallback, this, _1, node_idx));

            node.topo_pub
                = ros_node_handle_->advertise<rotors_comm::PPComTopology>("/" + node.name + "/ppcom_topology", 1);
            node.camera_pyramid_pub = ros_node_handle_->advertise<visualization_msgs::Marker>(
                                                    "/" + node.name + "/visualize", 1);

            // Create the storage of nodes to each object
            node.odom_msg_received = false;

            // Create rayshape object
            node.ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
                        (physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));
        }
        cloud_pub_ = ros_node_handle_->advertise<sensor_msgs::PointCloud2>("/interest_cloud", 100);

        // Listen to the update event. This event is broadcast every simulation iteration.
        last_time_ = world_->SimTime();
        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPPComPlugin::OnUpdate, this, _1));
    }

    void GazeboPPComPlugin::OdomCallback(const nav_msgs::OdometryConstPtr &msg, int node_idx)
    {
        ppcom_nodes_[node_idx].odom_msg = *msg;
        ppcom_nodes_[node_idx].odom_msg_received = true;
        
        // nav_msgs::Odometry &odom = odom_msgs[node_idx];

        // Eigen::Vector3d ypr = Eigen::Quaterniond(odom.pose.pose.orientation.w,
        //                                          odom.pose.pose.orientation.x,
        //                                          odom.pose.pose.orientation.y,
        //                                          odom.pose.pose.orientation.z).toRotationMatrix().eulerAngles(0, 1, 2);

        // printf("Callback of %s. From node %s. XYZ: %6.3f, %6.3f, %6.3f. RPY: %6.3f, %6.3f, %6.3f.\n",
        //         ppcom_id_.c_str(), ppcom_nodes_[node_idx].name.c_str(),
        //         odom.pose.pose.position.x,
        //         odom.pose.pose.position.y,
        //         odom.pose.pose.position.z,
        //         ypr.x(), ypr.y(), ypr.z());
    }

    void GazeboPPComPlugin::OnUpdate(const common::UpdateInfo &_info)
    {
        if (kPrintOnUpdates)
            gzdbg << __FUNCTION__ << "() called." << endl;

        common::Time current_time = world_->SimTime();
        double dt = (current_time - last_time_).Double();
        double dt2 = (current_time - last_time_cam_).Double();

        if (ppcom_nodes_[ppcom_slf_idx_].role != "manager")
            return;

        // Update the ray casting every 0.1s
        if (dt > 1.0/ppcom_hz_)
        {
            last_time_ = current_time;

            Eigen::MatrixXd distMat = -1*Eigen::MatrixXd::Ones(Nnodes_, Nnodes_);
            vector<vector<bool>> los_check(Nnodes_, vector<bool>(Nnodes_, false));

            for(int i = 0; i < Nnodes_; i ++)
            {
                PPComNode &node_i = ppcom_nodes_[i];

                if (!node_i.odom_msg_received)
                    continue;

                // Create the start points
                Vector3d pi(node_i.odom_msg.pose.pose.position.x,
                            node_i.odom_msg.pose.pose.position.y,
                            node_i.odom_msg.pose.pose.position.z);
                
                for (int j = i+1; j < Nnodes_; j++)
                {
                    PPComNode &node_j = ppcom_nodes_[j];

                    if (!node_j.odom_msg_received)
                        continue;

                    assert(node_i.name != node_j.name);

                    // Find the position of the neighbour
                    Vector3d pj(node_j.odom_msg.pose.pose.position.x,
                                node_j.odom_msg.pose.pose.position.y,
                                node_j.odom_msg.pose.pose.position.z);

                    los_check[i][j] = los_check[j][i] = CheckLOS(pi, node_i.offset, pj, node_j.offset, node_i.ray);
                    
                    // Assign the distance if there is line of sight
                    if (los_check[i][j])
                    {
                        distMat(i, j) = (pi - pj).norm();
                        distMat(j, i) = distMat(i, j);
                    }
                }
            }
            
            
            /* #region Publish visualization of the topology --------------------------------------------------------*/

            // typedef visualization_msgs::Marker RosVizMarker;
            // typedef std_msgs::ColorRGBA RosVizColor;
            // typedef ros::Publisher RosPub;

            // struct VizAid
            // {
            //     bool           inited = false;
            //     RosVizColor    color  = RosVizColor();
            //     RosVizMarker   marker = RosVizMarker();
            //     ros::Publisher rosPub = RosPub();
            // };

            // // Predefined colors
            // static RosVizColor los_color;
            // los_color.r = 0.0;
            // los_color.g = 1.0;
            // los_color.b = 0.5;
            // los_color.a = 1.0;

            // static RosVizColor nlos_color;
            // nlos_color.r = 1.0;
            // nlos_color.g = 0.65;
            // nlos_color.b = 0.0;
            // nlos_color.a = 1.0;

            // // Create the los marker
            // static vector<VizAid> vizAid(Nnodes_);
            // VizAid &vizAidSelf = vizAid[ppcom_slf_idx_];
            
            // // Initialize the loop marker
            // if (!vizAidSelf.inited)
            // {
            //     vizAidSelf.rosPub = ros_node_handle_->advertise<RosVizMarker>("/" + ppcom_id_ + "/los_marker", 1);

            //     // Set up the loop marker
            //     vizAidSelf.marker.header.frame_id = "world";
            //     vizAidSelf.marker.ns       = "loop_marker";
            //     vizAidSelf.marker.type     = visualization_msgs::Marker::LINE_LIST;
            //     vizAidSelf.marker.action   = visualization_msgs::Marker::ADD;
            //     vizAidSelf.marker.pose.orientation.w = 1.0;
            //     vizAidSelf.marker.lifetime = ros::Duration(0);
            //     vizAidSelf.marker.id       = 0;

            //     vizAidSelf.marker.scale.x = 0.15;
            //     vizAidSelf.marker.scale.y = 0.15;
            //     vizAidSelf.marker.scale.z = 0.15;

            //     vizAidSelf.marker.color.r = 0.0;
            //     vizAidSelf.marker.color.g = 1.0;
            //     vizAidSelf.marker.color.b = 1.0;
            //     vizAidSelf.marker.color.a = 1.0;
    
            //     vizAidSelf.color = los_color;

            //     vizAidSelf.inited = true;
            // }

            // vizAidSelf.marker.points.clear();
            // vizAidSelf.marker.colors.clear();

            // for(int i = 0; i < Nnodes_; i++)
            // {
            //     for (int j = i+1; j < Nnodes_; j++)
            //     {
            //         if(los_check[i][j])
            //         {

            //             vizAidSelf.marker.points.push_back(ppcom_nodes_[i].odom_msg.pose.pose.position);
            //             vizAidSelf.marker.colors.push_back(los_color);

            //             vizAidSelf.marker.points.push_back(ppcom_nodes_[j].odom_msg.pose.pose.position);
            //             vizAidSelf.marker.colors.push_back(los_color);
            //         }
            //         else if(ppcom_nodes_[i].odom_msg_received && ppcom_nodes_[j].odom_msg_received)
            //         {
            //             vizAidSelf.marker.points.push_back(ppcom_nodes_[i].odom_msg.pose.pose.position);
            //             vizAidSelf.marker.colors.push_back(nlos_color);

            //             vizAidSelf.marker.points.push_back(ppcom_nodes_[j].odom_msg.pose.pose.position);
            //             vizAidSelf.marker.colors.push_back(nlos_color);
            //         }
                    
            //     }
            // }

            // vizAidSelf.rosPub.publish(vizAidSelf.marker);

            /* #endregion Publish visualization of the topology -----------------------------------------------------*/


            // Publish the information of the topology
            rotors_comm::PPComTopology topo_msg;
            topo_msg.header.frame_id = "world";
            topo_msg.header.stamp = ros::Time::now();

            topo_msg.node_id.clear();
            for(PPComNode &node : ppcom_nodes_)
            {
                topo_msg.node_id.push_back(node.name);
                topo_msg.node_role.push_back(node.role);
                topo_msg.node_odom.push_back(node.odom_msg);
                // printf("Node %s. OdomCov: %f\n", node.name.c_str(), node.odom_msg.pose.covariance[0]);
            }
            
            topo_msg.range.clear();
            for(int i = 0; i < Nnodes_; i++)
                for(int j = i+1; j < Nnodes_; j++)
                    topo_msg.range.push_back(distMat(i, j));
            
            ppcom_nodes_[ppcom_slf_idx_].topo_pub.publish(topo_msg);
        }
        if (dt2 > 1.0/cam_evaluate_hz_)
        {
            last_time_cam_ = current_time;
            for(int i = 0; i < Nnodes_; i ++)
            {
                PPComNode &node_i = ppcom_nodes_[i];
                if (node_i.name == "gcs") continue;
                
                if (!node_i.odom_msg_received)
                    continue;

                // Create the start points
                myTf<double> tf_uav(node_i.odom_msg);
                // Vector3d pi(node_i.odom_msg.pose.pose.position.x,
                //             node_i.odom_msg.pose.pose.position.y,
                //             node_i.odom_msg.pose.pose.position.z);
                Vector3d p_cam_world = tf_uav*node_i.Cam_rel_Uav;
                pcl::PointXYZINormal pos_cam;
                pos_cam.x = p_cam_world(0);
                pos_cam.y = p_cam_world(1);
                pos_cam.z = p_cam_world(2);
                node_i.cam_rpy = Vector3d(0.0, 0.0, tf_uav.yaw());
                myTf<double> tf_cam(Util::YPR2Rot(node_i.cam_rpy), p_cam_world);            

                std::vector<int> k_idx;
                std::vector<float> k_distances;
                KtfreeInterests_.radiusSearch(pos_cam, node_i.visible_radius, k_idx, k_distances);        

                for (int j = 0; j < k_idx.size(); j++) //
                {
                    pcl::PointXYZINormal &kpoint = cloud_->points[k_idx[j]];
                    Vector3d InPoint_world(kpoint.x, kpoint.y, kpoint.z);
                    Vector3d InPoint_cam = tf_cam.inverse()*InPoint_world;
                    if (InPoint_cam(0)<=0.0) continue;
                    double horiz_angle = atan(InPoint_cam(1)/InPoint_cam(0))/M_PI*180.0;
                    double vert_angle = atan(InPoint_cam(2)/InPoint_cam(0))/M_PI*180.0;
                    if (horiz_angle>node_i.fov_h/2 || horiz_angle<-node_i.fov_h/2 ||
                        vert_angle >node_i.fov_v/2 || vert_angle <-node_i.fov_v/2)
                        continue;
                    if (CheckLOS(p_cam_world, InPoint_world, node_i.ray))
                    {
                        cloud_->points[k_idx[j]].intensity = 1.0;
                        // ROS_INFO("Setting points to high intensity!!");
                    }
                }
        
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
                // viewPub_list_[agent_id].publish(m); //DELETE last line first
                node_i.camera_pyramid_pub.publish(m);
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
                // viewPub_list_[agent_id].publish(m);
                node_i.camera_pyramid_pub.publish(m);
            }            
            sensor_msgs::PointCloud2 msg1;
            pcl::toROSMsg(*cloud_, msg1);
            msg1.header.frame_id = "world";
            cloud_pub_.publish(msg1);
        }        
    }

    void GazeboPPComPlugin::readPCloud(std::string filename)
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
        KtfreeInterests_.setInputCloud(cloud_);
    }

    bool GazeboPPComPlugin::CheckLOS(const Vector3d &pi, double bi, const Vector3d &pj, double bj, gazebo::physics::RayShapePtr &ray)
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
        double rtDist;           // Raytracing distance
        double ppDist = 0;       // Peer to peer distance
        string entity_name;      // Name of intersected object
        ignition::math::Vector3d start_point;
        ignition::math::Vector3d end_point;
        bool los = false;
        for(Vector3d &pa : Pi)
        {
            start_point = ignition::math::Vector3d(pa.x(), pa.y(), pa.z());

            for(Vector3d &pb : Pj)
            {
                end_point = ignition::math::Vector3d(pb.x(), pb.y(), pb.z());

                ray->SetPoints(start_point, end_point);
                ray->GetIntersection(rtDist, entity_name);

                ppDist = (pa - pb).norm();
                if (rtDist >= ppDist - 0.1)
                    return true;
            }
        }
        return los;
    }

    bool GazeboPPComPlugin::CheckLOS(const Eigen::Vector3d &pi, const Eigen::Vector3d &pj, gazebo::physics::RayShapePtr &ray)
    {

        // Ray tracing from the slf node to each of the nbr node
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
        if (rtDist >= ppDist - 0.1)
            return true;
        return los;
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboPPComPlugin);

} // namespace gazebo
