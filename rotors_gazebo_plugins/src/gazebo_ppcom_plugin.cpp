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

        // Get the ppcom topic where data is published to
        getSdfParam<string>(_sdf, "ppcomTopic", ppcom_topic_, "ppcom");

        // Report on params obtained from sdf
        printf(KGRN "PPCom Id %s is set. Linkname %s. Config %s!\n" RESET,
                     ppcom_id_.c_str(), self_link_name_.c_str(), ppcom_config_.c_str());

        // Open the config file and read the links
        std::ifstream ppcom_config_file(ppcom_config_.c_str());
        
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

            // Decode the line
            vector<string> parts = Util::split(line, ",");
            PPComNode newNode;
            newNode.name = parts[0];
            newNode.role = parts[1];
            newNode.offset = stod(parts[2]);
            ppcom_nodes_.push_back(newNode);
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

            // Create the storage of nodes to each object
            node.odom_msg = nav_msgs::Odometry();
            node.odom_msg_received = false;

            // Create rayshape object
            node.ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
                        (physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));
        }

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

        // Update the ray casting every 0.1s
        if (dt > 0.1)
        {
            last_time_ = current_time;

            PPComNode &ppcom_slf = ppcom_nodes_[ppcom_slf_idx_];

            // Create the basic odom vector
            Vector3d ps(ppcom_slf.odom_msg.pose.pose.position.x,
                        ppcom_slf.odom_msg.pose.pose.position.y,
                        ppcom_slf.odom_msg.pose.pose.position.z);

            vector<Vector3d> pA;
            pA.push_back(ps + Vector3d(ppcom_slf.offset, 0, 0));
            pA.push_back(ps - Vector3d(ppcom_slf.offset, 0, 0));
            pA.push_back(ps + Vector3d(0, ppcom_slf.offset, 0));
            pA.push_back(ps - Vector3d(0, ppcom_slf.offset, 0));
            pA.push_back(ps + Vector3d(0, 0, ppcom_slf.offset));
            pA.push_back(ps - Vector3d(0, 0, ppcom_slf.offset));
            // Make sure the virtual antenna does not go below the ground
            pA.back().z() = max(0.1, pA.back().z());

            map<string, bool> los_check;
            for(PPComNode &ppcom_nbr : ppcom_nodes_)
            {
                // Defaulting the line of sight to this nbr as false
                los_check[ppcom_nbr.name] = false;
                
                // If no odom from node has arrived, skip
                if(!ppcom_nbr.odom_msg_received)
                    continue;

                // If this nbr node is the self, skip
                if (ppcom_slf.name == ppcom_nbr.name)
                    continue;

                // Find the position of the neighbour
                Vector3d pe(ppcom_nbr.odom_msg.pose.pose.position.x,
                            ppcom_nbr.odom_msg.pose.pose.position.y,
                            ppcom_nbr.odom_msg.pose.pose.position.z);

                vector<Vector3d> pB;
                pB.push_back(pe + Vector3d(ppcom_nbr.offset, 0, 0));
                pB.push_back(pe - Vector3d(ppcom_nbr.offset, 0, 0));
                pB.push_back(pe + Vector3d(0, ppcom_nbr.offset, 0));
                pB.push_back(pe - Vector3d(0, ppcom_nbr.offset, 0));
                pB.push_back(pe + Vector3d(0, 0, ppcom_nbr.offset));
                pB.push_back(pe - Vector3d(0, 0, ppcom_nbr.offset));
                // Make sure the virtual antenna does not go below the ground
                pB.back().z() = max(0.1, pB.back().z());

                // Ray tracing from the slf node to each of the nbr node
                double rtDist;           // Raytracing distance
                double ppDist = 0;       // Peer to peer distance
                string entity_name;      // Name of intersected object
                ignition::math::Vector3d start_point;
                ignition::math::Vector3d end_point;
                for(Vector3d &pa : pA)
                {
                    start_point = ignition::math::Vector3d(pa.x(), pa.y(), pa.z());

                    for(Vector3d &pb : pB)
                    {
                        end_point = ignition::math::Vector3d(pb.x(), pb.y(), pb.z());

                        ppcom_nbr.ray->SetPoints(start_point, end_point);
                        ppcom_nbr.ray->GetIntersection(rtDist, entity_name);

                        ppDist = (pa - pb).norm();
                        if (rtDist >= ppDist - 0.1)
                        {
                            los_check[ppcom_nbr.name] = true;
                            break;
                        }
                    }

                    if (los_check[ppcom_nbr.name])
                        break;
                }

                // Print out the result for the first node
                // if(ppcom_id_ == "firefly1")
                // {
                //     printf("%s"
                //             "Time: %9.3f.\n"
                //             "Start %s. XYZ: %7.3f, %7.3f, %7.3f. ppDist: %7.3f.\n"
                //             "End   %s. XYZ: %7.3f, %7.3f, %7.3f. rtDist: %7.3f. Entity: %s\n\n" RESET
                //             ,
                //             line_of_sight ? KBLU "LOS " : KYEL "NLOS",
                //             t, ppcom_id_.c_str(), node_id.c_str(),
                //             start_point.X(), start_point.Y(), start_point.Z(),
                //             end_point.X(), end_point.Y(), end_point.Z(),
                //             ppDist, rtDist, entity_name.c_str());
                // }
            }

            // cout << "Thread: " << this_thread::get_id() << ". ";
            // printf("Node %02d-%s. LOS check: ", ppcom_slf_idx_ + 1, ppcom_id_.c_str());
            // for(int node_idx = 0; node_idx < Nnodes_; node_idx++)
            //     cout << los_check[node_idx] << " ";
            // cout << endl;

            typedef visualization_msgs::Marker RosVizMarker;
            typedef std_msgs::ColorRGBA RosVizColor;
            typedef ros::Publisher RosPub;

            struct VizAid
            {
                bool           inited = false;
                RosVizColor    color  = RosVizColor();
                RosVizMarker   marker = RosVizMarker();
                ros::Publisher rosPub = RosPub();
            };

            // Create the los marker
            static vector<VizAid> vizAid(Nnodes_);
            VizAid &vizAidSelf = vizAid[ppcom_slf_idx_];
            
            // Initialize the loop marker
            if (!vizAidSelf.inited)
            {
                vizAidSelf.rosPub = ros_node_handle_->advertise<RosVizMarker>("/" + ppcom_id_ + "/los_marker", 1);

                // Set up the loop marker
                vizAidSelf.marker.header.frame_id = "world";
                vizAidSelf.marker.ns       = "loop_marker";
                vizAidSelf.marker.type     = visualization_msgs::Marker::LINE_LIST;
                vizAidSelf.marker.action   = visualization_msgs::Marker::ADD;
                vizAidSelf.marker.pose.orientation.w = 1.0;
                vizAidSelf.marker.lifetime = ros::Duration(0);
                vizAidSelf.marker.id       = 0;

                vizAidSelf.marker.scale.x = 0.3;
                vizAidSelf.marker.scale.y = 0.3;
                vizAidSelf.marker.scale.z = 0.3;

                vizAidSelf.marker.color.r = 0.0;
                vizAidSelf.marker.color.g = 1.0;
                vizAidSelf.marker.color.b = 1.0;
                vizAidSelf.marker.color.a = 1.0;
    
                vizAidSelf.color.r = 0.0;
                vizAidSelf.color.g = 1.0;
                vizAidSelf.color.b = 1.0;
                vizAidSelf.color.a = 1.0;

                vizAidSelf.inited = true;
            }

            vizAidSelf.marker.points.clear();
            vizAidSelf.marker.colors.clear();

            for(PPComNode &ppcom_nbr : ppcom_nodes_)
            {
                if (ppcom_nbr.name == ppcom_slf.name)
                    continue;

                // string node_id = ppcom_nodes_[node_idx].name;

                if(los_check[ppcom_nbr.name])
                {
                    geometry_msgs::Point point;

                    vizAidSelf.marker.points.push_back(ppcom_slf.odom_msg.pose.pose.position);
                    vizAidSelf.marker.colors.push_back(vizAidSelf.color);

                    vizAidSelf.marker.points.push_back(ppcom_nbr.odom_msg.pose.pose.position);
                    vizAidSelf.marker.colors.push_back(vizAidSelf.color);
                }
            }

            vizAidSelf.rosPub.publish(vizAidSelf.marker);
        }

    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboPPComPlugin);

} // namespace gazebo
