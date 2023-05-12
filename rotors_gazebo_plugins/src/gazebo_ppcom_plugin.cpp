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

// Physics
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>

// 3RD PARTY
#include "mav_msgs/default_topics.h"

// // USER HEADERS
// #include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo
{
    GazeboPPComPlugin::GazeboPPComPlugin()
        : ModelPlugin(), gz_node_handle_(0) {}

    GazeboPPComPlugin::~GazeboPPComPlugin() {}

    void GazeboPPComPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (kPrintOnPluginLoad)
            gzdbg << __FUNCTION__ << "() called." << std::endl;

        gzdbg << "_model = " << _model->GetName() << std::endl;

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
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a robotNamespace.\n";

        if (_sdf->HasElement("linkName"))
            self_link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a linkName.\n";

        if (_sdf->HasElement("ppcomId"))
            ppcom_id_ = _sdf->GetElement("ppcomId")->Get<std::string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify a ppcomId.\n";

        if (_sdf->HasElement("ppcomConfig"))
            ppcom_config_ = _sdf->GetElement("ppcomConfig")->Get<std::string>();
        else
            gzerr << "[gazebo_ppcom_plugin] Please specify ppcomConfig.\n";

        // Get the ppcom topic where data is published to
        getSdfParam<std::string>(_sdf, "ppcomTopic", ppcom_topic_, "ppcom");

        // Report on params obtained from sdf
        printf(KGRN "PPCom Id %s is set. Linkname %s. Config %s!\n" RESET,
                     ppcom_id_.c_str(),
                     self_link_name_.c_str(),
                     ppcom_config_.c_str());

        // Open the config file and read the links
        std::ifstream ppcom_config_file(ppcom_config_.c_str());
        
        // Read the declared nodes
        ppcom_nodes_.clear();
        std::string line;
        while (std::getline(ppcom_config_file, line))
        {
            // Process the line here
            std::cout << "Reading " << line << std::endl;
            vector<string> parts = Util::split(line, ",");
            PPComNode newNode;
            newNode.name = parts[0];
            newNode.offset = std::stod(parts[1]);
            ppcom_nodes_.push_back(newNode);
        }

        // Assert that ppcom_id_ is found in the network
        bool ppcom_id_in_network = false;
        for (int idx = 0; idx < ppcom_nodes_.size(); idx++)
            if (ppcom_id_ == ppcom_nodes_[idx].name)
            {
                ppcom_id_in_network = true;
                ppcom_idx_ = idx;
                break;
            }

        assert(ppcom_id_in_network);

        //==============================================//
        //========== CREATE THE TRANSPORT STRUCTURES ===//
        //==============================================//

        // Create a gazebo node handle and initialize with the namespace
        gz_node_handle_ = transport::NodePtr(new transport::Node());
        gz_node_handle_->Init();

        // Create a ros node
        ros_node_handle_ = new ros::NodeHandle("/firefly" + ppcom_id_ + "rosnode");

        // Subscribe to the odom topics
        int Nnodes = ppcom_nodes_.size();
        for (int i = 0; i < Nnodes; i++)
        {
            // Create the subscriber to each nodes
            odom_sub[ppcom_nodes_[i].name]
                = ros_node_handle_->subscribe<nav_msgs::Odometry>("/" + ppcom_nodes_[i].name + "/ground_truth/odometry", 1,
                                                                  boost::bind(&GazeboPPComPlugin::OdomCallback, this, _1, i));

            // Create the storage of nodes to each object
            odom_msgs[ppcom_nodes_[i].name] = nav_msgs::Odometry();

            // Create rayshape object
            rays_[ppcom_nodes_[i].name]
                = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
                    (physics_->CreateShape("ray", gazebo::physics::CollisionPtr()));
        }

        // Listen to the update event. This event is broadcast every simulation iteration.
        last_time_ = world_->SimTime();
        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPPComPlugin::OnUpdate, this, _1));
    }

    void GazeboPPComPlugin::OdomCallback(const nav_msgs::OdometryConstPtr &msg, int node_idx)
    {
        odom_msgs[ppcom_nodes_[node_idx].name] = *msg;
        // nav_msgs::Odometry &odom = odom_msgs[ppcom_nodes_[node_idx].name];

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
            gzdbg << __FUNCTION__ << "() called." << std::endl;

        // if (!pubs_and_subs_created_)
        // {
        //     CreatePubsAndSubs();
        //     pubs_and_subs_created_ = true;
        // }

        common::Time current_time = world_->SimTime();
        double t = current_time.Double();
        double dt = (current_time - last_time_).Double();

        // Update the ray casting every 0.1s
        if (dt > 0.1)
        {
            last_time_ = current_time;

            Vector3d ps(odom_msgs[ppcom_id_].pose.pose.position.x,
                        odom_msgs[ppcom_id_].pose.pose.position.y,
                        odom_msgs[ppcom_id_].pose.pose.position.z);

            vector<Vector3d> pA;
            pA.push_back(ps + Vector3d(0, 0, ppcom_nodes_[ppcom_idx_].offset));
            pA.push_back(ps - Vector3d(0, 0, ppcom_nodes_[ppcom_idx_].offset));
            pA.push_back(ps + Vector3d(0, ppcom_nodes_[ppcom_idx_].offset, 0));
            pA.push_back(ps - Vector3d(0, ppcom_nodes_[ppcom_idx_].offset, 0));
            pA.push_back(ps + Vector3d(0, 0, ppcom_nodes_[ppcom_idx_].offset));
            pA.push_back(ps - Vector3d(0, 0, ppcom_nodes_[ppcom_idx_].offset));
            
            for(int node_idx = 0; node_idx < ppcom_nodes_.size(); node_idx++)
            {
                std::string node_id = ppcom_nodes_[node_idx].name;

                // If node is the same with id, skip
                if (node_id == ppcom_id_)
                    continue;

                // Find the position of the neighbour
                Vector3d pe(odom_msgs[node_id].pose.pose.position.x,
                            odom_msgs[node_id].pose.pose.position.y,
                            odom_msgs[node_id].pose.pose.position.z);

                vector<Vector3d> pB;
                pB.push_back(pe + Vector3d(0, 0, ppcom_nodes_[node_idx].offset));
                pB.push_back(pe - Vector3d(0, 0, ppcom_nodes_[node_idx].offset));
                pB.push_back(pe + Vector3d(0, ppcom_nodes_[node_idx].offset, 0));
                pB.push_back(pe - Vector3d(0, ppcom_nodes_[node_idx].offset, 0));
                pB.push_back(pe + Vector3d(0, 0, ppcom_nodes_[node_idx].offset));
                pB.push_back(pe - Vector3d(0, 0, ppcom_nodes_[node_idx].offset));
                             
                // Ray tracing every pair to check the line of sight
                double rtDist;           // Raytracing distance
                double ppDist = 0;       // Peer to peer distance
                std::string entity_name; // Name of intersected object
                bool line_of_sight = false;
                ignition::math::Vector3d start_point;
                ignition::math::Vector3d end_point;

                for(Vector3d &pa : pA)
                {
                    start_point = ignition::math::Vector3d(pa.x(), pa.y(), pa.z());

                    for(Vector3d &pb : pB)
                    {
                        end_point = ignition::math::Vector3d(pb.x(), pb.y(), pb.z());

                        rays_[node_id]->SetPoints(start_point, end_point);
                        rays_[node_id]->GetIntersection(rtDist, entity_name);

                        ppDist = (pa - pb).norm();
                        if (rtDist >= ppDist - 0.1)
                        {
                            line_of_sight = true;
                            break;
                        }
                    }

                    if (line_of_sight)
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
        }
    }

    // void GazeboPPComPlugin::CreatePubsAndSubs()
    // {
    //     // Create temporary "ConnectGazeboToRosTopic" publisher and message
    //     gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
    //         gz_node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
    //             "~/" + kConnectGazeboToRosSubtopic, 1);

    //     // ============================================ //
    //     // =============== IMU MSG SETUP ============== //
    //     // ============================================ //

    //     ppcom_pub_ = gz_node_handle_->Advertise<gz_sensor_msgs::Imu>(
    //         "~/" + namespace_ + "/" + ppcom_topic_, 1);

    //     gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
    //     // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
    //     connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
    //                                                      ppcom_topic_);
    //     connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + ppcom_topic_);
    //     connect_gazebo_to_ros_topic_msg.set_msgtype(
    //         gz_std_msgs::ConnectGazeboToRosTopic::IMU);
    //     connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
    //                                              true);
    // }

    GZ_REGISTER_MODEL_PLUGIN(GazeboPPComPlugin);

} // namespace gazebo
