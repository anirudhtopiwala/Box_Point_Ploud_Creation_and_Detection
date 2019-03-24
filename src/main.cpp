/**
 * BSD 3-Clause LICENSE
 *
 * Copyright (c) 2019, Anirudh Topiwala
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  @file    main.cpp
 *  @author  Anirudh Topiwala
 *  @copyright BSD License
 *
 *  @brief Implementing Box_Detector
 *
 *  @section DESCRIPTION
 *
 *  This program is used to synthesize box pointcloud and then detect is using
 *  PCL and ROS                               
 *  
 */

#include "ros/ros.h"
#include <stdio.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char **argv) {
  // initializing the node name as walker
  ros::init(argc, argv, "plt");

  ros::start();

    // get node handle
    ros::NodeHandle n;
    ros::Rate loopRate(5);
    std::string topicName = "cloud";

    ros::Publisher pub = n.advertise<PointCloud>(topicName.c_str(),5);
    PointCloud msg;

    // Establishing TF Relationship
    msg.header.frame_id = "map";
    ROS_INFO("Publishing point cloud on topic \"%s\" once every 0.2 second.", topicName.c_str());
    
    // TO create Random Numbers
    const int range_from  = -5; const int range_to = 5;
    std::random_device rand_dev; std::mt19937   generator(rand_dev());
    std::uniform_real_distribution<double>  distr(range_from, range_to);
    std::uniform_real_distribution<double>  distro(0, 1);

    // Starting the ROS Loop
    while (ros::ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), msg.header.stamp);

        // Creating Plane Point Cloud
        for (int v=0; v<100; ++v)
        {
            pcl::PointXYZ newPoint;
            newPoint.x = distr(generator) ;
            newPoint.y = distr(generator) ;
            newPoint.z = 0;
            msg.points.push_back(newPoint);
        }

        // publish point cloud
        pub.publish(msg);
        ros::spinOnce ();
        // Clear Previous Points
        msg.clear();
        // pause for loop delay
        loopRate.sleep();
    }

    return 1;
}