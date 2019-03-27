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
 *  @brief Implementing Plane and Box Point cloud generation
 *
 *  @section DESCRIPTION
 *
 *  This program is used to synthesize plane point cloud with a 
 *  resolution of 0.1 m. The random box generation is also implemented and then
 *  both the point clouds are merged.
 *  
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>

#define PI 3.14159265;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Generate {

private:

    PointCloud msg_plane;
    PointCloud msg_box;
    PointCloud merged;
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Timer plane_pub_timer;
    ros::Timer box_pub_timer;
    
    // Random number generator
    std::random_device rand_dev;
    std::mt19937  generator;
    std::uniform_real_distribution<> xy_rnd; 
    std::uniform_real_distribution<> yaw_rnd;
    std::uniform_real_distribution<> noise_rnd;

    // Generate Plane
    void generate_plane() {
        msg_plane.header.frame_id = "world";
        pcl_conversions::toPCL(ros::Time::now(), msg_plane.header.stamp);
        for (double i = -5.0; i <= 5.0; i+=0.1) {
            for (double j = -5.0; j <= 5.0; j+=0.1) {
                pcl::PointXYZ point;
                point.x = i;
                point.y = j;
                point.z = 0.0;
                msg_plane.push_back(point);
            }
        }
    }

    // Generate Box with random position and Orientation
    void generate_box(double x,double y, double yaw ) {
        // msg_box.header.frame_id = "map";
        x=0;
        y=0;
        yaw=3.14159265*0.4;
        ROS_INFO_STREAM("Angle given: " << yaw* 180 / PI;);
        pcl_conversions::toPCL(ros::Time::now(), msg_box.header.stamp);
        for (double i = x+0; i <=x+1 ; i+=0.1) {
            for (double j = y+0; j <=y+1; j+=0.1) {
                for (double k = 0; k <=1; k+=0.1) {
                    pcl::PointXYZ point;
                    point.x = x + (i-x)*cos(yaw) - (j-y)*sin(yaw);  
                    point.y = y + (i-x)*sin(yaw) + (j-y)*cos(yaw);
                    point.z = k;
                    msg_box.push_back(point);
                }
            }
        }

    }

    // Generating Noise and Adding to merged point cloud
    PointCloud make_noise() {

            PointCloud merged_noise;
            for (int i = 0; i < merged.points.size(); i++) {
                pcl::PointXYZ point;
                point.x = merged.points[i].x + noise_rnd(generator);
                point.y = merged.points[i].y + noise_rnd(generator);
                point.z = merged.points[i].z + noise_rnd(generator);
                merged_noise.push_back(point);
            }
            return merged_noise;

    }

    // Merging the plane and box point cloud.
    void merging_both(){

        merged = msg_plane + msg_box;
    }


    // Callback with frequency of 5 hz for noise generation. 
    void publisher_Callback(const ros::TimerEvent&) {
        // pub.publish(msg_plane);
        auto merged_noise = make_noise();
        merged_noise.header.frame_id = "world";
        pub.publish(merged_noise);
        // ROS_INFO_STREAM("Publishing");
    }

    // Callback for generating random box and merging them every second.
    void box_Callback(const ros::TimerEvent&) {
        double x = xy_rnd(generator);
        double y = xy_rnd(generator);
        double yaw = yaw_rnd(generator);
        generate_box(x,y,yaw);
        // pub.publish(msg_box);
        merging_both();
        msg_box.clear();

    }


public:
    Generate(): xy_rnd(-2.0, 2.0),yaw_rnd(0.0, 3.1415/2),noise_rnd(-0.0002, 
        0.0002),generator(rand_dev()) {
        
        pub = n.advertise<PointCloud>("/cloud",1000);
        plane_pub_timer = n.createTimer(ros::Duration(0.2),
         &Generate::publisher_Callback, this);
        box_pub_timer = n.createTimer(ros::Duration(1),
         &Generate::box_Callback, this);
        generate_plane();
    }

};


int main(int argc, char** argv) {

    ros::init (argc, argv, "plt");
    Generate gen;
    ros::spin();
}