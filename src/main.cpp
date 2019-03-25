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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Generate {

private:

    PointCloud msg_plane;
    PointCloud msg_box;
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Timer plane_pub_timer;
    ros::Timer box_pub_timer;
    
    std::random_device rand_dev;
    std::mt19937  generator;
    std::uniform_real_distribution<> xy_rnd; 
    std::uniform_real_distribution<> yaw_rnd;

    void generate_plane() {
        msg_plane.header.frame_id = "map";
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

    void generate_box(double x,double y, double yaw ) {
        msg_box.header.frame_id = "map";
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

    void publisher_Callback(const ros::TimerEvent&) {
        pub.publish(msg_plane);

    }

    void box_Callback(const ros::TimerEvent&) {
        double x = xy_rnd(generator);
        double y = xy_rnd(generator);
        double yaw = yaw_rnd(generator);
        generate_box(x,y,yaw);
        pub.publish(msg_box);

    }


public:

    Generate(): xy_rnd(-2.0, 2.0),yaw_rnd(0.0, 3.1415/2),generator(rand_dev()) {
        
        pub = n.advertise<PointCloud>("/cloud",1000);
        // plane_pub_timer = n.createTimer(ros::Duration(0.2), &Generate::publisher_Callback, this);
        box_pub_timer = n.createTimer(ros::Duration(1), &Generate::box_Callback, this);
        generate_plane();
    }

};


int main(int argc, char** argv) {

    ros::init (argc, argv, "plt");
    Generate gen;
    ros::spin();

}