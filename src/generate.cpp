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
 *  @file    generate.cpp
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
#include "generate.hpp"


/**
 * Initializing Constructor for an object of walker class
 */
Generate::Generate() {
    // Initialing the range variable due to cppcheck warning
    ros::Publisher pub = n.advertise<PointCloud>("/cloud",1000);
    plane_pub_timer = n.createTimer(ros::Duration(0.2), &Generate::publisher_Callback, this);
      // RViz Visualization
    Generate::generate_plane();
}

/**
 * Returns the value of variable range
 */
void Generate::generate_plane() {

        for (double i = -5.0; i<=5;i+=0.1){
            for (double j = -5.0; j<=5;j+=0.1){
                pcl::PointXYZ point;
                point.x = i;
                point.y = j;
                point.z = 0.0;
                msg_plane->points.push_back(point);
            }
        }

    }

void Generate::publisher_Callback(const ros::TimerEvent&) {
    msg_plane->header.frame_id = "base_link";
    pub.publish(msg_plane);
}

/**
 * Calling Destructor for the object of walker class
 */
Generate::~Generate() {}