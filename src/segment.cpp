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
 *  @file    segment.cpp
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

// Include for the unfinished srv file: TODO - implement srv for writing to .pcd
// #include <box_detector/Writetofile.h>

// For use w/ finding planes
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class Segment {
private:
	ros::NodeHandle n;
	ros::Subscriber sub_ptcloud;
	PointCloud::Ptr curr;
	ros::Publisher pub;
    ros::Timer plane_pub_timer;

	void Callback(const PointCloud::ConstPtr& cloud) {
  		*curr = *cloud; // save current cloud 
  		auto ans = detect_plane();
  		pub.publish(ans);
  	}



  	PointCloud::Ptr detect_plane(){

		// Initialize new pointers to use in processing
		pcl::PointIndices::Ptr indices_internal (new pcl::PointIndices); // to store points of extracted plane
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients (true);
		// Search for a plane perpendicular to some axis (specified below).
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(400); 
		// Set the distance to the plane for a point to be an inlier.
		seg.setDistanceThreshold(0.01);
		//		

		PointCloud::Ptr new_cloud (new PointCloud);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		*new_cloud = *curr;

		seg.setInputCloud (new_cloud);
		seg.segment (*indices_internal, *coefficients);

		for (int i = 0; i < indices_internal->indices.size(); i++) {
				new_cloud->points[indices_internal->indices[i]].x;
				new_cloud->points[indices_internal->indices[i]].y;
				new_cloud->points[indices_internal->indices[i]].z;
			}

		extract.setInputCloud(new_cloud);
		extract.setIndices(indices_internal);
		extract.setNegative(true);
		extract.filter(*new_cloud);	

		ROS_INFO_STREAM("Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3]);

		return new_cloud;
		

			// TODO: Estimate, based on centroids/planes found, the pose.
	}


public:
	Segment(): curr(new PointCloud) {
			sub_ptcloud = n.subscribe("/cloud", 1000, &Segment::Callback, this);
			pub = n.advertise<PointCloud>("/cloud_plane",1000);
			// plane_pub_timer = n.createTimer(ros::Duration(0.2), &Segment::Callback, this);
			// write_serv = n.advertiseService("/write_to_file", &DetectBox::writeToFile, this);
	}


};

int main(int argc, char** argv) {
  ros::init(argc, argv, "segment");
  Segment seg;
  ros::spin();
  // return 0;
}