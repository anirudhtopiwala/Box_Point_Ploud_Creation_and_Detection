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
 *	Here we detect the planes of the point cloud being published. The box is 
 *	then detected and the position and orientation is then estimated. 
 *	A service for saving the pointcloud is also added.    
 *	 
 */
#include <ros/ros.h>
#include <vector>
#include <utility> 
#include <math.h>  
#include <stdlib.h>  
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "box_detector/write_to_file.h"

#define PI 3.14159265;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class Segment {
private:
	ros::NodeHandle n;
	ros::Subscriber sub_ptcloud;
	PointCloud::Ptr curr;
	ros::Publisher pub;
    ros::Timer plane_pub_timer;
	ros::ServiceServer serv;

	// Callback for visualizing the detected box in Rviz
	void Callback(const PointCloud::ConstPtr& cloud) {
  		*curr = *cloud; // save current cloud 
  		// remove_noise();
  		auto ans = detect_plane();
  		locate_box(ans);
  		pub.publish(ans);
  	}

  // 	void remove_noise(){
  // 		PointCloud::Ptr temp;
  // 		for(int i=0;i<5;i++){
  // 			for (int i = 0; i < curr->points.size(); i++) {
		// 		temp->points[i].x += curr->points[i].x/5;
		// 		temp->points[i].y += curr->points[i].y/5;
		// 		temp->points[i].z += curr->points[i].z/5;
  // 		}
  // 		ros::spinOnce();
  // 	}

  // 	*curr = *temp;
  // }

  	// Method to implement RANSAC implementation of PCL and detect the box.
  	PointCloud detect_plane(){

		// Initialize new pointers to use in processing
		// to store points of extracted plane
		pcl::PointIndices::Ptr indices_internal (new pcl::PointIndices); 
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients (true);
		// Search for a plane perpendicular to some axis (specified below).
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(400); 
		// Set the distance to the plane for a point to be an inlier.
		seg.setDistanceThreshold(0.01);

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

		// Extarcting the detected box.
		extract.setInputCloud(new_cloud);
		extract.setIndices(indices_internal);
		extract.setNegative(true);
		extract.filter(*new_cloud);
		return *new_cloud;
	}

	// Method to find the location of detected box.
	void locate_box(PointCloud box_cloud){
		double x = 0;
		double y = 0;
		double z = 0;
		double avg_layer_x = 0;
		double avg_layer_y = 0;
		std::vector<std::pair<double,double>> layer;
		std::vector<std::pair<double,double>> corner;
		for (int i = 0; i < box_cloud.points.size(); i++) {
			x += box_cloud.points[i].x;
			y += box_cloud.points[i].y;
			z += box_cloud.points[i].z;

			// Extracting points of any One plane
			if (box_cloud.points[i].z-1<0.01){
				layer.push_back(std::make_pair(box_cloud.points[i].x,box_cloud.points[i].y));
				avg_layer_x += box_cloud.points[i].x;
				avg_layer_y += box_cloud.points[i].y;
				// ROS_INFO_STREAM("layerx"<< avg_layer_x);
				// ROS_INFO_STREAM("layery"<< avg_layer_y);
			}
		}
		double x_avg = x/box_cloud.points.size();
		double y_avg = y/box_cloud.points.size();
		double z_avg = z/box_cloud.points.size();
		ROS_INFO_STREAM("New Centroid of cube is:");
		ROS_INFO_STREAM("X: "<<x_avg);
		ROS_INFO_STREAM("Y: "<<y_avg);
		ROS_INFO_STREAM("Z: "<<z_avg);	

		// getting Orientation

		// Getting Centroid of Square (layer of cube)
		avg_layer_x = avg_layer_x/layer.size();
		avg_layer_y = avg_layer_y/layer.size();
		// ROS_INFO_STREAM("layerx"<< avg_layer_x);
		// ROS_INFO_STREAM("layery"<< avg_layer_y);

		// Getting Orientation of Cube
		double xmax=0.0;
		double ymax=0.0;
		for (int i=0;i<layer.size();i++){
			auto x_sq = pow((avg_layer_x-layer[i].first),2);
			auto y_sq = pow((avg_layer_y-layer[i].second),2);
			double dist = sqrt(x_sq+y_sq);

			// ROS_INFO_STREAM("Dist"<<dist);
			if (dist>0.6 && (layer[i].first> avg_layer_x) && (layer[i].second> avg_layer_y)){
				
				if (layer[i].first> xmax){
					xmax = layer[i].first;
				}
				if (layer[i].second> ymax){
					ymax = layer[i].second;
				}
					
			}
		}
		auto ang = atan2( (ymax - avg_layer_y),(xmax - avg_layer_x)) * 180 / PI;
		ang= ang-45 ;
		// ROS_INFO_STREAM("X: "<< xmax );
		// ROS_INFO_STREAM("y: "<< ymax );
		ROS_INFO_STREAM("Orientation in Degrees: "<< ang );
				
	}





public:
	Segment(): curr(new PointCloud) {
			sub_ptcloud = n.subscribe("/cloud", 1000, 
				&Segment::Callback, this);
			pub = n.advertise<PointCloud>("/cloud_plane",1000);
			serv = n.advertiseService("/write_to_file", 
				&Segment::writeToFile, this);
	}

	bool writeToFile(box_detector::write_to_file::Request& req,
			 box_detector::write_to_file::Response& res ) {
			auto ans = detect_plane();
			ROS_WARN_STREAM("Saving .pcd File and affecting local storage");
			pcl::io::savePCDFileASCII ("segmented_box.pcd", ans);
			// res= true;
			return true;
	}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "segment");
  Segment seg;
  ros::spin();
}