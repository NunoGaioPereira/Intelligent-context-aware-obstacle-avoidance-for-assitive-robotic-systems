// This node receives a pointcloud and segments it, using the PCL library
// The output is an array containing the position and dimensions of each segmented object

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"

#include "segmentation/seg.h"

// Declaring the publsihers and subcribers
ros::Publisher pub;
ros::Publisher pub2;
ros::Subscriber sub;
ros::ServiceServer service;

// Declaring pointcloud opbject
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

// Service callback; receives pointcloud, returns segmented objects dimensions and positions
bool service_callback (segmentation::seg::Request  &req, segmentation::seg::Response &res ){

  ROS_INFO("Callback");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(req.pointcloud, *cloud_pcl); // Convert from ROS message to PCL pointcloud datatype

  std::cout << "PointCloud: " << cloud_pcl->points.size () << " data points." << std::endl;

  // Use this to save the pointcloud to a pcl file
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "main.pcd";
  //writer.write<pcl::PointXYZ> (ss.str (), *cloud_pcl, false);

  // Passthrough filter to remove point beyond a distance treshold
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_pcl);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0f, 0.8f);
  pass.filter (*cloud_passthrough_filtered);
  // writer.write<pcl::PointXYZ> ("filtered.pcd", *cloud_passthrough_filtered, false);

  // Transformations to rotate the point cloud
  float roty = M_PI/2.0;
  float rotz = -M_PI/2.0;
  Eigen::Affine3f transform_z = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
  transform_z.rotate (Eigen::AngleAxisf (rotz, Eigen::Vector3f::UnitZ()));
  transform_y.rotate (Eigen::AngleAxisf (roty, Eigen::Vector3f::UnitY()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed1_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed2_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*cloud_passthrough_filtered, *transformed1_cloud, transform_z);
  pcl::transformPointCloud (*transformed1_cloud, *transformed2_cloud, transform_y);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (transformed2_cloud);
  vg.setLeafSize (0.015f, 0.015f, 0.015f); // originally all 0.01f
  vg.filter (*cloud_filtered);
  std::cout << "Callback: PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // writer.write<pcl::PointXYZ> ("rotXZ.pcd", *cloud_filtered, false);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);


  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Callback: Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "Callback: PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers (i.e. only the objects remain)
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Calculate min and max of the removed table
  pcl::PointXYZ minTable, maxTable;
  pcl::getMinMax3D (*cloud_filtered, minTable, maxTable);
  std::cout << "Min table: " << minTable.z << std::endl;


  // writer.write<pcl::PointXYZ> ("notable.pcd", *cloud_filtered, false);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  // Euclidean Cluster extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // original = 2cm (0.02)
  ec.setMinClusterSize (20); //original = 100
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // Cloud to publish all clusters
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

   // Setup Radius outlier filter: http://pointclouds.org/documentation/tutorials/remove_outliers.php
   pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
   // Build the filter
   outrem.setInputCloud(cloud_cluster);
   outrem.setRadiusSearch(0.05);
   outrem.setMinNeighborsInRadius (25); // 25 - 30 looks good
   // Apply filter
   outrem.filter (*cluster_filtered);


    std::cout << "PointCloud representing the Cluster: " << cluster_filtered->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "CB_cluster_" << j << ".pcd";
    // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);

    //Find max and min xyz of each cluster
      pcl::PointXYZ minPt, maxPt;
      pcl::getMinMax3D (*cluster_filtered, minPt, maxPt);
      std::cout << "Cluster " << j << ":" << std::endl;


      // Calculate dimensions of each cluster
      float xDim = maxPt.x - minPt.x;
      float yDim = maxPt.y - minPt.y;
      float zDim = maxPt.z - minPt.z;
      // float zDim = maxPt.z - minTable.z;

      std::cout << "Dim x: " << xDim << std::endl;
      std::cout << "Dim y: " << yDim << std::endl;
      std::cout << "Dim z: " << zDim << std::endl;

      // // Calculate center point of each cluster
      float xCenter = maxPt.x;
      // float xCenter = (maxPt.x + minPt.x)/2;
      float yCenter = (maxPt.y + minPt.y)/2;
      float zCenter = (maxPt.z + minPt.z)/2;

      std::cout << "Center x: " << xCenter << std::endl;
      std::cout << "Center y: " << yCenter << std::endl;
      std::cout << "Center z: " << zCenter << std::endl;

      // Eliminate clusters that are outside the table
      if(xCenter < 2.0) {
        res.objects.data.push_back(xDim);
        res.objects.data.push_back(yDim);
        res.objects.data.push_back(zDim);
        res.objects.data.push_back(xCenter);
        res.objects.data.push_back(yCenter);
        res.objects.data.push_back(zCenter);
      }

      // Merge each cluster to one cloud
      *clustered_cloud += *cluster_filtered;
      j++;
  }

  // writer.write<pcl::PointXYZ> ("clusters.pcd", *clustered_cloud, false);
  // Publish the clusters to a ROS topic
  sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*clustered_cloud , *clusters);
  clusters->header.frame_id = "/camera_link";
  clusters->header.stamp=ros::Time::now();
  pub2.publish (*clusters);

  return true;
}


int main (int argc, char** argv)
{
  // Initialize node
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle nh;

  // Pointcloud publisher
  pub2= nh.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 100);

  // Service advertiser
  service = nh.advertiseService("segmentation_service", service_callback);

  ROS_INFO("Ready to segment pointcloud.");
  ros::spin();

  return 0;
}
