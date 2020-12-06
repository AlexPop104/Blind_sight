/*
 * voxel_filter.cpp
 *
 *  Created on: 06.09.2013
 *      Author: goa
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_tutorial/zoom_in_nodeConfig.h>
#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/transforms.h>




class ZoominNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  ZoominNode()
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/zoom_in_scalled_pointcloud", 1);
    sub_ = nh_.subscribe ("/pf_out", 1,  &ZoominNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&ZoominNode::dynReconfCallback, this, _1, _2));
  }

  ~ZoominNode() {}

  void
  dynReconfCallback(pcl_tutorial::zoom_in_nodeConfig &config, uint32_t level)
  {
       dimension_scale=config.dimension_scale;
      x_translation=config.x_translation;
      y_translation=config.y_translation;
      z_translation=config.z_translation;
  }

  void
  cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
      pcl::PointCloud<pcl::PointXYZ> cloud_Test;
      pcl::fromROSMsg(*cloud_msg, cloud_Test);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPTR = cloud_Test;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      
      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
      transform (0,0) = transform (0,0) * dimension_scale;
      transform (1,1) = transform (1,1) * dimension_scale;
      transform (2,2) = transform (2,2) * dimension_scale;
      transform (0,3) =x_translation;
      transform (1,3) =y_translation;
      transform (2,3) =z_translation;
      pcl::transformPointCloud (*cloudPTR, *transformed_cloud, transform);


      sensor_msgs::PointCloud2 tempROSMsg;

      pcl::toROSMsg(*transformed_cloud, tempROSMsg);

      tempROSMsg.header.frame_id = "camera_depth_optical_frame";

      pub_.publish(tempROSMsg);

  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<pcl_tutorial::zoom_in_nodeConfig> config_server_;

 double dimension_scale;
  double x_translation;
  double y_translation;
  double z_translation;

  
  

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "zoom_in_node");

  ZoominNode vf;

  ros::spin();
}

