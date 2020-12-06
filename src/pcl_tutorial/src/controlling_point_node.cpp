/*
 * voxel_filter.cpp
 *
 *  Created on: 06.09.2013
 *      Author: goa
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_tutorial/controlling_point_nodeConfig.h>
#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/transforms.h>

class ControllingPointNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  ControllingPointNode()
  {
    pub1_ = nh_.advertise<sensor_msgs::PointCloud2>("/Two_points", 1);
    pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("/One_point", 1);
    pub3_ = nh_.advertise<sensor_msgs::PointCloud2>("/Proximity_points", 1);
    sub_ = nh_.subscribe("/pf_zoom_out", 1, &ControllingPointNode::cloudCallback, this);

    config_server_.setCallback(boost::bind(&ControllingPointNode::dynReconfCallback, this, _1, _2));
  }

  ~ControllingPointNode() {}

  void
  dynReconfCallback(pcl_tutorial::controlling_point_nodeConfig &config, uint32_t level)
  {
    dimension_scale = config.dimension_scale;
    x_translation = config.x_translation;
    y_translation = config.y_translation;
    z_translation = config.z_translation;

    point_movement_speed = config.point_movement_speed;
    point_x_translation = config.point_x_translation;
    point_y_translation = config.point_y_translation;
    point_z_translation = config.point_z_translation;

    proximity_threshold = config.proximity_threshold;
  }

  void
  Transform_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_puncte,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud,
                       double dimension_scale,
                       double x_translation,
                       double y_translation,
                       double z_translation)
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = transform(0, 0) * dimension_scale;
    transform(1, 1) = transform(1, 1) * dimension_scale;
    transform(2, 2) = transform(2, 2) * dimension_scale;
    transform(0, 3) = x_translation;
    transform(1, 3) = y_translation;
    transform(2, 3) = z_translation;
    pcl::transformPointCloud(*cloud_puncte, *transformed_cloud, transform);
  }



  void check_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr Controller_point,
                      pcl::PointCloud<pcl::PointXYZ> &final_pointcloud,
                      double proximity_threshold)
  {
    double dist_x;
    double dist_y;
    double dist_z;

    double dist_finala;

    for (auto &punct : *cloud)
    {
      dist_x = punct.x - Controller_point->points[0].x;
      dist_y = punct.y - Controller_point->points[0].y;
      dist_z = punct.z - Controller_point->points[0].z;

      dist_finala = sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);


      if (dist_finala<proximity_threshold)
      {

         int N=final_pointcloud.width;

          final_pointcloud.width=final_pointcloud.width+1;
          final_pointcloud.is_dense = false;
           final_pointcloud.resize(final_pointcloud.width * final_pointcloud.height);

          final_pointcloud.points[N].x=punct.x;
          final_pointcloud.points[N].y=punct.y;
         final_pointcloud.points[N].z=punct.z;
        
        
        
      }

    }


  }

  void
  cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_Test;
    pcl::fromROSMsg(*cloud_msg, cloud_Test);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPTR = cloud_Test;

    ////////////////// POINTCLOUD OBIECT DE REFERINTA SI OBIECT IN MISCARE

    pcl::PointCloud<pcl::PointXYZ> cloud_puncte;

    cloud_puncte.width = 2;
    cloud_puncte.height = 1;
    cloud_puncte.is_dense = false;
    cloud_puncte.resize(cloud_puncte.width * cloud_puncte.height);

    cloud_puncte.points[0].x = 0.1;
    cloud_puncte.points[0].y = 0.1;
    cloud_puncte.points[0].z = 0.1;

    cloud_puncte.points[1].x = cloud_puncte.points[0].x + x_translation;
    cloud_puncte.points[1].y = cloud_puncte.points[0].y + y_translation;
    cloud_puncte.points[1].z = cloud_puncte.points[0].z + z_translation;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR1(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPTR1 = cloud_puncte;

    ////////////////////////////////////////////
    // Transform the 2 points in the pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    Transform_pointcloud(cloudPTR1,
                         transformed_cloud,
                         dimension_scale,
                         0,
                         0,
                         0);

    /////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ> cloud_Final;

    cloud_Final.width = 1;
    cloud_Final.height = 1;
    cloud_Final.is_dense = false;
    cloud_Final.resize(cloud_Final.width * cloud_Final.height);

    cloud_Final.points[0].x = transformed_cloud->points[1].x - transformed_cloud->points[0].x;
    cloud_Final.points[0].y = transformed_cloud->points[1].y - transformed_cloud->points[0].y;
    cloud_Final.points[0].z = transformed_cloud->points[1].z - transformed_cloud->points[0].z;

    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR2(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPTR2 = cloud_Final;

    ////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);

    Transform_pointcloud(cloudPTR2, transformed_cloud_2, point_movement_speed, point_x_translation, point_y_translation, point_z_translation);

    /////////////////////////////////////////////

     pcl::PointCloud<pcl::PointXYZ> cloud_proximitate;

    cloud_proximitate.width = 0;
    cloud_proximitate.height = 1;
    cloud_proximitate.is_dense = false;
    cloud_proximitate.resize(cloud_proximitate.width * cloud_proximitate.height);

/*
    cloud_proximitate.points[0].x = 5;
    cloud_proximitate.points[0].y =5;
    cloud_proximitate.points[0].z = 5;

    */

    
    check_distance(cloudPTR,
                   transformed_cloud_2,
                   cloud_proximitate,
                   proximity_threshold
                   );

                   
    ///////////////////

    sensor_msgs::PointCloud2 tempROSMsg;
    sensor_msgs::PointCloud2 tempROSMsg2;
    sensor_msgs::PointCloud2 tempROSMsg3;

    pcl::toROSMsg(*transformed_cloud_2, tempROSMsg2);
    pcl::toROSMsg(*transformed_cloud, tempROSMsg);
    pcl::toROSMsg(cloud_proximitate, tempROSMsg3);

    tempROSMsg.header.frame_id = "camera_depth_optical_frame";
    tempROSMsg2.header.frame_id = "camera_depth_optical_frame";
    tempROSMsg3.header.frame_id = "camera_depth_optical_frame";

    pub1_.publish(tempROSMsg);
    pub2_.publish(tempROSMsg2);
    pub3_.publish(tempROSMsg3);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub1_;
  ros::Publisher pub2_;
   ros::Publisher pub3_;
  dynamic_reconfigure::Server<pcl_tutorial::controlling_point_nodeConfig> config_server_;

  double dimension_scale;
  double x_translation;
  double y_translation;
  double z_translation;

  double point_movement_speed;
  double point_x_translation;
  double point_y_translation;
  double point_z_translation;

  double proximity_threshold;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controlling_point_node");

  ControllingPointNode vf;

  ros::spin();
}
