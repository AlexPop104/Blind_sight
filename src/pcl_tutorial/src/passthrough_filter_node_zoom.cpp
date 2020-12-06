
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_tutorial/passthrough_filter_node_zoomConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>


class PassthroughFilterNodeZoom
{
public:

  PassthroughFilterNodeZoom()
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pf_zoom_out", 1);
    sub_ = nh_.subscribe ("/zoom_in_scalled_pointcloud", 1,  &PassthroughFilterNodeZoom::cloudCallback, this);
    config_server_.setCallback(boost::bind(&PassthroughFilterNodeZoom::dynReconfCallback, this, _1, _2));

    double z_upper_limit, z_lower_limit;
    double x_upper_limit, x_lower_limit;
    double y_upper_limit, y_lower_limit;

    // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
    ros::NodeHandle private_nh("~");

    //read parameters with default value
    private_nh.param("z_lower_limit", z_lower_limit, 2.);
    private_nh.param("z_upper_limit", z_upper_limit, 5.);

    pt1_.setFilterFieldName ("z");
    pt1_.setFilterLimits (z_lower_limit, z_upper_limit);
    
    // read parameters with default value
    private_nh.param("x_lower_limit", x_lower_limit, 2.);
    private_nh.param("x_upper_limit", x_upper_limit, 5.);

    pt2_.setFilterFieldName ("x");
    pt2_.setFilterLimits (x_lower_limit, x_upper_limit);
    
    //read parameters with default value
    private_nh.param("y_lower_limit", y_lower_limit, 2.);
    private_nh.param("y_upper_limit", y_upper_limit, 5.);

    pt3_.setFilterFieldName ("y");
    pt3_.setFilterLimits (y_lower_limit, y_upper_limit);
    
  }

  ~PassthroughFilterNodeZoom() {}

  void
  dynReconfCallback(pcl_tutorial::passthrough_filter_node_zoomConfig &config, uint32_t level)
  {
    pt1_.setFilterLimits(config.z_lower_limit, config.z_upper_limit);
    pt2_.setFilterLimits(config.x_lower_limit, config.x_upper_limit);
    pt3_.setFilterLimits(config.y_lower_limit, config.y_upper_limit);
  }

  void
  cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
      pcl::PointCloud<pcl::PointXYZ> cloud_Test;
      pcl::fromROSMsg(*cloud_msg, cloud_Test);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_in = cloud_Test;

    pt1_.setInputCloud(cloud_in);
    pcl::PointCloud<pcl::PointXYZ> cloud_out1;
    pt1_.filter(cloud_out1);
    
    
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_2(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_in_2 = cloud_out1;
 
	pt2_.setInputCloud(cloud_in_2);
    pcl::PointCloud<pcl::PointXYZ> cloud_out2;
    pt2_.filter(cloud_out2);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_3(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_in_3 = cloud_out2;
 
	pt3_.setInputCloud(cloud_in_3);
    pcl::PointCloud<pcl::PointXYZ> cloud_out3;
    pt3_.filter(cloud_out3);
    
    
    
    pub_.publish(cloud_out3);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<pcl_tutorial::passthrough_filter_node_zoomConfig> config_server_;

  pcl::PassThrough<pcl::PointXYZ> pt1_,pt2_,pt3_;

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "voxel_filter_node");

  PassthroughFilterNodeZoom vf;

  ros::spin();
}

