#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

class PCDownSampler {
  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    
    
    float leaf_size_;
    
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter_;
    
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void publishCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std_msgs::Header& header);
    
  public:
    PCDownSampler();
};

void PCDownSampler::
cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Convert cloud to PCL format
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2 ());
  pcl_conversions::toPCL(*msg, *cloud);
  
  // Perform initial down sampling
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  voxel_filter_.setInputCloud(cloud);
  voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_filter_.filter(*cloud_filtered);

  // Determine if additional down sampling is required
  while(cloud_filtered->width * cloud_filtered->height > 600000)
  {
    // Increase new leaf size and down sample again until the cloud size is below the initial cloud size
    leaf_size_ = leaf_size_ + 0.02f;
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_filter_.filter(*cloud_filtered);
  }
  
  // Publish pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud_filtered, *(tmp));
  publishCloud(tmp, msg->header);
}

void PCDownSampler::
publishCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std_msgs::Header& header) {
  sensor_msgs::PointCloud2 msg;
  msg.data.resize(12 * cloud->points.size());
  msg.width = cloud->points.size();
  msg.height = 1;
  msg.row_step = cloud->points.size() * 12;
  msg.is_dense = true;
  sensor_msgs::PointField fx;
  fx.name="x";
  fx.offset = 0;
  fx.datatype = 7;
  fx.count = 1;
  msg.fields.push_back(fx);
  fx.name="y";
  fx.offset = 4;
  msg.fields.push_back(fx);
  fx.name="z";
  fx.offset=8;
  msg.fields.push_back(fx);
  msg.point_step = 12;
  for (size_t i = 0; i < cloud->points.size(); i++) {
    *(float*)(&msg.data[i*12 + 0]) = cloud->points[i].x;
    *(float*)(&msg.data[i*12 + 4]) = cloud->points[i].y;
    *(float*)(&msg.data[i*12 + 8]) = cloud->points[i].z;
  }

  msg.header.seq = header.seq;
  msg.header.stamp = header.stamp;
  msg.header.frame_id = header.frame_id;
  cloud_pub_.publish(msg);
}

PCDownSampler::PCDownSampler() {
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_("~");
  leaf_size_ = 0.1f;
  cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud_throttled", 1, &PCDownSampler::cloudCallback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_throttled_downsampled", 1, true);
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_downsample");
  PCDownSampler pc_down_sampler;
  ros::spin();
  return 0;
}
