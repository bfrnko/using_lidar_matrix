// Optimization node
#include <string>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string.h>
#include <utility>
#include <ros/package.h>
#include "point_xyzir.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/flann.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define PI 3.141592653589793238463
bool sensor_pair = 0;
cv::Mat raw_image;
pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZIR>);
image_transport::Publisher pub_img_dist;
cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
std::vector<double> rotm2eul(cv::Mat);
double * converto_imgpts(double x, double y, double z);
void sensor_info_callback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc);
pcl::PointCloud<pcl::PointXYZIR> organized_pointcloud(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud);


void image_projection ();


int main(int argc, char **argv)
{
  ROS_INFO("LetsGo");
  ros::init(argc, argv, "LiveProjection");
  ros::NodeHandle n;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "cv_camera/image_raw", 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(n, "velodyne_points", 5);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, pcl_sub);
  sync.registerCallback(boost::bind(&sensor_info_callback, _1, _2));

  image_transport::ImageTransport it(n);
  pub_img_dist = it.advertise("ImageProjection", 20);

  ros::spin();
  return 0;
}
double tmpxC;


double * converto_imgpts(double x, double y, double z)
{
  double colmap[50][3] =  {{0,0,0.5385},{0,0,0.6154},{0,0,0.6923},
                         {0,0,0.7692},{0,0,0.8462},{0,0,0.9231},
                         {0,0,1.0000},{0,0.0769,1.0000},{0,0.1538,1.0000},
                         {0,0.2308,1.0000},{0,0.3846,1.0000},{0,0.4615,1.0000},
                         {0,0.5385,1.0000},{0,0.6154,1.0000},{0,0.6923,1.0000},
                         {0,0.7692,1.0000},{0,0.8462,1.0000},{0,0.9231,1.0000},
                         {0,1.0000,1.0000},{0.0769,1.0000,0.9231},{0.1538,1.0000,0.8462},
                         {0.2308,1.0000,0.7692},{0.3077,1.0000,0.6923},{0.3846,1.0000,0.6154},
                         {0.4615,1.0000,0.5385},{0.5385,1.0000,0.4615},{0.6154,1.0000,0.3846},
                         {0.6923,1.0000,0.3077},{0.7692,1.0000,0.2308},{0.8462,1.0000,0.1538},
                         {0.9231,1.0000,0.0769},{1.0000,1.0000,0},{1.0000,0.9231,0},
                         {1.0000,0.8462,0},{1.0000,0.7692,0},{1.0000,0.6923,0},
                         {1.0000,0.6154,0},{1.0000,0.5385,0},{1.0000,0.4615,0},
                         {1.0000,0.3846,0},{1.0000,0.3077,0},{1.0000,0.2308,0},
                         {1.0000,0.1538,0},{1.0000,0.0769,0},{1.0000,0,0},
                         {0.9231,0,0},{0.8462,0,0},{0.7692,0,0},{0.6923,0,0}};
  tmpxC = x/z;
  double tmpyC = y/z;
  cv::Point2d planepointsC;

  planepointsC.x = tmpxC;
  planepointsC.y = tmpyC;

  double r2 = tmpxC*tmpxC + tmpyC*tmpyC;


  double r1 = pow(r2,0.5);
  double a0 = std::atan(r1);
  double a1 = a0*(1 + double(-0.12825674964571737)*pow(a0,2) + double(-0.012608237548114742)*pow(a0,4)
                      + double(0.00017289033019328565)*pow(a0,6) + double(-0.00035634233489559319)*pow(a0,8));
  planepointsC.x = (a1/r1)*tmpxC;
  planepointsC.y = (a1/r1)*tmpyC;
  planepointsC.x = double(1269.6086139494651)*planepointsC.x + double(911.09389667924859);
  planepointsC.y = double(1327.2259861277089)*planepointsC.y + double(421.82521390545230);


  double * img_coord = new double[2];
  *(img_coord) = planepointsC.x;
  *(img_coord+1) = planepointsC.y;

  return img_coord;
}

void sensor_info_callback(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc)
{

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img,"bgr8");
  raw_image = cv_ptr->image;
  pcl::fromROSMsg (*pc, *cloud);
  image_projection();
}

void image_projection()
{
    double colmap[50][3] =  {{0,0,0.5385},{0,0,0.6154},{0,0,0.6923},
                         {0,0,0.7692},{0,0,0.8462},{0,0,0.9231},
                         {0,0,1.0000},{0,0.0769,1.0000},{0,0.1538,1.0000},
                         {0,0.2308,1.0000},{0,0.3846,1.0000},{0,0.4615,1.0000},
                         {0,0.5385,1.0000},{0,0.6154,1.0000},{0,0.6923,1.0000},
                         {0,0.7692,1.0000},{0,0.8462,1.0000},{0,0.9231,1.0000},
                         {0,1.0000,1.0000},{0.0769,1.0000,0.9231},{0.1538,1.0000,0.8462},
                         {0.2308,1.0000,0.7692},{0.3077,1.0000,0.6923},{0.3846,1.0000,0.6154},
                         {0.4615,1.0000,0.5385},{0.5385,1.0000,0.4615},{0.6154,1.0000,0.3846},
                         {0.6923,1.0000,0.3077},{0.7692,1.0000,0.2308},{0.8462,1.0000,0.1538},
                         {0.9231,1.0000,0.0769},{1.0000,1.0000,0},{1.0000,0.9231,0},
                         {1.0000,0.8462,0},{1.0000,0.7692,0},{1.0000,0.6923,0},
                         {1.0000,0.6154,0},{1.0000,0.5385,0},{1.0000,0.4615,0},
                         {1.0000,0.3846,0},{1.0000,0.3077,0},{1.0000,0.2308,0},
                         {1.0000,0.1538,0},{1.0000,0.0769,0},{1.0000,0,0},
                         {0.9231,0,0},{0.8462,0,0},{0.7692,0,0},{0.6923,0,0}};

  cv::Mat new_image_raw;
  new_image_raw = raw_image.clone();

  //Extrinsic parameter: Transform Velodyne -> cameras
  tf::Matrix3x3 rot;
  rot.setRPY(-1.68054, 0.00197023, -1.54517);

  Eigen::MatrixXf t1(4,4),t2(4,4);
  t1 << rot.getRow(0)[0], rot.getRow(0)[1], rot.getRow(0)[2], 0.155639,
        rot.getRow(1)[0], rot.getRow(1)[1], rot.getRow(1)[2], 0.0327405,
        rot.getRow(2)[0], rot.getRow(2)[1], rot.getRow(2)[2], 0.178031,
        0, 0, 0, 1;
  t2 = t1.inverse();

  Eigen::Affine3f transform_A = Eigen::Affine3f::Identity();
  transform_A.matrix() << t2(0,0), t2(0,1), t2(0,2), t2(0,3),
      t2(1,0), t2(1,1), t2(1,2), t2(1,3),
      t2(2,0), t2(2,1), t2(2,2), t2(2,3),
      t2(3,0), t2(3,1), t2(3,2), t2(3,3);

  if (cloud->size() < 1)
    return;

  pcl::PointCloud<pcl::PointXYZIR> organized;
  organized = organized_pointcloud(cloud);

  for (pcl::PointCloud<pcl::PointXYZIR>::const_iterator it = organized.begin(); it != organized.end(); it++)
  {
    pcl::PointXYZIR itA;
    itA = pcl::transformPoint (*it, transform_A);
    if (itA.z < 0 or std::abs(itA.x/itA.z) > 1.2)
      continue;

    double * img_pts = converto_imgpts(itA.x, itA.y, itA.z);
    double length = sqrt(pow(itA.x,2) + pow(itA.y,2) + pow(itA.z,2)); //range of every point
    int color = std::min(round((length/30)*49), 49.0);

    if (img_pts[1] >=0 and img_pts[1] < 1080
        and img_pts[0] >=0 and img_pts[0] < 1920)
    {
      cv::circle(new_image_raw, cv::Point(img_pts[0], img_pts[1]), 3,
          CV_RGB(255*colmap[color][0], 255*colmap[color][1], 255*colmap[color][2]), -1);
    }
  }

  // Publish the image projection
  ros::Time time = ros::Time::now();
  cv_ptr->encoding = "bgr8";
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "/traj_output";
  cv_ptr->image = new_image_raw;
  pub_img_dist.publish(cv_ptr->toImageMsg());
}

pcl::PointCloud<pcl::PointXYZIR> organized_pointcloud(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZIR> organized_pc;
  pcl::KdTreeFLANN<pcl::PointXYZIR> kdtree;

  // Kdtree to sort the point cloud
  kdtree.setInputCloud (input_pointcloud);

  pcl::PointXYZIR searchPoint;// camera position as target
  searchPoint.x = 0.0f;
  searchPoint.y = 0.0f;
  searchPoint.z = 0.0f;

  int K = input_pointcloud->points.size();
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // Sort the point cloud based on distance to the camera
  if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    {
      pcl::PointXYZIR point;
      point.x = input_pointcloud->points[ pointIdxNKNSearch[i] ].x;
      point.y = input_pointcloud->points[ pointIdxNKNSearch[i] ].y;
      point.z = input_pointcloud->points[ pointIdxNKNSearch[i] ].z;
      point.intensity = input_pointcloud->points[ pointIdxNKNSearch[i] ].intensity;
      point.ring = input_pointcloud->points[ pointIdxNKNSearch[i] ].ring;
      organized_pc.push_back(point);
    }
  }

  //Return sorted point cloud
  return(organized_pc);
}
