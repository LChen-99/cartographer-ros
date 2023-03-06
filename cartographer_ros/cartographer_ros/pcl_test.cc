
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <chrono>
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(100,1));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  for (auto& point : *cloud_in)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  
  std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;

  *cloud_out = *cloud_in;
  
  std::cout << "size:" << cloud_out->size() << std::endl;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.1, 0.2, 0.15;
  transform.rotate(Eigen::AngleAxisf(M_PI / 60, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*cloud_in, *cloud_out, transform);  
  srand(time(NULL));
  for(auto& point : *cloud_out){
    point.x += 0.02 * rand() / (RAND_MAX + 1.0f);
    point.y += 0.02 * rand() / (RAND_MAX + 1.0f);
    point.z += 0.02 * rand() / (RAND_MAX + 1.0f);
  }


  std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();



  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::cout << "cost " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us." << std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

 return 0;
}