#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <assert.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/ndt.h>
#include <unordered_map>
#include <numeric>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
#include "mls.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//#include <pcl/surface/impl/mls.hpp>

#include <pcl/surface/impl/gp3.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(GreedyProjectionTriangulation, (pcl::PointNormal)(pcl::PointXYZRGBNormal)(pcl::PointXYZINormal))



ros::Publisher marker_pub;
ros::Publisher points_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr LoadPcdFile(std::string file_name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "try to read " << file_name << " ." << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        abort();
    }
    std::cout << "read " << file_name << " success!" << std::endl;
    return cloud;
}

void load_mesh(pcl::PolygonMesh &triangles, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,visualization_msgs::MarkerArray &marker_array)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "mesh";
    marker.id = 3;
    //marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.9;
    marker.color.b = 0.7;
    marker.color.a = 1;

    for (auto p : triangles.polygons)
    {
        auto n0 = cloud->points[p.vertices[0]];
        auto n1 = cloud->points[p.vertices[1]];
        auto n2 = cloud->points[p.vertices[2]];
        Eigen::Vector3f e0 = n0.getVector3fMap();
        Eigen::Vector3f e1 = n1.getVector3fMap();
        Eigen::Vector3f e2 = n2.getVector3fMap();


        Eigen::Vector3f  norm =  (e1 - e0).cross(e2 - e0);
        norm /= norm.norm();

        geometry_msgs::Point A, B, C;
        A.x = n0.x;
        A.y = n0.y;
        A.z = n0.z;
        B.x = n1.x;
        B.y = n1.y;
        B.z = n1.z;
        C.x = n2.x;
        C.y = n2.y;
        C.z = n2.z;
        if (norm.z() > 0)
        {
            marker.points.push_back(A);
            marker.points.push_back(B);
            marker.points.push_back(C);
        }
        else
        {
            marker.points.push_back(C);
            marker.points.push_back(B);
            marker.points.push_back(A);
        }
    }
    marker_array.markers.push_back(marker);
}


pcl::PointCloud<pcl::PointNormal>::Ptr
smooth (pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_pre,
         double search_radius,
         bool sqr_gauss_param_set,
         double sqr_gauss_param,
         int polynomial_order)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr 
      xyz_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      
  // Filter the NaNs from the cloud
  for (std::size_t i = 0; i < xyz_cloud_pre->size (); ++i)
    if (std::isfinite (xyz_cloud_pre->points[i].x))
      xyz_cloud->push_back (xyz_cloud_pre->points[i]);
  xyz_cloud->header = xyz_cloud_pre->header;
  xyz_cloud->height = 1;
  xyz_cloud->width = static_cast<std::uint32_t> (xyz_cloud->size ());
  xyz_cloud->is_dense = false;
  
  

  pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud_smoothed (new pcl::PointCloud<pcl::PointNormal> ());

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setInputCloud (xyz_cloud);
  mls.setSearchRadius (search_radius);
  if (sqr_gauss_param_set) mls.setSqrGaussParam (sqr_gauss_param);
  mls.setPolynomialOrder (polynomial_order);

//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::RANDOM_UNIFORM_DENSITY);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::VOXEL_GRID_DILATION);
  mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::NONE);
  mls.setPointDensity (60000 * int (search_radius)); // 300 points in a 5 cm radius
  mls.setUpsamplingRadius (0.025);
  mls.setUpsamplingStepSize (0.015);
  mls.setDilationIterations (2);
  mls.setDilationVoxelSize (0.01f);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  mls.setSearchMethod (tree);
  mls.setComputeNormals (true);

  PCL_INFO ("Computing smoothed surface and normals with search_radius %f , sqr_gaussian_param %f, polynomial order %d\n",
            mls.getSearchRadius(), mls.getSqrGaussParam(), mls.getPolynomialOrder());
  mls.process (*xyz_cloud_smoothed);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::copyPointCloud(*xyz_cloud_smoothed, *mls_cloud);

  return xyz_cloud_smoothed;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "trav");
    ros::NodeHandle nh;

    assert(argc >= 2);
    std::cout << "pcd file:" << argv[1] << std::endl;
    auto cloud_raw = LoadPcdFile(argv[1]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    pcl::VoxelGrid<pcl::PointNormal> avg;
    //avg.setLeafSize(0.5f, 0.5f, 0.5f);
    avg.setLeafSize(0.1f, 0.1f, 0.1f);

    auto smoothed_cloud = smooth(cloud_raw,0.3,true,0.09,0);
    std::cout << "smooth OK!"  << std::endl;
    avg.setInputCloud(smoothed_cloud);
    avg.filter(*cloud_with_normals);
    pcl::copyPointCloud(*cloud_with_normals, *cloud);

    std::cout << "filter OK!"  << std::endl;

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1, true);
    points_pub = nh.advertise<sensor_msgs::PointCloud2 >("points", 100, true);
    /*
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(100);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures
    std::cout << "normals OK!"  << std::endl;
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    */
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.8);

    // Set typical values for the parameters
    gp3.setMu(10);
    gp3.setMaximumNearestNeighbors(20);
    gp3.setMaximumSurfaceAngle(M_PI /6); // 45 degrees
    gp3.setMinimumAngle(M_PI /18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    std::cout << "mesh OK!"  << std::endl;
    visualization_msgs::MarkerArray marker_array;
    load_mesh(triangles, cloud, marker_array);
    marker_pub.publish(marker_array);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
    points_pub.publish(output);


    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    ros::spin();
    // Finish
    ;
}
