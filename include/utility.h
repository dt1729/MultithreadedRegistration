#include <open3d/Open3D.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen.h>

struct imu_data{
    float roll;
    float pitch;
    float yaw;
    float imuAccX;
    float imuAccY;
    float imuAccZ;
    float imuAngularVeloX;
    float imuAngularVeloY;
    float imuAngularVeloZ;
};

struct buffer_struct{
    open3d::geometry::PointCloud point_cloud; // Can i use template here to abstract out type between open3d and pcl?
    imu_data imu_data;

    pcl::PointCloud convert(){
        const uint32_t size = point_cloud->points_.size();

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl_cloud->width = size;
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = false;
        pcl_cloud->points.resize( size );

        #pragma omp parallel for
        for( int32_t i = 0; i < size; i++ ){
            pcl_cloud->points[i].getVector3fMap() = point_cloud->points_[i].cast<float>();
        }

        return pcl_cloud;
    }
};
