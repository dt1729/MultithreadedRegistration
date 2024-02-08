#include <librealsense2/rs.hpp> 
#include <open3d/Open3D.hpp>
#include <Eigen/Eigen.h>

using namespace rs2;

/**
 * @brief A class to capture rgbd image from realsense sensors.
 * 
 */
class rgbd_image_capture{
    public:
        tio::RealSenseSensor rs;

        void rgbd_image_capture(){
            // TODO: Add functions from JSON to load config file and bag_file name.

            // TODO: Add if condition to check how many sensors can be seen here, if none fail the initalisation and return 0;
            this->rs.ListDevices();
            this->rs.InitSensor(rs_cfg, 0, bag_file);
            this->rs.StartCapture(true);  // true: start recording with capture
        }


    private:
        std::string config_file, bag_file;
}

/**
 * @brief A class to create an open3d point cloud out of any type of image/pointcloud capturing class
 * 
 */
class xyz_to_pointcloud : public rgbd_image_capture{
    public:
        void create_point_cloud(open3d::geometry::PointCloud &point_cloud){
            im_rgbd = rs.CaptureFrame(true, true);

            open3d::camera::PinholeCameraIntrinsic::PinholeCameraIntrinsic(width, height, std::move(intrinsic_mtrx));

            point_cloud = open3d::geometry::PointCloud::CreateFromDepthImage(std::move(im_rgbd), core::Tensor::Eye(4, core::Float32, core::Device("CPU:0")), core::Tensor::Eye(4, core::Float32, core::Device("CPU:0")), rs2_get_depth_scale(rs), 1.0f, 1,false);

            utility::LogInfo("{}", rs.GetMetadata().ToString());
        }
}


/***
 * 1. Initialise all the threads.
 * 2. Create a point cloud buffer, create an IMU buffer. 
 * 3. Spawn two threads:
 *      1. Reader from sensor.
 *      2. SLAM iteration, this replaces the code.
 * /