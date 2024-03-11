#include <librealsense2/rs.hpp> 
#include <mutex>
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <cstring>
#include <open3d/Open3D.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen.h>

using namespace rs2;

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
};

/**
 * @brief A class to capture rgbd image from realsense sensors.
 * 
 */
class d455_frame_capture{
    public:
        rs2::tio::RealSenseSensor rs;
        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;

        void d455_frame_capture(){
            // TODO: Add functions from JSON to load config file and bag_file name.

            // Create a configuration for configuring the pipeline with a non default profile
            config cfg;
            // Add streams of gyro and accelerometer to configuration
            cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

            // TODO: Add if condition to check how many sensors can be seen here, if none fail the initalisation and return 0;
            this->rs.ListDevices();
            this->rs.InitSensor(cfg, 0, bag_file);
            this->rs.StartCapture(false);  // true: start recording with capture
        }

    private:
        std::string config_file, bag_file;
}

class rotation_estimator
{
    // theta is the angle of camera rotation in x, y and z components
    rs2::float3 theta;
    std::mutex theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98f;
    bool firstGyro = true;
    bool firstAccel = true;
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;

public:
    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(rs2_vector gyro_data, double ts)
    {
        if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            firstGyro = false;
            last_ts_gyro = ts;
            return;
        }
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;

        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = (ts - last_ts_gyro) / 1000.0;
        last_ts_gyro = ts;

        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

        // Apply the calculated change of angle to the current angle (theta)
        std::lock_guard<std::mutex> lock(theta_mtx);
        theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    }

    void process_accel(rs2_vector accel_data)
    {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;

        // Calculate rotation angle from accelerometer data
        accel_angle.z = atan2(accel_data.y, accel_data.z);
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        std::lock_guard<std::mutex> lock(theta_mtx);
        if (firstAccel)
        {
            firstAccel = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            theta.y = PI_FL;
        }
        else
        {
            /* 
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }

    /**
     * @brief Get the theta object
     * 
     * @return * Returns 
     */
    float3 get_theta()
    {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }
};

/**
 * @brief A class to create an open3d point cloud out of any type of image/pointcloud capturing class
 * 
 */
class sensor_to_buffer : public d455_frame_capture{
    public:
        void data_to_buffer(std::queue<buffer_struct> qq){
            pipe.start();

            auto im_rgbd = rs.CaptureFrame(true, true);
            buffer_struct buffer();

            open3d::camera::PinholeCameraIntrinsic::PinholeCameraIntrinsic(width, height, std::move(intrinsic_mtrx));

            buffer.point_cloud = open3d::geometry::PointCloud::CreateFromDepthImage(std::move(im_rgbd), core::Tensor::Eye(4, core::Float32, core::Device("CPU:0")), core::Tensor::Eye(4, core::Float32, core::Device("CPU:0")), rs2_get_depth_scale(rs), 1.0f, 1,false);

            buffer.imu_data = create_imu_data(std::move(im_rgbd));

            buffer.time_stamp = im_rgbd.time_stamp() ;
            std::lock_guard<std::mutex> lck(m);
                qq.push_back(buffer);
        }

        void create_imu_data(rs::frame&& frame){
            rotation_estimator algo;
            // Cast the frame that arrived to motion frame
            auto motion = frame->as<rs2::motion_frame>();
            // If casting succeeded and the arrived frame is from gyro stream
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get the timestamp of the current frame
                double ts = motion.get_timestamp();
                // Get gyro measures
                rs2_vector gyro_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                algo.process_gyro(gyro_data, ts);
            }
            // If casting succeeded and the arrived frame is from accelerometer stream
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get accelerometer measures
                rs2_vector accel_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                algo.process_accel(accel_data);
            }
            return algo.get_theta();
        }

    private: 
        open3d::geometry::PointCloud point_cloud;
}


void slam_call(std::queue<buffer_struct> qq, std::mutex m){
    FeatureAssociation FA;
    ImageProjection IP;
    mapOptimization MO;
    TransformFusion TFusion;

    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);

    ros::Rate rate(200);

    while (ros::ok())
    {
        ros::spinOnce();
        FA.laserCloudCopier(qq, m);
        FA.runFeatureAssociation(); // Add parameter 
        MO.run();
        rate.sleep();
    }

    loopthread.join();
}

/***
 * 1. Initialise all the threads.
 * 2. Create a point cloud and IMU buffer. 
 * 3. Spawn two threads:
 *      1. Reader from sensor.
 *      2. SLAM iteration.
*/

int main(int argc, char *argv[]){
    ros::init(argc, argv, "lego_loam");
    std::mutex tt;
    sensor_to_buffer s2b;
    std::queue<buffer_struct> producer_consumer_queue; // Should this be a part of the bigger structure? How should this be shared as a variable between two threads

    std::thread sensor_reader_thread(&s2b.data_to_buffer, &s2b, producer_consumer_queue, tt);
    std::thread slam_thread(slam_call, producer_consumer_queue, tt);

    sensor_reader_thread.join();
    slam_thread.join();
    return 0;
}
