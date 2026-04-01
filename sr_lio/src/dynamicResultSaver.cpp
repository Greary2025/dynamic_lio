#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace
{
using PointType = pcl::PointXYZI;

struct PoseEntry
{
    double stamp = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

class DynamicResultSaver
{
public:
    explicit DynamicResultSaver(ros::NodeHandle &nh)
        : frontend_raw_map_(new pcl::PointCloud<PointType>()),
          frontend_static_map_(new pcl::PointCloud<PointType>()),
          backend_map_(new pcl::PointCloud<PointType>())
    {
        nh.param<std::string>("output_dir", output_dir_, "/mnt/d/rosbag_evo/0331single/dynamic/xianfeng");
        nh.param<double>("save_interval_sec", save_interval_sec_, 2.0);

        std::filesystem::create_directories(output_dir_);

        frontend_odom_sub_ = nh.subscribe("/Odometry_after_opt", 2000, &DynamicResultSaver::frontendOdomHandler, this);
        frontend_raw_map_sub_ = nh.subscribe("/loop_map", 100, &DynamicResultSaver::frontendRawMapHandler, this);
        frontend_static_map_sub_ = nh.subscribe("/cloud_global_map", 100, &DynamicResultSaver::frontendStaticMapHandler, this);
        backend_path_sub_ = nh.subscribe("/aft_pgo_path", 100, &DynamicResultSaver::backendPathHandler, this);
        backend_map_sub_ = nh.subscribe("/aft_pgo_map", 10, &DynamicResultSaver::backendMapHandler, this);

        flush_timer_ = nh.createTimer(ros::Duration(save_interval_sec_), &DynamicResultSaver::flushTimerCallback, this);

        ROS_INFO_STREAM("dynamic_result_saver output dir: " << output_dir_);
    }

    ~DynamicResultSaver()
    {
        flushOutputs(true);
    }

private:
    void frontendOdomHandler(const nav_msgs::Odometry::ConstPtr &msg)
    {
        const double stamp = msg->header.stamp.toSec();

        std::lock_guard<std::mutex> lock(mutex_);
        if (!frontend_traj_.empty() && stamp <= frontend_traj_.back().stamp + 1e-9)
            return;

        PoseEntry pose;
        pose.stamp = stamp;
        pose.tx = msg->pose.pose.position.x;
        pose.ty = msg->pose.pose.position.y;
        pose.tz = msg->pose.pose.position.z;
        pose.qx = msg->pose.pose.orientation.x;
        pose.qy = msg->pose.pose.orientation.y;
        pose.qz = msg->pose.pose.orientation.z;
        pose.qw = msg->pose.pose.orientation.w;
        frontend_traj_.push_back(pose);
        frontend_traj_dirty_ = true;
    }

    void frontendRawMapHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointType> incoming_cloud;
        pcl::fromROSMsg(*msg, incoming_cloud);

        std::lock_guard<std::mutex> lock(mutex_);
        *frontend_raw_map_ += incoming_cloud;
        frontend_raw_map_dirty_ = true;
    }

    void frontendStaticMapHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointType> incoming_cloud;
        pcl::fromROSMsg(*msg, incoming_cloud);

        std::lock_guard<std::mutex> lock(mutex_);
        *frontend_static_map_ += incoming_cloud;
        frontend_static_map_dirty_ = true;
    }

    void backendPathHandler(const nav_msgs::Path::ConstPtr &msg)
    {
        std::vector<PoseEntry> path_entries;
        path_entries.reserve(msg->poses.size());

        for (const auto &pose_msg : msg->poses)
        {
            PoseEntry pose;
            pose.stamp = pose_msg.header.stamp.toSec();
            if (pose.stamp <= 0.0)
                pose.stamp = msg->header.stamp.toSec();
            pose.tx = pose_msg.pose.position.x;
            pose.ty = pose_msg.pose.position.y;
            pose.tz = pose_msg.pose.position.z;
            pose.qx = pose_msg.pose.orientation.x;
            pose.qy = pose_msg.pose.orientation.y;
            pose.qz = pose_msg.pose.orientation.z;
            pose.qw = pose_msg.pose.orientation.w;
            path_entries.push_back(pose);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        backend_traj_.swap(path_entries);
        backend_traj_dirty_ = true;
    }

    void backendMapHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointType> incoming_cloud;
        pcl::fromROSMsg(*msg, incoming_cloud);

        std::lock_guard<std::mutex> lock(mutex_);
        *backend_map_ = incoming_cloud;
        backend_map_dirty_ = true;
    }

    void flushTimerCallback(const ros::TimerEvent &)
    {
        flushOutputs(false);
    }

    void flushOutputs(bool force)
    {
        std::vector<PoseEntry> frontend_traj_copy;
        std::vector<PoseEntry> backend_traj_copy;
        pcl::PointCloud<PointType>::Ptr frontend_raw_map_copy(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr frontend_static_map_copy(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr backend_map_copy(new pcl::PointCloud<PointType>());

        bool write_frontend_traj = false;
        bool write_backend_traj = false;
        bool write_frontend_raw_map = false;
        bool write_frontend_static_map = false;
        bool write_backend_map = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            write_frontend_traj = (force || frontend_traj_dirty_) && !frontend_traj_.empty();
            write_backend_traj = (force || backend_traj_dirty_) && !backend_traj_.empty();
            write_frontend_raw_map = (force || frontend_raw_map_dirty_) && !frontend_raw_map_->empty();
            write_frontend_static_map = (force || frontend_static_map_dirty_) && !frontend_static_map_->empty();
            write_backend_map = (force || backend_map_dirty_) && !backend_map_->empty();

            if (write_frontend_traj)
                frontend_traj_copy = frontend_traj_;
            if (write_backend_traj)
                backend_traj_copy = backend_traj_;
            if (write_frontend_raw_map)
                *frontend_raw_map_copy = *frontend_raw_map_;
            if (write_frontend_static_map)
                *frontend_static_map_copy = *frontend_static_map_;
            if (write_backend_map)
                *backend_map_copy = *backend_map_;

            frontend_traj_dirty_ = false;
            backend_traj_dirty_ = false;
            frontend_raw_map_dirty_ = false;
            frontend_static_map_dirty_ = false;
            backend_map_dirty_ = false;
        }

        std::filesystem::create_directories(output_dir_);

        if (write_frontend_traj)
            saveTrajectory(frontend_traj_copy, output_dir_ + "/frontend_traj.txt");
        if (write_backend_traj)
            saveTrajectory(backend_traj_copy, output_dir_ + "/backend_traj.txt");
        if (write_frontend_raw_map)
            pcl::io::savePCDFileBinary(output_dir_ + "/frontend_map_raw.pcd", *frontend_raw_map_copy);
        if (write_frontend_static_map)
            pcl::io::savePCDFileBinary(output_dir_ + "/frontend_map_static.pcd", *frontend_static_map_copy);
        if (write_backend_map)
            pcl::io::savePCDFileBinary(output_dir_ + "/backend_map.pcd", *backend_map_copy);
    }

    void saveTrajectory(const std::vector<PoseEntry> &trajectory, const std::string &file_path) const
    {
        std::ofstream output(file_path, std::ios::out | std::ios::trunc);
        output << std::fixed << std::setprecision(9);

        for (const auto &pose : trajectory)
        {
            output << pose.stamp << " "
                   << pose.tx << " " << pose.ty << " " << pose.tz << " "
                   << pose.qx << " " << pose.qy << " " << pose.qz << " " << pose.qw << "\n";
        }
    }

    std::string output_dir_;
    double save_interval_sec_ = 2.0;

    ros::Subscriber frontend_odom_sub_;
    ros::Subscriber frontend_raw_map_sub_;
    ros::Subscriber frontend_static_map_sub_;
    ros::Subscriber backend_path_sub_;
    ros::Subscriber backend_map_sub_;
    ros::Timer flush_timer_;

    std::mutex mutex_;
    std::vector<PoseEntry> frontend_traj_;
    std::vector<PoseEntry> backend_traj_;
    pcl::PointCloud<PointType>::Ptr frontend_raw_map_;
    pcl::PointCloud<PointType>::Ptr frontend_static_map_;
    pcl::PointCloud<PointType>::Ptr backend_map_;

    bool frontend_traj_dirty_ = false;
    bool backend_traj_dirty_ = false;
    bool frontend_raw_map_dirty_ = false;
    bool frontend_static_map_dirty_ = false;
    bool backend_map_dirty_ = false;
};
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_result_saver");
    ros::NodeHandle nh("~");

    DynamicResultSaver saver(nh);
    ros::spin();

    return 0;
}
