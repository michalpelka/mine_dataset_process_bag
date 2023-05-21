#include <vector>
#include <map>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "point_types.h"
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/Imu.h>
#include <filesystem>


geometry_msgs::Quaternion toRosMsg(Eigen::Quaternionf q) {
    q.normalize();
    geometry_msgs::Quaternion qs;
    qs.x = q.x();
    qs.y = q.y();
    qs.z = q.z();
    qs.w = q.w();
    return qs;
}

geometry_msgs::Vector3 toRosMsg(const Eigen::Vector3f &v) {
    geometry_msgs::Vector3 v3;
    v3.x = v.x();
    v3.y = v.y();
    v3.z = v.z();
    return v3;
}

geometry_msgs::Transform toRosMsg(const Eigen::Affine3f &mat) {
    geometry_msgs::Transform transform;
    transform.translation = toRosMsg(mat.translation());
    transform.rotation = toRosMsg(Eigen::Quaternionf(mat.rotation()));
    return transform;
}


int main(int argc, char **argv) {
    // parse command line arguments for directory to process
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <directory with csvs> <output_bag> \n";
        return 1;
    }
    const std::string input_directory = argv[1];
    const std::string output_bag_name = argv[2];


    std::cout << "Processing directory: " << input_directory << " creating bag " << output_bag_name << std::endl;

    rosbag::Bag output_bag;
    output_bag.open(output_bag_name, rosbag::bagmode::Write);
    std::map<double, Eigen::Affine3f> trajectory_nodes;
    // go for every file in the directory
    for (const auto &entry: boost::filesystem::directory_iterator(input_directory)) {
        if (entry.path().extension() == ".csv") {
            std::cout << "Processing file: " << entry.path() << std::endl;
            std::ifstream file(entry.path().c_str());
            std::string line;
            std::getline(file, line); // skip header
            while (std::getline(file, line)) {
                // replace ever , in the line with a space
                std::replace(line.begin(), line.end(), ',', ' ');
                std::stringstream ss(line);
                double timestamp;
                double r00, r01, r02, t0, r10, r11, r12, t1, r20, r21, r22, t2;
                double x, y, z;
                ss >> timestamp;
                ss >> r00;
                ss >> r01;
                ss >> r02;
                ss >> t0;
                ss >> r10;
                ss >> r11;
                ss >> r12;
                ss >> t1;
                ss >> r20;
                ss >> r21;
                ss >> r22;
                ss >> t2;

                Eigen::Affine3f pose {Eigen::Affine3f::Identity()};
                pose.translation() = Eigen::Vector3f(t0,t1,t2);
                Eigen::Matrix3f rot;
                rot << r00, r01, r02, r10, r11, r12, r20, r21, r22;
                auto q = Eigen::Quaternionf(rot);
                q.normalize();
                pose.rotate(q);
                trajectory_nodes[timestamp] = pose;
            }
        }
    }

    //create tf_tree for VLP16 rotation
    for (auto &[timestamp, pose]: trajectory_nodes)
    {
        tf2_msgs::TFMessage tf_msgs;
        tf_msgs.transforms.resize(1);
        auto &tf_msg = tf_msgs.transforms[0];
        tf_msg.header.stamp = ros::Time(timestamp);
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform = toRosMsg(pose);
        output_bag.write("/tf", ros::Time(timestamp - 0.001f), tf_msgs);
    }
    return 0;
}