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

const double angular_encoder_offset = -5.53747 - 0.98 * M_PI;

float getTimestamp(const pcl::PointXYZINormal &point) {
    return point.normal_y;
}

float getAngle(const pcl::PointXYZINormal &point) {
    return point.normal_x;
}

int getRing(const pcl::PointXYZINormal &point) {
    return point.normal_z;
}

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


Eigen::Affine3f calibMatrixVLP32C() {
   Eigen::Matrix4f vlp32c;
    //cad
    vlp32c << 0.0000, 0.6428, 0.7660, 0.2061,
            0.7049, -0.5433, 0.4559, 0.0325,
            0.7093, 0.5400, -0.4531, -0.1567,
            0.0000, 0.0000, 0.0000, 1.0000;
    return Eigen::Affine3f(vlp32c);
}

Eigen::Affine3f calibMatrixVLP16() {
    Eigen::Matrix4f vlp16;
    //cad
    vlp16 << 0.0000, 0.0000, 1.0000, -0.0678,
            -0.0000, 1.0000, -0.0000, -0.1292,
            -1.0000, -0.0000, 0.0000, -0.0584,
            0.0000, 0.0000, 0.0000, 1.0000;
    return Eigen::Affine3f(vlp16);
}

Eigen::Affine3f calibMatrixIMU() {
    Eigen::Matrix4f imu;
    //cad
    imu << 0.0, 0.0, 1.0, -0.0680,
            0.0, 1.0, 0.0, -0.1304,
            -1.0, 0.0, 0.0, -0.0509,
            0.0, 0.0, 0.0, 1.0;
    return Eigen::Affine3f(imu);
}

Eigen::Affine3f calibMatrixVLP16Local() {
    Eigen::Matrix4f vlp16_calib;
    vlp16_calib << 0.999921, -0.00638093, 0.0107958, -0.00731472,
            0.00634553, 0.999974, 0.00331037, 0.0463718,
            -0.0108166, -0.00324161, 0.999936, 0.0222408,
            0, 0, 0, 1;
    Eigen::Affine3f vlp16_calib_affine(vlp16_calib);
    Eigen::Affine3f rot0(Eigen::Affine3f::Identity());
    rot0.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));
    Eigen::Affine3f rot1(Eigen::Affine3f::Identity());
    rot1.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f staticTransform = rot1 * rot0;
    return rot1 * rot0 * vlp16_calib_affine;
}

void addStaticTransformTree(rosbag::Bag &output_bag, float bagStartTimestamp) {
    // write static tf for vlp16 from rotation to laser
    tf2_msgs::TFMessage tf_msgs;
    tf_msgs.transforms.resize(5);
    tf_msgs.transforms[0].header.stamp = ros::Time(bagStartTimestamp);
    tf_msgs.transforms[0].header.frame_id = "velodyne_rot_angular_offset";
    tf_msgs.transforms[0].child_frame_id = "velodyne_rot";
    tf_msgs.transforms[0].transform = toRosMsg(calibMatrixVLP16Local());

    tf_msgs.transforms[1].header.stamp = ros::Time(bagStartTimestamp);
    tf_msgs.transforms[1].header.frame_id = "base_link";
    tf_msgs.transforms[1].child_frame_id = "velodyne_rot_base";
    tf_msgs.transforms[1].transform = toRosMsg(calibMatrixVLP16());

    tf_msgs.transforms[2].header.stamp = ros::Time(bagStartTimestamp);
    tf_msgs.transforms[2].header.frame_id = "base_link";
    tf_msgs.transforms[2].child_frame_id = "velodyne";
    tf_msgs.transforms[2].transform = toRosMsg(calibMatrixVLP32C());

    tf_msgs.transforms[3].header.stamp = ros::Time(bagStartTimestamp);
    tf_msgs.transforms[3].header.frame_id = "base_link";
    tf_msgs.transforms[3].child_frame_id = "livox";
    tf_msgs.transforms[3].transform = toRosMsg(Eigen::Affine3f::Identity());

    tf_msgs.transforms[4].header.stamp = ros::Time(bagStartTimestamp);
    tf_msgs.transforms[4].header.frame_id = "base_link";
    tf_msgs.transforms[4].child_frame_id = "imu";
    tf_msgs.transforms[4].transform = toRosMsg(calibMatrixIMU());

    boost::shared_ptr<ros::M_string> header(new ros::M_string);
    (*header)["callerid"] = "/static_transform_publisher";
    (*header)["latching"] = "1";
    (*header)["md5sum"] = ros::message_traits::MD5Sum<tf2_msgs::TFMessage>::value();
    (*header)["topic"] = "/tf_static";
    (*header)["type"] = "tf2_msgs/TFMessage";
    (*header)["message_definition"] = ros::message_traits::Definition<tf2_msgs::TFMessage>::value();
    output_bag.write("/tf_static", ros::Time(bagStartTimestamp - 0.001f), tf_msgs, header);
}

void savePointclouds(rosbag::Bag &output_bag, const std::string &topicName, const std::string &frameName,
                     const std::deque<pcl::PointXYZINormal> &points, float pointcloudLen, float scale = 1.0f) {
    double time_beg = getTimestamp(points.front());
    double time_end = getTimestamp(points.back());

    float startTimestampPointCloud = getTimestamp(points.front());
    pcl::PointCloud<mine_dataset::PointXYZIRT> pc_pcl;
    for (auto &p: points) {
        auto timestamp = getTimestamp(p);
        if (timestamp > time_beg && timestamp < time_end) {
            float delta = getTimestamp(p) - startTimestampPointCloud;
            mine_dataset::PointXYZIRT p_new;
            p_new.getArray3fMap() = scale * p.getArray3fMap();
            p_new.intensity = p.intensity;
            p_new.ring = getRing(p);
            p_new.time = getTimestamp(p);
            pc_pcl.push_back(p_new);
            if (delta > pointcloudLen) {
                startTimestampPointCloud = getTimestamp(p);
                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(pc_pcl, pc_msg);
                pc_msg.header.stamp = ros::Time(startTimestampPointCloud);
                pc_msg.header.frame_id = frameName;
                output_bag.write(topicName, ros::Time(startTimestampPointCloud), pc_msg);
                pc_pcl.clear();
            }
        }
    }
}

int main(int argc, char **argv) {
    // parse command line arguments for directory to process
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input_bag> <output_bag> --add_data --add_static_tf \n";
        std::cout << " Options are:\n";
        std::cout << "  --pc_len <length of one chunk of pointcloud> (default 0.1 s)" << std::endl;
        std::cout << "  --add_static_tf <add static tf to output bag>" << std::endl;
        std::cout << "  --append allow to append to rosbag" << std::endl;
        return 1;
    }
    bool append = false;
    bool add_static_tf = false;
    std::string calib = "CAD";
    float pc_len = 0.1;
    const std::string input_bag_name = argv[1];
    const std::string output_bag_name = argv[2];
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.size() > 1 && arg[0] == '-' && arg[1] == '-') {
            if (arg == "--pc_len") {
                pc_len = std::stof(argv[i + 1]);
                i++;
            } else if (arg == "--calib") {
                calib = std::string(argv[i + 1]);
                i++;
            } else if (arg == "--append") {
                append = true;
            } else if (arg == "--add_static_tf") {
                add_static_tf = true;
            } else {
                std::cout << "Unknown option: " << arg << std::endl;
                return 1;
            }
        }
    }

    std::cout << "Processing bag: " << input_bag_name << " creating bag " << output_bag_name << std::endl;
    std::cout << "length of one chunk of pointcloud: " << pc_len << std::endl;

    rosbag::Bag input_bag;
    rosbag::Bag output_bag;

    std::deque<pcl::PointXYZINormal> velodyne_rot_deque;
    std::deque<pcl::PointXYZINormal> velodyne_stationary;
    std::deque<pcl::PointXYZINormal> livox;

    std::map<float, float> rotation_angle;
    input_bag.open(input_bag_name, rosbag::bagmode::Read);
    if (append) {
        output_bag.open(output_bag_name, rosbag::bagmode::Append);
    } else {
        output_bag.open(output_bag_name, rosbag::bagmode::Write);
    }
    rosbag::View view(input_bag);
    for (rosbag::MessageInstance const m: view) {
        // read livox and store it in the queue
        if (m.getTopic() == "/livox_raw") {
            sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
            if (pc != nullptr) {
                pcl::PointCloud<pcl::PointXYZINormal> pc_pcl;
                pcl::fromROSMsg(*pc, pc_pcl);
                for (auto &p: pc_pcl.points) {
                    livox.push_back(p);
                }
            }
        }
        // read velodyne and store it in a deque
        if (m.getTopic() == "/velodyne_static_raw") {
            sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
            if (pc != nullptr) {
                pcl::PointCloud<pcl::PointXYZINormal> pc_pcl;
                pcl::fromROSMsg(*pc, pc_pcl);
                for (auto &p: pc_pcl.points) {
                    velodyne_stationary.push_back(p);
                }
            }
        }
        // read velodyne_rot and store it in a deque
        if (m.getTopic() == "/velodyne_rot") {
            sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
            if (pc != nullptr) {
                pcl::PointCloud<pcl::PointXYZINormal> pc_pcl;
                pcl::fromROSMsg(*pc, pc_pcl);
                for (auto &p: pc_pcl.points) {
                    velodyne_rot_deque.push_back(p);
                    const float timestamp = getTimestamp(p);
                    rotation_angle[timestamp] = angular_encoder_offset - getAngle(p);
                }
            }
        }
        if (m.getTopic() == "/imu/data_hwts") {
            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            if (imu != nullptr) {
                const double timestamp = imu->header.stamp.toSec() - 1600000000.0;
                output_bag.write("/imu", ros::Time(timestamp), *imu);

            }
        }
    }
    std::cout << "velodyne_rot_deque.size(): " << velodyne_rot_deque.size() / 1e6 << std::endl;
    std::cout << "velodyne_stationary.size(): " << velodyne_stationary.size()/ 1e6 << std::endl;
    std::cout << "livox.size(): " << livox.size()/ 1e6 << std::endl;

    std::cout << "velodyne_rot_deque.timestamps : " << getTimestamp(velodyne_rot_deque.front()) << " "
              << getTimestamp(velodyne_rot_deque.back()) << std::endl;
    std::cout << "velodyne_stationary.timestamps : " << getTimestamp(velodyne_stationary.front()) << " "
              << getTimestamp(velodyne_stationary.back()) << std::endl;
    std::cout << "livox.timestamps : " << getTimestamp(livox.front()) << " "
              << getTimestamp(livox.back()) << std::endl;
    const float bagStartTimestamp = getTimestamp(velodyne_rot_deque.front());


    // create static tf
    if (add_static_tf) {
        addStaticTransformTree(output_bag, bagStartTimestamp);
    }
    savePointclouds(output_bag, "/velodyne_rot", "velodyne_rot_base", velodyne_rot_deque, pc_len);
    savePointclouds(output_bag, "/velodyne", "velodyne", velodyne_stationary, pc_len, 2.0f);
    savePointclouds(output_bag, "/livox", "livox", livox, pc_len);

    //create tf_tree for VLP16 rotation
    for (auto &p: rotation_angle) {
        tf2_msgs::TFMessage tf_msgs;
        tf_msgs.transforms.resize(1);
        auto &tf_msg = tf_msgs.transforms[0];
        float timestamp = p.first;
        tf_msg.header.stamp = ros::Time(timestamp);
        tf_msg.header.frame_id = "velodyne_rot_base";
        tf_msg.child_frame_id = "velodyne_rot_angular_offset";
        Eigen::Quaternionf angularOffset(Eigen::AngleAxisf(p.second, Eigen::Vector3f::UnitZ()));
        tf_msg.transform.rotation = toRosMsg(angularOffset);
        output_bag.write("/tf", ros::Time(timestamp - 0.001f), tf_msgs);
    }
    return 0;
}