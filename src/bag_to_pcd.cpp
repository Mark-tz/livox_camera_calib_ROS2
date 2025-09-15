#include "livox_ros_driver/msg/custom_msg.hpp"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rmw/types.h>

using namespace std;

string bag_file;
string lidar_topic;
string pcd_file;
bool is_custom_msg;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bag_to_pcd");
  
  // Declare parameters
  node->declare_parameter("bag_file", "");
  node->declare_parameter("pcd_file", "");
  node->declare_parameter("lidar_topic", "/livox/lidar");
  node->declare_parameter("is_custom_msg", false);
  
  // Get parameters
  bag_file = node->get_parameter("bag_file").as_string();
  pcd_file = node->get_parameter("pcd_file").as_string();
  lidar_topic = node->get_parameter("lidar_topic").as_string();
  is_custom_msg = node->get_parameter("is_custom_msg").as_bool();
  
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  std::fstream file_;
  file_.open(bag_file, ios::in);
  if (!file_) {
    std::string msg = "Loading the rosbag " + bag_file + " failure";
    RCLCPP_ERROR_STREAM(node->get_logger(), msg.c_str());
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Loading the rosbag %s", bag_file.c_str());
  
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_file;
  storage_options.storage_id = "sqlite3";
  
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  
  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "LOADING BAG FAILED: " << e.what());
    return -1;
  }
  
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    
    if (bag_message->topic_name == lidar_topic) {
      if (is_custom_msg) {
        // 处理 Livox 自定义消息
        rclcpp::Serialization<livox_ros_driver::msg::CustomMsg> serialization;
        livox_ros_driver::msg::CustomMsg livox_cloud_msg;
        
        // 创建 SerializedMessage 对象
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(bag_message->serialized_data->buffer_length);
        serialized_msg.get_rcl_serialized_message().buffer_length = bag_message->serialized_data->buffer_length;
        serialized_msg.get_rcl_serialized_message().buffer_capacity = bag_message->serialized_data->buffer_capacity;
        serialized_msg.get_rcl_serialized_message().buffer = bag_message->serialized_data->buffer;
        
        try {
          serialization.deserialize_message(&serialized_msg, &livox_cloud_msg);
          
          for (uint32_t i = 0; i < livox_cloud_msg.point_num; ++i) {
            pcl::PointXYZI p;
            p.x = livox_cloud_msg.points[i].x;
            p.y = livox_cloud_msg.points[i].y;
            p.z = livox_cloud_msg.points[i].z;
            p.intensity = livox_cloud_msg.points[i].reflectivity;
            output_cloud.points.push_back(p);
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to deserialize CustomMsg: " << e.what());
          continue;
        }
      } else {
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        sensor_msgs::msg::PointCloud2 livox_cloud;
        
        // 创建 SerializedMessage 对象
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(bag_message->serialized_data->buffer_length);
        serialized_msg.get_rcl_serialized_message().buffer_length = bag_message->serialized_data->buffer_length;
        serialized_msg.get_rcl_serialized_message().buffer_capacity = bag_message->serialized_data->buffer_capacity;
        serialized_msg.get_rcl_serialized_message().buffer = bag_message->serialized_data->buffer;
        
        try {
          serialization.deserialize_message(&serialized_msg, &livox_cloud);
          
          pcl::PointCloud<pcl::PointXYZI> cloud;
          pcl::PCLPointCloud2 pcl_pc;
          pcl_conversions::toPCL(livox_cloud, pcl_pc);
          pcl::fromPCLPointCloud2(pcl_pc, cloud);
          for (size_t i = 0; i < cloud.size(); ++i) {
            output_cloud.points.push_back(cloud.points[i]);
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to deserialize PointCloud2: " << e.what());
          continue;
        }
      }
    }
  }
  
  output_cloud.is_dense = false;
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  
  if (output_cloud.points.empty()) {
    RCLCPP_WARN(node->get_logger(), "No points found in the bag file!");
    return -1;
  }
  
  pcl::io::savePCDFileASCII(pcd_file, output_cloud);
  string msg = "Successfully save point cloud to pcd file: " + pcd_file;
  RCLCPP_INFO_STREAM(node->get_logger(), msg.c_str());
  RCLCPP_INFO(node->get_logger(), "Total points saved: %zu", output_cloud.points.size());
  
  rclcpp::shutdown();
  return 0;
}