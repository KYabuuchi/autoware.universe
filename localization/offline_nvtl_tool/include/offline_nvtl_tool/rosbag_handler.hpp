#pragma once
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>

template <typename T>
T decode_with_type(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message)
{
  T msg;
  rclcpp::Serialization<T> serialization;
  rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
  serialization.deserialize_message(&extracted_serialized_msg, &msg);
  return msg;
}

class RosbagReader
{
public:
  RosbagReader(const std::string & input_bag_path)
  {
    const rosbag2_storage::StorageOptions storage_options{input_bag_path, "sqlite3"};
    const rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};

    reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader_->open(storage_options, converter_options);
  }

  bool has_next() const { return reader_->has_next(); }
  auto read_next() const { return reader_->read_next(); }

private:
  std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
};