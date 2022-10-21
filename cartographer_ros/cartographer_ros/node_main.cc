/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h" // absl是谷歌开源出来的一个C++标准库的扩充[参考:https://blog.csdn.net/zhghost/article/details/122613091]
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

// 使用函数google::ParseCommandLineFlags初始化运行参数，需要先通过gflags的宏来定义参数对象(main函数的输入)
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.; // 缓存的时间长度。constexpr验证是否为常量表达式，目的是将运算尽量放在编译阶段，而不是运行阶段
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options; // 传感器配置
  std::tie(node_options, trajectory_options) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename); // tie:创建左值引用的元组tuple

  auto map_builder = cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer, FLAGS_collect_metrics); // std::move将参数转换为右值引用，右值引用可以进行读写操作，减少对象的构造
            
  if (!FLAGS_load_state_filename.empty()) { // 是否加载历史地图
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state); // 加载数据包数据
  }

  if (FLAGS_start_trajectory_with_default_topics) { // 是否以默认topic（从配置中读取）开启trajectory
    node.StartTrajectoryWithDefaultTopics(trajectory_options); // 开始轨迹跟踪
  }

  ::ros::spin();

  node.FinishAllTrajectories(); // 结束
  node.RunFinalOptimization(); // 结束后再做的优化

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename, true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]); // 初始化谷歌的日志系统glog
  google::ParseCommandLineFlags(&argc, &argv, true); // 解析输入

  CHECK(!FLAGS_configuration_directory.empty()) << "-configuration_directory is missing."; // 检查是否在运行参数中指定了配置文件和目录
  CHECK(!FLAGS_configuration_basename.empty()) << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node"); // 开头的::表示全局作用域
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink; // 日志输出机制
  cartographer_ros::Run();
  ::ros::shutdown();
}
