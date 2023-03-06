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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

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
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  // tf_buffer只保留最近10s内的transform
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  //节点的参数选项
  NodeOptions node_options;
  //轨迹的参数选项
  TrajectoryOptions trajectory_options;
  // tie把node_options和trajection_options组合成一个元组，通常用在函数多返回值
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
  // MapBuilder类是完整的SLAM算法类
  // 包含前端(TrajectoryBuilders,scan to submap) 与 后端 (检测回环与PoseGraph) 
  // 创建时初始化了pose_graph,初始化了sensor_collator
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  // ros里的node节点，将ROS的数据传入MapBuilder，其中定义了各种topic和service
  // FLAGS_collect_metrics, 收集一些metrics
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);

  // 是否加载已有地图，.pbstream
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }
  // 是否使用默认话题开始一条轨迹
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::spin();
  // 结束所有处于活动状态的轨迹
  node.FinishAllTrajectories();
  // 当所有的轨迹结束时, 再执行一次全局优化
  node.RunFinalOptimization();
  
  // 如果save_state_filename非空, 就保存pbstream文件
  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // 初始化glog
  google::InitGoogleLogging(argv[0]);
  // 解析命令行参数,
  // remove_flag第三个参数
  // 如果设为true，则该函数处理完成后，argv中只保留argv[0]，argc会被设置为1。
  // 如果为false，则argv和argc会被保留，但是注意函数会调整argv中的顺序。
  google::ParseCommandLineFlags(&argc, &argv, true);

  // glog里的CHECK系列的宏,如果非真就会打印后面的description和栈上的信息并退出程序
  // 
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  // ros节点初始化
  ::ros::init(argc, argv, "cartographer_node");
  // 一般不需要显式调用
  // 但是若想在创建任何NodeHandle实例之前启动ROS相关的线程, 网络等, 可以显式调用该函数.
  ::ros::start();
  // 使用ROS_INFO进行glog消息的输出
  cartographer_ros::ScopedRosLogSink ros_log_sink;

  // 启动cartographer_ros
  cartographer_ros::Run();
  ::ros::shutdown();
}
