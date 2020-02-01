#ifndef CBLOX_ROS_TSDF_SUBMAP_SERVER_H_
#define CBLOX_ROS_TSDF_SUBMAP_SERVER_H_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <voxblox/utils/color_maps.h>
#include <voxblox_ros/transformer.h>
#include <voxblox_msgs/FilePath.h>

#include <cblox/core/common.h>
#include <cblox/core/submap_collection.h>
#include <cblox/core/tsdf_submap.h>
#include <cblox/integrator/tsdf_submap_collection_integrator.h>
#include <cblox/mesh/submap_mesher.h>

#include "cblox_ros/active_submap_visualizer.h"
#include "cblox_ros/trajectory_visualizer.h"

namespace cblox {

// Default values for parameters
constexpr bool kDefaultVerbose = true;
constexpr int kDefaultNumFramesPerSubmap = 20;
constexpr double kDefaultMinTimeBetweenMsgsSec = 0.0;

// Data queue sizes
constexpr int kDefaultPointcloudQueueSize = 1;

struct PointCloudWithTF{
  PointCloudWithTF(): is_empty(true) {}
  PointCloudWithTF(sensor_msgs::PointCloud2::Ptr point_cloud_in, Transformation &T_G_C_in) {
    point_cloud = point_cloud_in;
    T_G_C = T_G_C_in;
    is_empty = false;
  }
  sensor_msgs::PointCloud2::Ptr point_cloud;
  Transformation T_G_C;
  bool is_empty;
  PointCloudWithTF& operator=(const PointCloudWithTF& other) {
    if (this != &other) {
      point_cloud = other.point_cloud;
      T_G_C = other.T_G_C;
    }
    return *this;
  }
};

template<class DataType>
class PointCloudWithTFRingBuffer {
 public:
  PointCloudWithTFRingBuffer(): buffer_size_(0) {}
  PointCloudWithTFRingBuffer(int buffer_size) {
    reset(buffer_size);
  }

  ~PointCloudWithTFRingBuffer() {
    buffer_.clear();
  }

  void reset(int buffer_size) {
    clear();
    if (buffer_size <= 0) {
      ROS_ERROR("Buffer size must be positive");
      return;
    }
    buffer_size_ = buffer_size;
    buffer_.resize(buffer_size_);
  }

  void clear() {
    buffer_size_ = 0;
    read_ind_ = 0;
    write_ind_ = 0;
    read_count_ = 0;
    write_count_ = 0;
    buffer_.clear();
  }

  size_t size() {
    if (write_ind_ >= read_ind_) return (write_ind_ - read_ind_);
    else return (write_ind_ - read_ind_ + buffer_size_);
  }

  void pop() {
    if (!empty()) increaseInd(read_ind_);
  }

  bool full() {
    return ((buffer_size_) && ((write_ind_ + 1) % (buffer_size_) == read_ind_));
  }

  bool empty() {
    return (read_ind_ == write_ind_);
  }

  int getRingSize() {
    return buffer_size_;
  }

  bool writeData(const DataType &data, bool write_even_fulled = false) {
    if (full() && !write_even_fulled) {
      if (write_even_fulled) {
        // Force to increase the read ind.
        increaseInd(read_ind_);
      } else {
        return false;
      }
    }
    writeDataToBuffer(write_ind_, data);
    increaseInd(write_ind_);
    return true;
  }

  bool readData(DataType &data) {
    if (empty()) return false;
    readDataFromBuffer(read_ind_, data);
    increaseInd(read_ind_);
  }

  bool readDataFromHistory(const int start_ind, const int len, std::vector<DataType> &data) {
    /* Be careful with this */
    if ((len < 0) || (len > buffer_size_)) return false;
    if ((start_ind < 0) || (start_ind > buffer_size_)) return false;
    int read_ind = start_ind;
    shiftInd(read_ind, len);
    for (int i = 0; i < len; ++i) {
      data.push_back(buffer_[read_ind]);
      increaseInd(read_ind);
    }
  }

 private:
  inline void increaseInd(int &ind) {
    ind = (ind + 1) % buffer_size_;
  }

  inline void shiftInd(int &ind, const int steps) {
    if (steps >= 0) {
      // Shift forward
      ind = (ind + steps) % buffer_size_;
    } else {
      // Shift backward
      int steps_tmp = steps;
      while (steps_tmp > buffer_size_) steps_tmp -= buffer_size_;
      ind -= steps_tmp;
      if (ind < 0) ind += buffer_size_;
    }
  }

  inline void writeDataToBuffer(const int ind, const DataType &data) {
    buffer_[ind] = data;
    ++write_count_;
  }

  inline void readDataFromBuffer(const int ind, DataType &data) {
    data = buffer_[ind];
    ++read_count_;
  }

  int buffer_size_;
  int read_ind_;
  int write_ind_;
  int write_count_;
  int read_count_;
  std::vector<DataType> buffer_;
};

// Receives ROS Data and produces a collection of submaps
class TsdfSubmapServer {
 public:
  // Constructor
  TsdfSubmapServer(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  TsdfSubmapServer(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      const TsdfMap::Config& tsdf_map_config,
      const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
      const voxblox::TsdfIntegratorType& tsdf_integrator_type,
      const voxblox::MeshIntegratorConfig& mesh_config);
  virtual ~TsdfSubmapServer() {}

  // Pointcloud data subscriber
  virtual void pointcloudCallback(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);

  // Saving and Loading callbacks
  bool saveMap(const std::string& file_path);
  bool loadMap(const std::string& file_path);
  bool saveMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT

  // Update the mesh and publish for visualization
  void updateMeshEvent(const ros::TimerEvent& /*event*/);
  void visualizeActiveSubmapMesh();
  void visualizeWholeMap();

  // Visualize trajectory
  void visualizeSubmapBaseframes() const;
  void visualizeTrajectory() const;

  // Mesh output
  bool generateSeparatedMeshCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
  bool generateCombinedMeshCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

 private:
  // Gets parameters
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // The two actions on pointcloud callback; add message to queue for
  // processing, and process messages in the queue.
  void addMesageToPointcloudQueue(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in);
  void addMesageToPointcloudWithTFQueue();
  void servicePointcloudQueue();

  // Checks if we can get the next message from queue.
  bool getNextPointcloudFromQueue(
    std::queue<PointCloudWithTF>* queue,
    sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C);

  bool getNextPointcloudFromBuffer(
    PointCloudWithTFRingBuffer<PointCloudWithTF>* buffer,
    sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C);

  // Pointcloud integration
  void processPointCloudMessageAndInsert(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
      const Transformation& T_G_C, const bool is_freespace_pointcloud);
  void integratePointcloud(const Transformation& T_G_C,
                           const Pointcloud& ptcloud_C, const Colors& colors,
                           const bool is_freespace_pointcloud);

  // Initializes the map
  bool mapIntialized() const { return !tsdf_submap_collection_ptr_->empty(); }
  void intializeMap(const Transformation& T_G_C);

  // Submap creation
  bool newSubmapRequired() const;
  void createNewSubmap(const Transformation& T_G_C);
  void resetSubmap(const Transformation& T_G_C);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber pointcloud_sub_;

  // Publishers
  ros::Publisher active_submap_mesh_pub_;
  ros::Publisher submap_poses_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher tsdf_map_pub_;

  // Services
  ros::ServiceServer generate_separated_mesh_srv_;
  ros::ServiceServer generate_combined_mesh_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;

  // Timers.
  ros::Timer update_mesh_timer_;

  bool verbose_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  std::string world_frame_;

  // The submap collection
  std::shared_ptr<SubmapCollection<TsdfSubmap>> tsdf_submap_collection_ptr_;

  // The integrator
  std::shared_ptr<TsdfSubmapCollectionIntegrator>
      tsdf_submap_collection_integrator_ptr_;

  // For meshing the entire collection to file
  std::shared_ptr<SubmapMesher> submap_mesher_ptr_;
  std::string mesh_filename_;

  // For meshing the active layer
  std::shared_ptr<ActiveSubmapVisualizer> active_submap_visualizer_ptr_;

  // For visualizing the trajectory
  std::shared_ptr<TrajectoryVisualizer> trajectory_visualizer_ptr_;

  // Transformer object to keep track of either TF transforms or messages from a
  // transform topic.
  voxblox::Transformer transformer_;

  // The queue of unprocessed pointclouds
  std::queue<sensor_msgs::PointCloud2::Ptr> pointcloud_queue_;
  std::queue<PointCloudWithTF> pointcloud_with_tf_queue_;

  PointCloudWithTFRingBuffer<PointCloudWithTF> pointcloud_with_tf_buffer_;

  // Last message times for throttling input.
  ros::Duration min_time_between_msgs_;
  ros::Time last_msg_time_ptcloud_;

  //
  Transformation last_T_G_C_;


  /// Colormap to use for intensity pointclouds.
  std::unique_ptr<voxblox::ColorMap> color_map_;

  // Number of frames integrated to the current submap
  int num_integrated_frames_current_submap_;
  // The number of frames integrated into a submap before requesting a new one.
  int num_integrated_frames_per_submap_;
};

}  // namespace cblox

#endif  // CBLOX_ROS_TSDF_SUBMAP_SERVER_H_
