#include "cblox_ros/tsdf_submap_server.h"

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <minkindr_conversions/kindr_msg.h>

#include <voxblox/utils/timing.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/conversions_inl.h>

#include "cblox/io/tsdf_submap_io.h"
#include "cblox_ros/pointcloud_conversions.h"
#include "cblox_ros/pose_vis.h"
#include "cblox_ros/ros_params.h"

namespace cblox {

TsdfSubmapServer::TsdfSubmapServer(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : TsdfSubmapServer(
          nh, nh_private, voxblox::getTsdfMapConfigFromRosParam(nh_private),
          voxblox::getTsdfIntegratorConfigFromRosParam(nh_private),
          getTsdfIntegratorTypeFromRosParam(nh_private),
          voxblox::getMeshIntegratorConfigFromRosParam(nh_private)) {}

TsdfSubmapServer::TsdfSubmapServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const TsdfMap::Config& tsdf_map_config,
    const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config,
    const voxblox::TsdfIntegratorType& tsdf_integrator_type,
    const voxblox::MeshIntegratorConfig& mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      num_integrated_frames_current_submap_(0),
      num_integrated_frames_per_submap_(kDefaultNumFramesPerSubmap),
      color_map_(new voxblox::GrayscaleColorMap()),
      transformer_(nh, nh_private) {
  ROS_DEBUG("Creating a TSDF Server");

  // Initial interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();

  // Creating the submap collection
  tsdf_submap_collection_ptr_.reset(
      new SubmapCollection<TsdfSubmap>(tsdf_map_config));

  // Creating an integrator and targetting the collection
  tsdf_submap_collection_integrator_ptr_.reset(
      new TsdfSubmapCollectionIntegrator(tsdf_integrator_config,
                                         tsdf_integrator_type,
                                         tsdf_submap_collection_ptr_));

  // An object to visualize the submaps
  submap_mesher_ptr_.reset(new SubmapMesher(tsdf_map_config, mesh_config));
  active_submap_visualizer_ptr_.reset(
      new ActiveSubmapVisualizer(mesh_config, tsdf_submap_collection_ptr_));

  // An object to visualize the trajectory
  trajectory_visualizer_ptr_.reset(new TrajectoryVisualizer);

  last_msg_time_ptcloud_ = ros::Time::now();

  pointcloud_with_tf_buffer_.reset(num_integrated_frames_per_submap_*2);

  start_map_service_req_ = false;
  mapping_cmd_ = false;
}

void TsdfSubmapServer::subscribeToTopics() {
  // Subscribing to the input pointcloud
  int pointcloud_queue_size = kDefaultPointcloudQueueSize;
  nh_private_.param("pointcloud_queue_size", pointcloud_queue_size,
                    pointcloud_queue_size);
  pointcloud_sub_ = nh_.subscribe("pointcloud", pointcloud_queue_size*10,
                                  &TsdfSubmapServer::pointcloudCallback, this);
  pointcloud2_sub_ = nh_.subscribe("pointcloud2", pointcloud_queue_size*10,
                                  &TsdfSubmapServer::pointcloud2Callback, this);
}

void TsdfSubmapServer::advertiseTopics() {
  // Services for saving meshes to file
  generate_separated_mesh_srv_ = nh_private_.advertiseService(
      "generate_separated_mesh",
      &TsdfSubmapServer::generateSeparatedMeshCallback, this);
  generate_combined_mesh_srv_ = nh_private_.advertiseService(
      "generate_combined_mesh", &TsdfSubmapServer::generateCombinedMeshCallback,
      this);
  // Services for loading and saving
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &TsdfSubmapServer::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &TsdfSubmapServer::loadMapCallback, this);

  // Service to start or stop generating map.
  start_map_srv_ = nh_private_.advertiseService(
      "start_map", &TsdfSubmapServer::startMapCallback, this);

  // Real-time publishing for rviz
  active_submap_mesh_pub_ =
      nh_private_.advertise<visualization_msgs::Marker>("separated_mesh", 1);
  submap_poses_pub_ =
      nh_private_.advertise<geometry_msgs::PoseArray>("submap_baseframes", 1);
  trajectory_pub_ = nh_private_.advertise<nav_msgs::Path>("trajectory", 1);
  tsdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("tsdf_map_out", 1);
}

void TsdfSubmapServer::getParametersFromRos() {
  ROS_DEBUG("Getting params from ROS");
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("world_frame", world_frame_, world_frame_);
  // Throttle frame integration
  double min_time_between_msgs_sec = 0.0;
  nh_private_.param("min_time_between_msgs_sec", min_time_between_msgs_sec,
                    min_time_between_msgs_sec);
  min_time_between_msgs_.fromSec(min_time_between_msgs_sec);
  nh_private_.param("mesh_filename", mesh_filename_, mesh_filename_);
  // Timed updates for submap mesh publishing.
  double update_mesh_every_n_sec = 0.0;
  nh_private_.param("update_mesh_every_n_sec", update_mesh_every_n_sec,
                    update_mesh_every_n_sec);
  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
                                &TsdfSubmapServer::updateMeshEvent, this);
  }
  // Frequency of submap creation
  nh_private_.param("num_integrated_frames_per_submap",
                    num_integrated_frames_per_submap_,
                    num_integrated_frames_per_submap_);
}


bool TsdfSubmapServer::startMapCallback(std_srvs::SetBool::Request& request,
                                       std_srvs::SetBool::Response& response) {
  start_map_service_req_ = true;
  mapping_cmd_ = request.data;
  response.success = true;
  response.message = "Received the cmd";
  ROS_INFO("Submap service requested: %s", mapping_cmd_?"Start mapping...":"Stop mapping...");
  return true;
}


void TsdfSubmapServer::pointcloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  addMesageToPointcloudQueue(pointcloud_msg_in);
  // Processing messages in the queue
  servicePointcloudQueue();
}

void TsdfSubmapServer::pointcloud2Callback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  pointcloud2_queue_.push(pointcloud_msg_in);
  addMesageToPointcloudWithTFQueue2();
}

void TsdfSubmapServer::addMesageToPointcloudQueue(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
  // Pushing this message onto the queue for processing
  if (pointcloud_msg_in->header.stamp - last_msg_time_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_ptcloud_ = pointcloud_msg_in->header.stamp;
    pointcloud_queue_.push(pointcloud_msg_in);
  }
  addMesageToPointcloudWithTFQueue();
}


void TsdfSubmapServer::addMesageToPointcloudWithTFQueue2() {
  const size_t kMaxQueueSize = 10;
  static Transformation last_T_G_C;

  int max_queue_size_ = 60;
  double min_tranlation_between_msgs_ = 0.1; //m
  double min_rotation_between_msgs_ = M_PI / 18; //10 deg

  if (pointcloud2_queue_.empty()) return;
  bool cont = true;
  while(cont) {
    sensor_msgs::PointCloud2::Ptr pointcloud_msg;
    if (!pointcloud2_queue_.empty()) pointcloud_msg = pointcloud2_queue_.front();
    else break;
    Transformation current_T_G_C;
    if (transformer_.lookupTransform(pointcloud_msg->header.frame_id,
                                   world_frame_,
                                   pointcloud_msg->header.stamp, &current_T_G_C)) {
      pointcloud2_queue_.pop();
      Transformation delta_pose = current_T_G_C * last_T_G_C.inverse();
      double d_translation = delta_pose.getPosition().norm();
      // ROS_INFO("Translation: %f ", d_translation);
      //double d_rotation = delta_pose.getRotation().getAngleShortestPath();
      double d_rotation = M_PI;
      if ((d_translation > min_tranlation_between_msgs_) && (d_rotation > min_rotation_between_msgs_)) {
        // pointcloud_with_tf_queue_.push(PointCloudWithTF(pointcloud_msg, current_T_G_C));
        std::lock_guard<std::mutex> guard(pointcloud_with_tf_buffer_mutex_);
        pointcloud_with_tf_buffer_.writeData(PointCloudWithTF(pointcloud_msg, current_T_G_C), true);
        last_T_G_C = current_T_G_C;
        // ROS_INFO("Added %d msgs", pointcloud_with_tf_buffer_.size());
      }
    } else {
      cont = false;
    }
  }

  if (pointcloud2_queue_.size() >= kMaxQueueSize) {
    ROS_ERROR_THROTTLE(60,
                        "Input pointcloud queue getting too long! Dropping "
                        "some pointclouds. Either unable to look up transform "
                        "timestamps or the processing is taking too long.");
    while (pointcloud2_queue_.size() >= kMaxQueueSize) {
      pointcloud2_queue_.pop();
    }
  }

  // if (pointcloud_with_tf_buffer_.size() > max_queue_size_) {
  //   ROS_ERROR_THROTTLE(60,
  //                       "Input pointcloud queue getting too long! Dropping "
  //                       "some pointclouds. Either unable to look up transform "
  //                       "timestamps or the processing is taking too long.");
  //   while (pointcloud_with_tf_buffer_.size() > max_queue_size_) {
  //     pointcloud_with_tf_buffer_.pop();
  //   }
  // }
}

void TsdfSubmapServer::addMesageToPointcloudWithTFQueue() {
  const size_t kMaxQueueSize = 10;

  int max_queue_size_ = 60;
  double min_tranlation_between_msgs_ = 0.1; //m
  double min_rotation_between_msgs_ = M_PI / 18; //10 deg

  if (pointcloud_queue_.empty()) return;
  bool cont = true;
  while(cont) {
    sensor_msgs::PointCloud2::Ptr pointcloud_msg;
    if (!pointcloud_queue_.empty()) pointcloud_msg = pointcloud_queue_.front();
    else break;
    Transformation current_T_G_C;
    if (transformer_.lookupTransform(pointcloud_msg->header.frame_id,
                                   world_frame_,
                                   pointcloud_msg->header.stamp, &current_T_G_C)) {
      pointcloud_queue_.pop();
      Transformation delta_pose = current_T_G_C * last_T_G_C_.inverse();
      double d_translation = delta_pose.getPosition().norm();
      // ROS_INFO("Translation: %f ", d_translation);
      //double d_rotation = delta_pose.getRotation().getAngleShortestPath();
      double d_rotation = M_PI;
      if ((d_translation > min_tranlation_between_msgs_) && (d_rotation > min_rotation_between_msgs_)) {
        // pointcloud_with_tf_queue_.push(PointCloudWithTF(pointcloud_msg, current_T_G_C));
        std::lock_guard<std::mutex> guard(pointcloud_with_tf_buffer_mutex_);
        pointcloud_with_tf_buffer_.writeData(PointCloudWithTF(pointcloud_msg, current_T_G_C), true);
        last_T_G_C_ = current_T_G_C;
        // ROS_INFO("Added %d msgs", pointcloud_with_tf_buffer_.size());
      }
    } else {
      cont = false;
    }
  }

  if (pointcloud_queue_.size() >= kMaxQueueSize) {
    ROS_ERROR_THROTTLE(60,
                        "Input pointcloud queue getting too long! Dropping "
                        "some pointclouds. Either unable to look up transform "
                        "timestamps or the processing is taking too long.");
    while (pointcloud_queue_.size() >= kMaxQueueSize) {
      pointcloud_queue_.pop();
    }
  }

  // if (pointcloud_with_tf_buffer_.size() > max_queue_size_) {
  //   ROS_ERROR_THROTTLE(60,
  //                       "Input pointcloud queue getting too long! Dropping "
  //                       "some pointclouds. Either unable to look up transform "
  //                       "timestamps or the processing is taking too long.");
  //   while (pointcloud_with_tf_buffer_.size() > max_queue_size_) {
  //     pointcloud_with_tf_buffer_.pop();
  //   }
  // }
}

void TsdfSubmapServer::servicePointcloudQueue() {
  // NOTE(alexmilane): T_G_C - Transformation between Camera frame (C) and
  //                           global tracking frame (G).

  if(start_map_service_req_) {
    ros::WallTime start = ros::WallTime::now();
    start_map_service_req_ = false; // clear the service request for next time.
    if(mapping_cmd_) {
      // Start mapping: build an initial map with N pointclouds from the buffer.
      // Get latest pose from the buffer.
      Transformation latest_T_G_C;
      std::vector<PointCloudWithTF> data_hist;
      pointcloud_with_tf_buffer_.readDataFromHistory(pointcloud_with_tf_buffer_.getWriteInd(), num_integrated_frames_per_submap_, data_hist);
      // latest_T_G_C = data_hist[num_integrated_frames_per_submap_-1].T_G_C;
      resetSubmap(latest_T_G_C);
      for (size_t l = 1; l < data_hist.size(); ++l) {
        if (!data_hist[l].is_empty)
          processPointCloudMessageAndInsert(data_hist[l].point_cloud, data_hist[l].T_G_C, false);
      }
      pointcloud_with_tf_buffer_.cleanReadInd();
      ROS_INFO("Map intialized from the buffer in %f(sec)", (ros::WallTime::now() - start).toSec());
    } else {
      // Stop mapping
      tsdf_submap_collection_ptr_->clear();
      ROS_INFO("Map stopped");
    }
    // Reset map.
    if (tsdf_map_pub_.getNumSubscribers()) {
      voxblox_msgs::Layer layer_msg;
      voxblox::serializeLayerAsMsg<TsdfVoxel>(tsdf_submap_collection_ptr_->getProjectedMap()->getTsdfLayer(),
                                    false, &layer_msg);
      layer_msg.action = static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
      tsdf_map_pub_.publish(layer_msg);
    }
  }

  static int count_to_trigger = 0;
  if(mapping_cmd_) {
    Transformation T_G_C;
    sensor_msgs::PointCloud2::Ptr pointcloud_msg;
    bool reset_submap = false;
    while (
        getNextPointcloudFromBuffer(&pointcloud_with_tf_buffer_, &pointcloud_msg, &T_G_C)) {
      constexpr bool is_freespace_pointcloud = false;
      processPointCloudMessageAndInsert(pointcloud_msg, T_G_C,
                                        is_freespace_pointcloud);
      ++count_to_trigger;
    }

    if (tsdf_map_pub_.getNumSubscribers() && (count_to_trigger > 20)) {
      ros::WallTime start = ros::WallTime::now();
      voxblox_msgs::Layer layer_msg;
      constexpr bool only_updated = true;
      voxblox::serializeLayerAsMsg<TsdfVoxel>(tsdf_submap_collection_ptr_->getActiveTsdfMapPtr()->getTsdfLayer(),  //getProjectedMap()
                                    only_updated, &layer_msg);
      tsdf_map_pub_.publish(layer_msg);
      count_to_trigger = 0;
      ROS_ERROR("Publishing new map costs %f (sec)", (ros::WallTime::now() - start).toSec());
    }
  }
}

bool TsdfSubmapServer::getNextPointcloudFromQueue(
    std::queue<PointCloudWithTF>* queue,
    sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C) {
  const size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front().point_cloud;
  *T_G_C = queue->front().T_G_C;
  queue->pop();
  return true;
}

bool TsdfSubmapServer::getNextPointcloudFromBuffer(
    PointCloudWithTFRingBuffer<PointCloudWithTF>* buffer,
    sensor_msgs::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C) {
  if (buffer->empty()) {
    return false;
  }
  PointCloudWithTF data;
  buffer->readData(data);
  *pointcloud_msg = data.point_cloud;
  *T_G_C = data.T_G_C;
  return true;
}

void TsdfSubmapServer::processPointCloudMessageAndInsert(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  // Convert the PCL pointcloud into our awesome format.
  Pointcloud points_C;
  Colors colors;
  convertPointcloudMsg(*color_map_, pointcloud_msg, &points_C, &colors);

  if (verbose_) {
    ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
  }

  if (!mapIntialized()) {
    ROS_INFO("Intializing map.");
    intializeMap(T_G_C);
  }

  ros::WallTime start = ros::WallTime::now();
  integratePointcloud(T_G_C, points_C, colors, is_freespace_pointcloud);
  ros::WallTime end = ros::WallTime::now();
  num_integrated_frames_current_submap_++;
  if (verbose_) {
    ROS_INFO(
        "Finished integrating in %f seconds, have %lu blocks. %u frames "
        "integrated to current submap.",
        (end - start).toSec(), tsdf_submap_collection_ptr_->getActiveTsdfMap()
                                   .getTsdfLayer()
                                   .getNumberOfAllocatedBlocks(),
        num_integrated_frames_current_submap_);
  }
}

void TsdfSubmapServer::integratePointcloud(const Transformation& T_G_C,
                                           const Pointcloud& ptcloud_C,
                                           const Colors& colors,
                                           const bool is_freespace_pointcloud) {
  // Note(alexmillane): Freespace pointcloud option left out for now.
  CHECK_EQ(ptcloud_C.size(), colors.size());
  tsdf_submap_collection_integrator_ptr_->integratePointCloud(T_G_C, ptcloud_C,
                                                              colors);
}

void TsdfSubmapServer::intializeMap(const Transformation& T_G_C) {
  // Just creates the first submap
  createNewSubmap(T_G_C);
}

bool TsdfSubmapServer::newSubmapRequired() const {
  return (num_integrated_frames_current_submap_ >
          num_integrated_frames_per_submap_);
}

void TsdfSubmapServer::resetSubmap(const Transformation& T_G_C) {
  // delete the previous submaps.
  tsdf_submap_collection_ptr_->clear();
  intializeMap(T_G_C);
}

void TsdfSubmapServer::createNewSubmap(const Transformation& T_G_C) {
  // Creating the submap
  const SubmapID submap_id =
      tsdf_submap_collection_ptr_->createNewSubmap(T_G_C);
  // Activating the submap in the frame integrator
  tsdf_submap_collection_integrator_ptr_->switchToActiveSubmap();
  // Resetting current submap counters
  num_integrated_frames_current_submap_ = 0;

  // Updating the active submap mesher
  active_submap_visualizer_ptr_->switchToActiveSubmap();

  // Publish the baseframes
  visualizeSubmapBaseframes();

  if (verbose_) {
    ROS_INFO_STREAM("Created a new submap with id: "
                    << submap_id << ". Total submap number: "
                    << tsdf_submap_collection_ptr_->size());
  }
}

void TsdfSubmapServer::visualizeActiveSubmapMesh() {
  // NOTE(alexmillane): For the time being only the mesh from the currently
  // active submap is updated. This breaks down when the pose of past submaps is
  // changed. We will need to handle this separately later.
  active_submap_visualizer_ptr_->updateMeshLayer();
  // Getting the display mesh
  visualization_msgs::Marker marker;
  active_submap_visualizer_ptr_->getDisplayMesh(&marker);
  marker.header.frame_id = world_frame_;
  // Publishing
  active_submap_mesh_pub_.publish(marker);
}

void TsdfSubmapServer::visualizeWholeMap() {
  // Looping through the whole map, meshing and publishing.
  for (const SubmapID submap_id : tsdf_submap_collection_ptr_->getIDs()) {
    tsdf_submap_collection_ptr_->activateSubmap(submap_id);
    active_submap_visualizer_ptr_->switchToActiveSubmap();
    visualizeActiveSubmapMesh();
  }
}

bool TsdfSubmapServer::generateSeparatedMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& /*response*/) {  // NO LINT
  // Saving mesh to file if required
  if (!mesh_filename_.empty()) {
    // Getting the requested mesh type from the mesher
    voxblox::MeshLayer seperated_mesh_layer(
        tsdf_submap_collection_ptr_->block_size());
    submap_mesher_ptr_->generateSeparatedMesh(*tsdf_submap_collection_ptr_,
                                              &seperated_mesh_layer);
    bool success = outputMeshLayerAsPly(mesh_filename_, seperated_mesh_layer);
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
      return true;
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  } else {
    ROS_ERROR("No path to mesh specified in ros_params.");
  }
  return false;
}

bool TsdfSubmapServer::generateCombinedMeshCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& /*response*/) {  // NO LINT
  // Saving mesh to file if required
  if (!mesh_filename_.empty()) {
    // Getting the requested mesh type from the mesher
    voxblox::MeshLayer combined_mesh_layer(
        tsdf_submap_collection_ptr_->block_size());
    submap_mesher_ptr_->generateCombinedMesh(*tsdf_submap_collection_ptr_,
                                             &combined_mesh_layer);
    bool success = outputMeshLayerAsPly(mesh_filename_, combined_mesh_layer);
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
      return true;
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  } else {
    ROS_ERROR("No path to mesh specified in ros_params.");
  }
  return false;
}

void TsdfSubmapServer::updateMeshEvent(const ros::TimerEvent& /*event*/) {
  if (mapIntialized()) {
    visualizeActiveSubmapMesh();
  }
}

void TsdfSubmapServer::visualizeSubmapBaseframes() const {
  // Get poses
  TransformationVector submap_poses;
  tsdf_submap_collection_ptr_->getSubmapPoses(&submap_poses);
  // Transform to message
  geometry_msgs::PoseArray pose_array_msg;
  posesToMsg(submap_poses, &pose_array_msg);
  pose_array_msg.header.frame_id = world_frame_;
  // Publish
  submap_poses_pub_.publish(pose_array_msg);
}

void TsdfSubmapServer::visualizeTrajectory() const {
  nav_msgs::Path path_msg;
  trajectory_visualizer_ptr_->getTrajectoryMsg(&path_msg);
  path_msg.header.frame_id = world_frame_;
  trajectory_pub_.publish(path_msg);
}

bool TsdfSubmapServer::saveMap(const std::string& file_path) {
  return cblox::io::SaveTsdfSubmapCollection(*tsdf_submap_collection_ptr_,
                                             file_path);
}
bool TsdfSubmapServer::loadMap(const std::string& file_path) {
  bool success = io::LoadSubmapCollection<TsdfSubmap>(
      file_path, &tsdf_submap_collection_ptr_);
  if (success) {
    ROS_INFO("Successfully loaded TSDFSubmapCollection.");
    constexpr bool kVisualizeMapOnLoad = true;
    if (kVisualizeMapOnLoad) {
      ROS_INFO("Publishing loaded map's mesh.");
      visualizeWholeMap();
    }
  }
  return success;
}

bool TsdfSubmapServer::saveMapCallback(voxblox_msgs::FilePath::Request& request,
                                       voxblox_msgs::FilePath::Response&
                                       /*response*/) {
  return saveMap(request.file_path);
}

bool TsdfSubmapServer::loadMapCallback(voxblox_msgs::FilePath::Request& request,
                                       voxblox_msgs::FilePath::Response&
                                       /*response*/) {
  bool success = loadMap(request.file_path);
  return success;
}

}  // namespace cblox
