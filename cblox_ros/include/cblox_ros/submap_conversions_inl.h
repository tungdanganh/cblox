#ifndef CBLOX_ROS_SUBMAP_CONVERSIONS_INL_H
#define CBLOX_ROS_SUBMAP_CONVERSIONS_INL_H

namespace cblox {

template <typename SubmapType>
std_msgs::Header generateHeaderMsg(
    const typename SubmapType::Ptr& submap_ptr, const ros::Time &timestamp) {
  std_msgs::Header msg_header;
  msg_header.frame_id = "submap_" + std::to_string(submap_ptr->getID());
  msg_header.stamp = timestamp;
  return msg_header;
}

template <typename SubmapType>
cblox_msgs::MapHeader generateSubmapHeaderMsg(
    const typename SubmapType::Ptr& submap_ptr) {
    // Set the submap ID and type
    cblox_msgs::MapHeader submap_header;
    submap_header.id = submap_ptr->getID();
    submap_header.is_submap = true;

    // Set the submap's start and end time
    submap_header.start_time = submap_ptr->getMappingInterval().first;
    submap_header.end_time = submap_ptr->getMappingInterval().second;

    // Set the pose estimate and indicate what frame it's in
    // TODO(victorr): Get the world frame name from FrameNames once implemented
    submap_header.pose_estimate.frame_id =
        "submap_" + std::to_string(submap_ptr->getID());
    tf::poseKindrToMsg(submap_ptr->getPose().template cast<double>(),
                       &submap_header.pose_estimate.map_pose);

    return submap_header;
  }

template <typename SubmapType>
void serializeSubmapToMsg(typename SubmapType::Ptr submap_ptr,
    cblox_msgs::MapLayer* msg) {
  CHECK_NOTNULL(msg);
  CHECK_NOTNULL(submap_ptr);

  ros::Time timestamp = ros::Time::now();
  // fill in headers
  msg->header = generateHeaderMsg<SubmapType>(submap_ptr, timestamp);
  msg->map_header = generateSubmapHeaderMsg<SubmapType>(submap_ptr);
  // fill in layers
  voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
      submap_ptr->getTsdfMapPtr()->getTsdfLayer(), false, &msg->tsdf_layer);
  msg->tsdf_layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
  voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(
      submap_ptr->getEsdfMapPtr()->getEsdfLayer(), false, &msg->esdf_layer);
  msg->esdf_layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
}

template <typename SubmapType>
bool deserializeMsgToSubmap(cblox_msgs::MapLayer* msg_ptr,
    typename SubmapCollection<SubmapType>::Ptr submap_collection_ptr) {

  CHECK_NOTNULL(submap_collection_ptr);
  if (!msg_ptr->map_header.is_submap) {
    return false;
  }

  // read id
  SubmapID submap_id = msg_ptr->map_header.id;
  // read pose
  kindr::minimal::QuatTransformationTemplate<double> pose;
  tf::poseMsgToKindr(msg_ptr->map_header.pose_estimate.map_pose, &pose);
  Transformation submap_pose = pose.cast<float>();

  // create or get submap in collection
  typename SubmapType::Ptr submap_ptr;
  if (submap_collection_ptr->exists(submap_id)) {
    submap_ptr = submap_collection_ptr->getSubmapPtr(submap_id);
    submap_ptr->setPose(submap_pose);
  } else {
    // create new submap
    submap_collection_ptr->createNewSubmap(submap_pose, submap_id);
  }

  // read mapping interval
  submap_ptr->startMappingTime(msg_ptr->map_header.start_time);
  submap_ptr->stopMappingTime(msg_ptr->map_header.end_time);

  // read tsdf layer
  bool success = voxblox::deserializeMsgToLayer(msg_ptr->tsdf_layer,
                                                submap_collection_ptr->getSubmapPtr(submap_id)->getTsdfMapPtr()->getTsdfLayerPtr());
  // read esdf layer
  // TODO: ONLY REPLACE IF LAYER IS SENT
  success &= voxblox::deserializeMsgToLayer(msg_ptr->esdf_layer,
                                            submap_collection_ptr->getSubmapPtr(submap_id)->getEsdfMapPtr()->getEsdfLayerPtr());

  return success;
}

template<>
inline void serializeSubmapToMsg<TsdfSubmap>(TsdfSubmap::Ptr submap_ptr,
    cblox_msgs::MapLayer* msg) {
  CHECK_NOTNULL(msg);
  CHECK_NOTNULL(submap_ptr);

  ros::Time timestamp = ros::Time::now();
  // fill in headers
  msg->header = generateHeaderMsg<TsdfSubmap>(submap_ptr, timestamp);
  msg->map_header = generateSubmapHeaderMsg<TsdfSubmap>(submap_ptr);
  // fill in layers
  voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
      submap_ptr->getTsdfMapPtr()->getTsdfLayer(), false, &msg->tsdf_layer);
  msg->tsdf_layer.action =
      static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
}

template<>
inline bool deserializeMsgToSubmap<TsdfSubmap>(cblox_msgs::MapLayer* msg_ptr,
    typename SubmapCollection<TsdfSubmap>::Ptr submap_collection_ptr) {

  CHECK_NOTNULL(submap_collection_ptr);
  if (!msg_ptr->map_header.is_submap) {
    return false;
  }

  // read id
  SubmapID submap_id = msg_ptr->map_header.id;
  // read pose
  kindr::minimal::QuatTransformationTemplate<double> pose;
  tf::poseMsgToKindr(msg_ptr->map_header.pose_estimate.map_pose, &pose);
  Transformation submap_pose = pose.cast<float>();

  // create or get submap in collection
  if (submap_collection_ptr->exists(submap_id)) {
    submap_collection_ptr->createNewSubmap(submap_pose, submap_id);
  }
  TsdfSubmap::Ptr submap_ptr = submap_collection_ptr->getSubmapPtr(submap_id);
  submap_ptr->setPose(submap_pose);

  // read mapping interval
  submap_ptr->startMappingTime(msg_ptr->map_header.start_time);
  submap_ptr->stopMappingTime(msg_ptr->map_header.end_time);

  // read tsdf layer
  return voxblox::deserializeMsgToLayer(msg_ptr->tsdf_layer,
                                        submap_collection_ptr->getSubmapPtr(submap_id)->getTsdfMapPtr()->getTsdfLayerPtr());
}


}
#endif //CBLOX_ROS_SUBMAP_CONVERSIONS_INL_H
