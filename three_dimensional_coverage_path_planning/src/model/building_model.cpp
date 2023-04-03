#include "three_dimensional_coverage_path_planning/model/building_model.h"

namespace three_dimensional_coverage_path_planning
{
BuildingModel::BuildingModel(ros::NodeHandle& nh, ros::NodeHandle& pnh, bool load_models, bool save_models)
  : nh_(nh),
    pnh_(pnh,
         "model")
{
  // Create models.
  complete_model_ = std::make_shared<ModelData>(nh_, pnh_, true, true, true, "complete");
  target_model_ = std::make_unique<ModelData>(nh_, pnh_, true, false, false, "target");


  // get mesh paths
  std::string complete_mesh_path = pnh_.param<std::string>("complete_mesh_path", "");
  std::string target_mesh_path = pnh_.param<std::string>("target_mesh_path", "");

  // init models
  if (load_models)
  {
    complete_model_->initFromFiles(complete_mesh_path);
    target_model_->initFromFiles(target_mesh_path);
  }
  else
  {
    complete_model_->initFromMesh(complete_mesh_path);
    target_model_->initFromMesh(target_mesh_path);
  }

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, "Loading models finished.");

  // only save models if requested and they were not loaded before (only save newly computed models)
  if (save_models && !load_models)
  {
    complete_model_->saveModels();
    target_model_->saveModels();

    ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, "Saving models finished.");
  }

  // prepare other data
  generateTargetSet();
}

std::shared_ptr<const Point3dSet> BuildingModel::getTargetSet()
{
  return target_set_;
}


std::shared_ptr<const ModelData> BuildingModel::getCompleteModel()
{
  return complete_model_;
}


std::string BuildingModel::getModelFrame()
{
  return complete_model_->getModelFrame();
}


// =========================================== //
// ============= private methods ============= //
// =========================================== //


void BuildingModel::generateTargetSet()
{
  // Note: generateTargetSet: maybe filter point cloud (to have less points) or generate target set from other source (e.g. octomap) (?)
  auto filtered_cloud = target_model_->getPointCloudXYZ();

  Point3dVector point_vector;
  point_vector.reserve(filtered_cloud->size());
  Point3d tmp;

  // convert pcl::PointXYZ data to Point3d
  for (auto& it: *filtered_cloud)
  {
    utils::toEigenVector3d(it, tmp);
    point_vector.push_back(tmp);
  }

  // sort in order to lower complexity when inserting points in set
  std::sort(point_vector.begin(), point_vector.end(), Point3dLessComparator());

  target_set_ = std::make_shared<Point3dSet>(point_vector.begin(), point_vector.end());
}
} // end namespace three_dimensional_coverage_path_planning
