#include "three_dimensional_coverage_path_planning/model/real_world_model.h"

namespace three_dimensional_coverage_path_planning
{

RealWorldModel::RealWorldModel(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh, "model")
{
  // load target model
  std::string target_mesh_path = pnh_.param<std::string>("target_mesh_path", "");

  target_model_ = std::make_unique<ModelData>(nh_, pnh_, true, false, false, "target");
  target_model_->initFromFiles(target_mesh_path);
}
} // end namespace three_dimensional_coverage_path_planning
