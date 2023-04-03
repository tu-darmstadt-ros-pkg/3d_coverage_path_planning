#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_REAL_WORLD_MODEL_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_REAL_WORLD_MODEL_H


#include "ros/ros.h"
#include "three_dimensional_coverage_path_planning/model/model_data.h"


namespace three_dimensional_coverage_path_planning
{
// TODO class RealWorldModel
class RealWorldModel
{
public:
  RealWorldModel(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:

  std::unique_ptr<ModelData> target_model_;

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_REAL_WORLD_MODEL_H
