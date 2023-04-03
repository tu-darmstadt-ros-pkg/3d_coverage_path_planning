#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_BUILDING_MODEL_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_BUILDING_MODEL_H


#include "three_dimensional_coverage_path_planning/model/model_data.h"
#include "three_dimensional_coverage_path_planning/utils/types.h"

#include <ros/ros.h>

namespace three_dimensional_coverage_path_planning
{

class BuildingModel
{
public:
  BuildingModel(ros::NodeHandle& nh, ros::NodeHandle& pnh, bool load_models, bool save_models);

  /**
   * get a set containing all target points, i.e. all points that should be covered while executing the path
   * (e.g. points on walls, pipes, ...)
   * @return
   */
  std::shared_ptr<const Point3dSet> getTargetSet();

  std::shared_ptr<const ModelData> getCompleteModel();


  /**
    * Get Frame in which the building models are located.
    * @return frame id
    */
  std::string getModelFrame();

private:

  void generateTargetSet();


  // models
  std::shared_ptr<ModelData> complete_model_;
  std::unique_ptr<ModelData> target_model_;

  std::shared_ptr<Point3dSet> target_set_;

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_BUILDING_MODEL_H
