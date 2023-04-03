#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VISIBILITY_CHECKER_BASE_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VISIBILITY_CHECKER_BASE_H

#include "three_dimensional_coverage_path_planning/model/model_data.h"

#include "three_dimensional_coverage_path_planning/utils/colors.h"
#include "three_dimensional_coverage_path_planning/utils/types.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


namespace three_dimensional_coverage_path_planning
{

class VisibilityCheckerBase
{

public:
  VisibilityCheckerBase() = default;

  /**
   * Initialize plugin. When overwriting, call VisibilityCheckerBase first in order to set up all variables correctly.
   * @param model Model that should be used for checks.
   */
  virtual void initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model);

  /**
   * Check if a point might be visible.
   * This method is implemented in the plugins.
   * @param start viewpoint
   * @param end target point, that should be seen
   * @return true if is might be visible, false otherwise
   */
  virtual bool check(Pose3d& start, Point3d& end) = 0;


  // stats
  virtual void printStats();

  void publishAllMarkers();

  void resetAllMsgs();


  std::string getPluginName();


protected:

  void addMarkerToMsg(const int& idx, Point3d& start, Point3d& end,
                      std_msgs::ColorRGBA color_start, std_msgs::ColorRGBA color_end);

  void
  addMarkerToMsg(const int& idx, Point3d& start, Point3d& end, std_msgs::ColorRGBA color);

  void publishMarkerMsg(const int& idx);

  void incStats(const int& idx);


  // data
  std::shared_ptr<const ModelData> model_; /// model on which the visibility will be checked

  std::string plugin_name_;

  int num_idx_ = 0;

  ros::NodeHandle pnh_;

  // visualization
  bool publish_visibility_checks_ = false;

  std::vector<ros::Publisher> check_pubs_;
  std::vector<visualization_msgs::Marker> check_msgs_;

  std::vector<std::vector<geometry_msgs::Point>> points_;
  std::vector<std::vector<std_msgs::ColorRGBA>> colors_;

  // stats
  bool print_stats_ = false;
  std::vector<int> stats_;
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VISIBILITY_CHECKER_BASE_H
