#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_SELECTOR_BASE_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_SELECTOR_BASE_H


#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


namespace three_dimensional_coverage_path_planning
{


class ViewpointSelectorBase
{
public:
  ViewpointSelectorBase();

  /**
   * Initialize viewpoint selector.
   * When overwriting, also call ViewpointSelectorBase::initialize in order to set up all variables correctly.
   * @param nh NodeHande in whose namespace the parameters (e.g. min_reward) can be found
   * @param candidates
   * @param target_set
   */
  virtual void initialize(ros::NodeHandle& nh, std::vector<Viewpoint>& candidates,
                          const std::shared_ptr<const Point3dSet>& target_set);

  virtual void selectViewpoints() = 0;

  /**
   * Get a list of selected viewpoints.
   * @return
   */
  std::shared_ptr<std::vector<Viewpoint>> getSelectedViewpoints();

  /**
   * Get a list of all uncovered target points.
   * (Contains all target points if selectViewpoints was not called before).
   * @return
   */
  const Point3dSet& getUncoveredTargetPoints();

protected:

  /**
   * Check if selection is sufficient (break condition for terminating viewpoint selection).
   * Might be overwritten in order to modify/add conditions.
   * @return true if selection is sufficient
   */
  virtual bool isSelectionSufficient();

  /**
   * Adds viewpoint to selected_viewpoints_,
   * removes it from remaining_candidates_,
   * updates reward_ of all other remaining candidates,
   * removes covered points from uncovered_target_points_ set.
   * @param viewpoint viewpoint to select
   */
  void selectViewpoint(Viewpoint viewpoint);

  /**
   * Resets the selector in order to allow the next execution, e.g. if computed multiple times.
   *
   * Does the following actions:
   *    - remaining_candidates_ = all_candidates_
   *    - uncovered_target_points_ = all_target_points_
   *    - selected_viewpoints_ is cleared.
   *    - delete all markers
   */
  void resetSelector();


  double min_reward_; /// minimal reward a candidate must have to be selectable (used in isSelectionSufficient)

  std::shared_ptr<std::vector<Viewpoint>> selected_viewpoints_;

  std::vector<Viewpoint> remaining_candidates_;

  std::vector<Viewpoint> all_candidates_;

  Point3dSet uncovered_target_points_;

  Point3dSet all_target_points_;

  /**
   * If the mapping covered target point --> viewpoints that cover this viewpoint shall be stored.
   * False in base class, can be set to true in subclasses
   */
  bool store_map_target_to_viewpoints_;

  std::map<Point3d, std::vector<Viewpoint>, Point3dLessComparator> map_covered_targets_to_viewpoints_; /// mapping covered target point --> viewpoints that cover this viewpoint, see store_map_target_to_viewpoints_

  ros::NodeHandle pnh_;

  // visualization
  std::shared_ptr<ros::Publisher> selected_candidates_pub_;
  visualization_msgs::Marker selected_cand_msg_;
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_SELECTOR_BASE_H
