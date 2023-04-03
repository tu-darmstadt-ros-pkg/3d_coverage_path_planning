
#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_computer.h"

#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"

#include <pcl_conversions/pcl_conversions.h>

// TEMPORARY
#include "three_dimensional_coverage_path_planning/utils/file_utils.h"

namespace three_dimensional_coverage_path_planning
{

ViewpointComputer::ViewpointComputer(std::shared_ptr<const ModelData> base_model,
                                     std::shared_ptr<const Point3dSet> target_set,
                                     PlanActionServer* const action_server)
  : candidate_generator_loader_("three_dimensional_coverage_path_planning",
                                "three_dimensional_coverage_path_planning::CandidateGeneratorBase"),
    viewpoint_selector_loader_("three_dimensional_coverage_path_planning",
                               "three_dimensional_coverage_path_planning::ViewpointSelectorBase"),
    base_model_(std::move(base_model)),
    target_set_(std::move(target_set)),
    action_server_(action_server)
{
  pnh_ = ros::NodeHandle("~viewpoint_computation");


  // init candidate generator
  std::string generator_name = pnh_.param<std::string>("candidate_generator_plugin",
                                                       "three_dimensional_coverage_path_planning::MeshNavigationCandidateGenerator");
  candidate_generator_.reset(candidate_generator_loader_.createUnmanagedInstance(generator_name));

  // init reward evaluator
  reward_evaluator_ = std::make_unique<RewardEvaluator>(pnh_, base_model_, target_set_);

  // init viewpoint selector
  std::string selector_name = pnh_.param<std::string>("viewpoint_selector_plugin",
                                                      "three_dimensional_coverage_path_planning::GreedySelector");
  viewpoint_selector_.reset(viewpoint_selector_loader_.createUnmanagedInstance(selector_name));


  // init publisher
  if (pnh_.param<bool>("publish_all_candidates", false))
  {
    all_candidates_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("all_candidates", 2, true);
  }
  if (pnh_.param<bool>("publish_uncovered_target_points", false))
  {
    uncovered_target_points_pub_ = pnh_.advertise<visualization_msgs::Marker>("uncovered_target_points", 2, true);
  }
}


std::shared_ptr<std::vector<Viewpoint>> ViewpointComputer::computeViewpoints()
{

  if (!action_utils::actionOk(action_server_))
  {
    return std::make_shared<std::vector<Viewpoint>>();
  }

  PlanActionFeedback feedback;

  feedback.progress = "Generating candidates";
  action_server_->publishFeedback(feedback);

  generateCandidates();

  if (!action_utils::actionOk(action_server_))
  {
    return std::make_shared<std::vector<Viewpoint>>();
  }

  feedback.progress = "Rating candidates";
  action_server_->publishFeedback(feedback);

  rateCandidates();

  if (!action_utils::actionOk(action_server_))
  {
    return std::make_shared<std::vector<Viewpoint>>();
  }

  feedback.progress = "Selecting viewpoints";
  action_server_->publishFeedback(feedback);

  selectViewpoints();

  ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, viewpoint_selector_->getSelectedViewpoints()->size() << " waypoints have been selected.");

  return viewpoint_selector_->getSelectedViewpoints();
}

// ============================ //
// ====== private methods ===== //
// ============================ //

void ViewpointComputer::generateCandidates()
{
  candidate_generator_->initialize(pnh_, base_model_);

  candidates_ = candidate_generator_->generateCandidates();

  ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, candidates_.size() << " candidates have been generated.");

  // TODO maybe reset candidate_generator here in order to not have conflicts with mesh map from mesh navigation path planner
}

void ViewpointComputer::rateCandidates()
{
  reward_evaluator_->computeRewards(candidates_);


  if (pnh_.param<bool>("publish_all_candidates", false))
  {
    // publish rated candidates
    pcl::PointCloud<pcl::PointXYZI> candidates_with_intensity;
    for (auto& candidate: candidates_)
    {
      pcl::PointXYZI p;
      p.x = static_cast<float>(candidate.getPose().position.x());
      p.y = static_cast<float>(candidate.getPose().position.y());
      p.z = static_cast<float>(candidate.getPose().position.z());
      p.intensity = static_cast<float>(candidate.getReward());
      candidates_with_intensity.emplace_back(p);
    }
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(candidates_with_intensity, pc_msg);
    pc_msg.header.frame_id = base_model_->getModelFrame();

    all_candidates_pub_.publish(pc_msg);
  }
}

void ViewpointComputer::selectViewpoints()
{

  // TEMPORARY write ranked candidates and target set to file for selector tests and comparison
//  std::string file_name = ros::package::getPath(ROS_PACKAGE_NAME) + "/test/test_selector_node/data" + "/" + "small_target_candidates.bag";
//  utils::writeRankedCandidatesAndTargetSetToFile(file_name, candidates_, target_set_, base_model_->getModelFrame());

  viewpoint_selector_->initialize(pnh_, candidates_, target_set_);

  viewpoint_selector_->selectViewpoints();

  ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, viewpoint_selector_->getSelectedViewpoints()->size() << " viewpoints have been selected.");
  ROS_WARN_STREAM(viewpoint_selector_->getUncoveredTargetPoints().size() << " target points have not been covered.");

  if (pnh_.param<bool>("publish_uncovered_target_points", false))
  {
    // publish selected candidates
    auto uncovered_tp = viewpoint_selector_->getUncoveredTargetPoints();
    visualization_msgs::Marker msg;
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.pose.orientation.w = 1;
    msg.scale.x = 0.025;
    msg.scale.y = 0.025;
    msg.header.frame_id = base_model_->getModelFrame();
    msg.id = 19;

    for (auto& target: uncovered_tp)
    {
      msg.points.push_back(utils::toMsgPoint(target));
      msg.colors.push_back(colors::red());
    }
    uncovered_target_points_pub_.publish(msg);
  }
}
} // end namespace three_dimensional_coverage_path_planning
