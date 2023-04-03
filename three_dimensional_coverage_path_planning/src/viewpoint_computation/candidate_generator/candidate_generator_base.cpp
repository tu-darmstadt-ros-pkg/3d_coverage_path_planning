
#include "three_dimensional_coverage_path_planning/viewpoint_computation/candidate_generator/candidate_generator_base.h"
#include "three_dimensional_coverage_path_planning/utils/utils.h"

#include <utility>

namespace three_dimensional_coverage_path_planning
{

void CandidateGeneratorBase::initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model)
{
  pnh_ = ros::NodeHandle(pnh, "candidate_generation");
  model_ = std::move(model);
}

std::vector<Viewpoint> CandidateGeneratorBase::generateCandidates()
{
  generateCandidatePositions();

  generateCandidateOrientations();

  return candidate_viewpoints_;
}


void CandidateGeneratorBase::generateCandidateOrientations()
{

  if (candidate_viewpoints_.empty())
  {
    throw std::runtime_error("Cannot generate orientations for candidates as candidate list is empty!");
  }


  // get candidate orientation factor
  double candidate_orientation_factor;
  utils::searchAndGetParam(pnh_, "candidate_orientation_factor", 2.0, candidate_orientation_factor);

  // if factor is 0, do not generate candidate orientations
  if (candidate_orientation_factor == 0)
  {
    ROS_WARN_STREAM("Candidate orientation factor was 0.0, no candidate orientations will be generated, all candidate orientations are set to Identity!");
    return;
  }


  std::map<std::string, double> fov;
  utils::searchAndGetParam(pnh_, "field_of_view", fov);

  // get fov angles and convert to radian
  double fov_horizontal_min = fov.at("horizontal_min") * M_PI / 180;
  double fov_horizontal_max = fov.at("horizontal_max") * M_PI / 180;



  // compute number of orientations to generate
  double complete_horiz_fov = std::abs(fov_horizontal_min) + std::abs(fov_horizontal_max);
  int num_orientations = std::ceil(candidate_orientation_factor * (2 * M_PI) / complete_horiz_fov);

  double orientation_increment = 2.0 * M_PI / num_orientations;

  std::vector<Viewpoint> candidates_with_orientations;
  candidates_with_orientations.reserve(candidate_viewpoints_.size() * num_orientations);

  for (auto& candidate: candidate_viewpoints_)
  {
    for (int i = 0; i < num_orientations; i++)
    {
      // set orientation as rotation around the z axis
      Eigen::Vector3d euler(0, 0, orientation_increment * i);

      Quaterniond orientation;
      utils::rotationEulerToQuaternion(euler, orientation);

      Pose3d old_pose = candidate.getPose();
      Pose3d pose(old_pose.position, orientation, old_pose.frame_id);

      candidates_with_orientations.emplace_back(pose);
    }
  }

  // replace the candidate viewpoints with the newly generated
  candidate_viewpoints_ = candidates_with_orientations;
}
} // end namespace three_dimensional_coverage_path_planning
