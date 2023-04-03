#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MESH_NAVIGATION_CANDIDATE_GENERATOR_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MESH_NAVIGATION_CANDIDATE_GENERATOR_H

#include "three_dimensional_coverage_path_planning/viewpoint_computation/candidate_generator/candidate_generator_base.h"

#include <mesh_map/mesh_map.h>


namespace three_dimensional_coverage_path_planning
{


class MeshNavigationCandidateGenerator : public CandidateGeneratorBase
{
public:
  void initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model) override;

private:

  void generateCandidatePositions() override;

  pcl::PointCloud<pcl::PointXYZ>::Ptr iterAllVertices();

  pcl::PointCloud<pcl::PointXYZ>::Ptr iterVerticesFromStart(const mesh_map::Vector& start);


  float cost_threshold_;

  float percentage_candidates_to_keep_;
  unsigned int min_number_of_candidates_to_keep_;

  mesh_map::MeshMap::Ptr mesh_map_ptr_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MESH_NAVIGATION_CANDIDATE_GENERATOR_H
