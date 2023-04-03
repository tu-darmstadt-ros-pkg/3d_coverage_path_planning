


#include <three_dimensional_coverage_path_planning/utils/file_utils.h>

#include <three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.h>


#include <ros/ros.h>

#include <pluginlib/class_loader.h>

#include <chrono>

using namespace three_dimensional_coverage_path_planning;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_selector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // load candidates and target set from file
  std::string file_name = "small_complete_candidates.bag";
//  std::string file_name = "small_target_candidates.bag";

  file_name = (file_name[0] == '/') ? file_name :
              ros::package::getPath(ROS_PACKAGE_NAME) + "/test/test_selector_node/data" + "/" + file_name;


  std::vector<Viewpoint> candidates;
  std::shared_ptr<const Point3dSet> target_set;
  utils::readRankedCandidatesAndTargetSetFromFile(file_name, candidates, target_set);

  ROS_INFO_STREAM(
    "Bag file successfully read. Candidates size: " << candidates.size() << ", target_set size: "
                                                    << target_set->size() << "\n");

  // Init solver
  pluginlib::ClassLoader<ViewpointSelectorBase> selector_loader_(
    "three_dimensional_coverage_path_planning",
    "three_dimensional_coverage_path_planning::ViewpointSelectorBase");

  std::string selector_names[4] = {"three_dimensional_coverage_path_planning::GreedySelector",
                                   "three_dimensional_coverage_path_planning::GreedyWithoutRedundanciesSelector",
                                   "three_dimensional_coverage_path_planning::ProbabilisticSelector",
                                   "three_dimensional_coverage_path_planning::ProbabilisticExpDistSelector"};

  std::unique_ptr<ViewpointSelectorBase> selector_;

  // parameters to use
  pnh.setParam("min_reward", 100);
  pnh.setParam("max_num_lost_covered_targets", 100);

  pnh.setParam("num_selections_repetitions", 1);

  int NUM_REPETITIONS = 1;



  // start async spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // all 4 selectors
//  for (const auto& selector_name: selector_names)
//  {
//
//    std::vector<std::pair<int, int>> data;
//
//    if (selector_name == selector_names[2] || selector_name == selector_names[3])
//    {
//      NUM_REPETITIONS = 100;
//    }
//    else
//    {
//      NUM_REPETITIONS = 1;
//    }
//
//
//    auto start = std::chrono::high_resolution_clock::now();
//
//    for (int i = 0; i < NUM_REPETITIONS; i++)
//    {
//
//      selector_.reset(selector_loader_.createUnmanagedInstance(selector_name));
//
//      selector_->initialize(pnh, candidates, target_set);
//
//      // start solver
//      selector_->selectViewpoints();
//
//      data.push_back(std::make_pair<int, int>(selector_->getSelectedViewpoints()->size(),
//                                              selector_->getUncoveredTargetPoints().size()));
//    }
//
//
//    auto stop = std::chrono::high_resolution_clock::now();
//
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//
//    ROS_INFO_STREAM(
//      selector_name.substr(selector_name.find_last_of(':') + 1) << ": " << selector_->getSelectedViewpoints()->size()
//                                                                << " selected viewpoints; "
//                                                                << selector_->getUncoveredTargetPoints().size()
//                                                                << " uncovered target points; time taken: "
//                                                                << duration.count() << "[microseconds].");
//
//
//    std::string data_file_name = file_name.substr(0, file_name.find_last_of('.')) + "_" + selector_name.substr(
//      selector_name.find_last_of(':') + 1) + ".txt";
//    utils::writeVectorInFile(data_file_name, data);
//  }


  // greedy with different min rewards (store reward of each to generate a graphic with reward over viewpoints

  // set new min_reward
  pnh.setParam("min_reward", 1);

  selector_.reset(selector_loader_.createUnmanagedInstance(selector_names[0]));

  selector_->initialize(pnh, candidates, target_set);

  // start solver
  selector_->selectViewpoints();

  auto selected = selector_->getSelectedViewpoints();

  std::vector<double> data;
  for(auto& viewpoint : *selected)
  {
    data.push_back(viewpoint.getReward());
  }

  std::string data_file_name = file_name.substr(0, file_name.find_last_of('.')) + "_" + "greedy_min_reward_1" + ".txt";
  utils::writeVectorInFile(data_file_name, data);

  ros::waitForShutdown();

  return 0;
}