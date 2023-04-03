
#include <three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.h>
#include <three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/greedy_selector.h>
#include <three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/greedy_without_redundancies_selector.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <gtest/gtest.h>


using namespace three_dimensional_coverage_path_planning;

class ViewpointSelectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pnh_ = ros::NodeHandle("~");
    pnh_.setParam("min_reward", 1);
    pnh_.setParam("max_num_lost_covered_targets", 0);

    // setup class loader
    viewpoint_selector_loader_ = std::make_shared<pluginlib::ClassLoader<ViewpointSelectorBase>>(
      "three_dimensional_coverage_path_planning",
      "three_dimensional_coverage_path_planning::ViewpointSelectorBase");

    std::string model_frame = "model_frame";

    // setup data
    Point3d t1 = Point3d(1, 0, 0);
    Point3d t2 = Point3d(2, 0, 0);
    Point3d t3 = Point3d(3, 0, 0);
    Point3d t4 = Point3d(4, 0, 0);
    Point3d t5 = Point3d(5, 0, 0);
    Point3d t6 = Point3d(6, 0, 0);
    Point3d t7 = Point3d(7, 0, 0);
    Point3d t8 = Point3d(8, 0, 0);
    Point3d t9 = Point3d(9, 0, 0);
    Point3d t10 = Point3d(10, 0, 0);

    target_set_ = std::make_shared<Point3dSet>();
    *target_set_ = {t1, t2, t3, t4, t5, t6, t7, t8, t9, t10};
    targets_unordered_ = {t1, t2, t3, t4, t5, t6, t7, t8, t9, t10};

    expected_uncovered_targets_ = {t7, t10};
    expected_uncovered_targets_1_loss_ = {t1, t6, t7, t10};

    // candidates
    candidates_ = {
      Viewpoint(Pose3d(Point3d(2, 2, 0), model_frame)),
      Viewpoint(Pose3d(Point3d(4, 2, 0), model_frame)),
      Viewpoint(Pose3d(Point3d(5, 2, 0), model_frame)),
      Viewpoint(Pose3d(Point3d(8, 2, 0), model_frame)),
      Viewpoint(Pose3d(Point3d(5, -2, 0), model_frame))
    };

    candidates_[0].setVisibleTargets({t1, t2, t3});
    candidates_[1].setVisibleTargets({t2, t3, t4, t5});
    candidates_[2].setVisibleTargets({t4, t5, t6});
    candidates_[3].setVisibleTargets({t8, t9});
    candidates_[4].setVisibleTargets({t3, t8});
  }

  // common tests and computations for all plugins
  boost::shared_ptr<ViewpointSelectorBase> testSelector(const std::string& plugin_name, bool target_loss = false)
  {
    // construct
    EXPECT_NO_THROW(viewpoint_selector_loader_->createInstance(plugin_name));
    boost::shared_ptr<ViewpointSelectorBase> selector = viewpoint_selector_loader_->createInstance(plugin_name);

    // init
    EXPECT_NO_THROW(selector->initialize(pnh_, candidates_, target_set_));
    selector->initialize(pnh_, candidates_, target_set_);

    // select viewpoints
    EXPECT_NO_THROW(selector->selectViewpoints());
    selector->selectViewpoints();


    // check if correct targets remain uncovered
    auto uncovered_targets = selector->getUncoveredTargetPoints();
    auto expected = target_loss ? expected_uncovered_targets_1_loss_ : expected_uncovered_targets_;

    EXPECT_EQ(uncovered_targets.size(), expected.size())
            << "Size of uncovered_targets set was not correct.";

    auto res_it = uncovered_targets.begin();
    for (const auto& expected_uncovered_target: expected)
    {
      EXPECT_EQ(expected_uncovered_target, *res_it) << "Uncovered targets are not correct!";
      ++res_it;
    }

    return selector;
  }

  std::shared_ptr<pluginlib::ClassLoader<ViewpointSelectorBase>> viewpoint_selector_loader_;
  std::vector<Viewpoint> candidates_;
  std::shared_ptr<Point3dSet> target_set_;
  Point3dVector targets_unordered_; // vector with targets in order to make them more accessible for tests

  Point3dSet expected_uncovered_targets_;
  Point3dSet expected_uncovered_targets_1_loss_;

  ros::NodeHandle pnh_;
};


TEST_F(ViewpointSelectorTest, GreedySelector)
{
  auto selector = testSelector("three_dimensional_coverage_path_planning::GreedySelector");

  // check if correct viewpoints have been selected
  auto selected_viewpoints = selector->getSelectedViewpoints();

  // order 2 and then 0 as always the last one is taken
  std::vector<Viewpoint> expected_result = {candidates_[1], candidates_[3], candidates_[2], candidates_[0]};

  std::vector<Point3dSet> expected_seen_targets = {{targets_unordered_[1], targets_unordered_[2], targets_unordered_[3], targets_unordered_[4]},
                                                   {targets_unordered_[7], targets_unordered_[8]},
                                                   {targets_unordered_[5]},
                                                   {targets_unordered_[0]}};

  // check if right viewpoints in right order have been selected
  EXPECT_EQ(selected_viewpoints->size(), expected_result.size())
          << "Size of selected viewpoints vector was not correct.";
  for (int i = 0; i < selected_viewpoints->size(); ++i)
  {
    auto vp = selected_viewpoints->at(i);
    EXPECT_EQ(vp, expected_result.at(i))
            << "Selected viewpoints are not as expected at position: " << i
            << ", expected position:\n" << expected_result.at(i).getPose().position
            << ", result position:\n" << vp.getPose().position;

    // check if the seen targets for each viewpoint are the right ones
    EXPECT_EQ(vp.getTargetsSeenFromThisViewpoint().size(), expected_seen_targets[i].size())
            << "Size of selected viewpoints vector was not correct.";
    auto exp_it = expected_seen_targets[i].begin();
    for (const auto& it: vp.getTargetsSeenFromThisViewpoint())
    {
      EXPECT_EQ(it, *exp_it)
              << "Seen targets for viewpoint " << vp.getPose().position << " are not as expected. Expected: "
              << *exp_it << ", result: " << it;
      ++exp_it;
    }
  }
}

TEST_F(ViewpointSelectorTest, GreedyWithoutRedundanciesSelector0)
{
  pnh_.setParam("max_num_lost_covered_targets", 0);

  auto selector = testSelector("three_dimensional_coverage_path_planning::GreedyWithoutRedundanciesSelector");

  // check if correct viewpoints have been selected
  auto selected_viewpoints = selector->getSelectedViewpoints();

  // order 2 and then 0 as always the last one is taken
  std::vector<Viewpoint> expected_result = {candidates_[3], candidates_[2], candidates_[0]};

  std::vector<Point3dSet> expected_seen_targets = {{targets_unordered_[7], targets_unordered_[8]},
                                                   {targets_unordered_[3], targets_unordered_[4], targets_unordered_[5]},
                                                   {targets_unordered_[0], targets_unordered_[1], targets_unordered_[2]}};

  // check if right viewpoints in right order have been selected
  EXPECT_EQ(selected_viewpoints->size(), expected_result.size())
          << "Size of selected viewpoints vector was not correct.";
  for (int i = 0; i < selected_viewpoints->size(); ++i)
  {
    auto vp = selected_viewpoints->at(i);
    EXPECT_EQ(vp, expected_result.at(i))
            << "Selected viewpoints are not as expected at position: " << i
            << "\nexpected position:\n" << expected_result.at(i).getPose().position
            << "\nresult position:\n" << vp.getPose().position;

    // check if the seen targets for each viewpoint are the right ones
    EXPECT_EQ(vp.getTargetsSeenFromThisViewpoint().size(), expected_seen_targets[i].size())
            << "Size of seen targets from this viewpoint was not correct for viewpoint \n" << vp.getPose().position << ".";
    auto exp_it = expected_seen_targets[i].begin();
    for (const auto& it: vp.getTargetsSeenFromThisViewpoint())
    {
      EXPECT_EQ(it, *exp_it)
              << "Seen targets for viewpoint \n" << vp.getPose().position << "\nare not as expected. Expected: \n"
              << *exp_it << "\n result: \n" << it;
      ++exp_it;
    }
  }
}


TEST_F(ViewpointSelectorTest, GreedyWithoutRedundanciesSelector1)
{
  pnh_.setParam("max_num_lost_covered_targets", 1);

  auto selector = testSelector("three_dimensional_coverage_path_planning::GreedyWithoutRedundanciesSelector", true);

  // check if correct viewpoints have been selected
  auto selected_viewpoints = selector->getSelectedViewpoints();

  // order 2 and then 0 as always the last one is taken
  std::vector<Viewpoint> expected_result = {candidates_[1], candidates_[3]};

  std::vector<Point3dSet> expected_seen_targets = {{targets_unordered_[1], targets_unordered_[2], targets_unordered_[3], targets_unordered_[4]},
                                                   {targets_unordered_[7], targets_unordered_[8]}};


  // check if right viewpoints in right order have been selected
  EXPECT_EQ(selected_viewpoints->size(), expected_result.size())
          << "Size of selected viewpoints vector was not correct.";
  for (int i = 0; i < selected_viewpoints->size(); ++i)
  {
    auto vp = selected_viewpoints->at(i);
    EXPECT_EQ(vp, expected_result.at(i))
            << "Selected viewpoints are not as expected at position: " << i
            << "\nexpected position:\n" << expected_result.at(i).getPose().position
            << "\nresult position:\n" << vp.getPose().position;

    // check if the seen targets for each viewpoint are the right ones
    EXPECT_EQ(vp.getTargetsSeenFromThisViewpoint().size(), expected_seen_targets[i].size())
            << "Size of selected viewpoints vector was not correct.";
    auto exp_it = expected_seen_targets[i].begin();
    for (const auto& it: vp.getTargetsSeenFromThisViewpoint())
    {
      EXPECT_EQ(it, *exp_it)
              << "Seen targets for viewpoint \n" << vp.getPose().position << "\nare not as expected. Expected: \n"
              << *exp_it << "\n result: \n" << it;
      ++exp_it;
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_viewpoint_selectors");
  testing::InitGoogleTest(&argc, argv);

  ros::start();

  return RUN_ALL_TESTS();
}