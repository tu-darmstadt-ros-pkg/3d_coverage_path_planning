
#include <three_dimensional_coverage_path_planning/model/model_data.h>

#include <gtest/gtest.h>


using namespace three_dimensional_coverage_path_planning;

class ModelDataTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pnh_ = ros::NodeHandle("~model");

//    ROS_WARN_STREAM("test pnh namespace: " << pnh_.getNamespace());
//    std::vector<std::string> params;
//    pnh_.getParamNames(params);
//    for(auto& p : params)
//    {
//      ROS_WARN_STREAM(p);
//    }

    models_["full_model"] = std::make_shared<ModelData>(nh_, pnh_, true, true, true, "complete");
    models_["none_model"] = std::make_shared<ModelData>(nh_, pnh_, false, false, false, "complete");

    models_["no_esdf_model"] = std::make_shared<ModelData>(nh_, pnh_, true, false, true, "complete");
    models_["no_point_cloud_model"] = std::make_shared<ModelData>(nh_, pnh_, false, true, true, "complete");
    models_["no_octomap_model"] = std::make_shared<ModelData>(nh_, pnh_, true, true, false, "complete");

    models_["only_esdf_model"] = std::make_shared<ModelData>(nh_, pnh_, false, true, false, "complete");
    models_["only_point_cloud_model"] = std::make_shared<ModelData>(nh_, pnh_, true, false, false, "complete");
    models_["only_octomap_model"] = std::make_shared<ModelData>(nh_, pnh_, false, false, true, "complete");
  }

  std::map<std::string, std::shared_ptr<ModelData>> models_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};


TEST_F(ModelDataTest, InitFromMeshAndSave)
{
  // Combine initFromMesh and save in order to have saved files with current configuration for InitFromFiles tests

  for (auto& model: models_)
  {
    EXPECT_THROW(model.second->initFromMesh(""), std::invalid_argument)
            << "initFromMesh failed not with empty mesh path for model " + model.first;
  }


  std::string mesh_path = pnh_.param<std::string>("complete_mesh_path", "");
  for (auto& model: models_)
  {
    EXPECT_NO_THROW(model.second->initFromMesh(mesh_path))
            << "initFromMesh with valid mesh_path: \"" + mesh_path + "\" failed for model " + model.first;

    EXPECT_NO_THROW(model.second->saveModels()) << "saveModels throws for model " + model.first;
  }
}

TEST_F(ModelDataTest, InitFromFiles)
{
  std::string mesh_path = pnh_.param<std::string>("complete_mesh_path", "");
  for (auto& model: models_)
  {
    EXPECT_NO_THROW(model.second->initFromFiles(mesh_path))
            << "initFromFiles with valid mesh_path: \"" + mesh_path + "\" failed for model " + model.first;
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_model_data");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}