
#include "three_dimensional_coverage_path_planning/model/model_data.h"

#include "three_dimensional_coverage_path_planning/utils/utils.h"

#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <mesh_conversions/ConvertMesh.h>
#include <mesh_to_sampled_point_cloud/mesh_to_sampled_point_cloud.h>
#include <mesh_to_sdf/mesh_to_sdf.h>


namespace three_dimensional_coverage_path_planning
{


ModelData::ModelData(ros::NodeHandle& nh, ros::NodeHandle& pnh, bool have_point_cloud, bool have_sdf,
                     bool have_octomap, std::string model_name)
  : model_name_(model_name), nh_(nh), pnh_(pnh, model_name),
    have_point_cloud_(have_point_cloud), have_octomap_(have_octomap), have_sdf_(have_sdf)
{
  // get ros params

  // general model namespace (search for params to retrieve them from private or parent namespace
  utils::searchAndGetParam(pnh_, "data_directory", ros::package::getPath(ROS_PACKAGE_NAME) + "/data/", data_directory_);
  utils::searchAndGetParam(pnh_, "voxel_size", 0.05f, voxel_size_);
  utils::searchAndGetParam(pnh_, "model_frame", std::string("world"), model_frame_);

  // mesh namespace
  std::string mesh_param_prefix = "mesh/";
  scale_factor_ = pnh_.param<float>(mesh_param_prefix + "scale_factor", 1.0);
  translation_.x = pnh_.param(mesh_param_prefix + "translation_x", 0);
  translation_.y = pnh_.param(mesh_param_prefix + "translation_y", 0);
  translation_.z = pnh_.param(mesh_param_prefix + "translation_z", 0);

  rotation_euler_deg_.x = pnh_.param(mesh_param_prefix + "rotate_around_x", 0);
  rotation_euler_deg_.y = pnh_.param(mesh_param_prefix + "rotate_around_y", 0);
  rotation_euler_deg_.z = pnh_.param(mesh_param_prefix + "rotate_around_z", 0);

  // convert euler angles to quaternion
  utils::rotationEulerToQuaternion(rotation_euler_deg_, rotation_quat_);


  // init service clients
  mesh_conversions_client_ = nh_.serviceClient<mesh_conversions::ConvertMesh>("/mesh_conversions/convert_mesh");

  // init publisher
  if (have_point_cloud || have_octomap_)
  {
    point_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1, true);
  }
  if (have_octomap_)
  {
    octomap_pub_ = pnh_.advertise<octomap_msgs::Octomap>("octomap", 1, true);
  }
  if (have_sdf_)
  {
    sdf_point_cloud_pub_ = pnh_.advertise<pcl::PointCloud<pcl::PointXYZI> >("sdf_pointcloud", 1, false);
    sdf_slice_pub_ = pnh_.advertise<pcl::PointCloud<pcl::PointXYZI> >("sdf_slice", 1, true);
  }




  // init required models

  if (have_point_cloud_ || have_octomap_)
  {
    point_cloud_ = std::make_shared<sensor_msgs::PointCloud2>();
    point_cloud_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }
  if (have_octomap_)
  {
    ros::NodeHandle octomap_nh(pnh_, "octomap");
    float voxel_size_octomap;
    utils::searchAndGetParam(octomap_nh, "voxel_size", voxel_size_, voxel_size_octomap);

    octomap_ = std::make_shared<octomap::OcTree>(voxel_size_octomap);
  }
  if (have_sdf_)
  {
    ros::NodeHandle sdf_nh(pnh_, "sdf");
    bool use_tsdf = sdf_nh.param<bool>("use_tsdf", true);

    float voxel_size_sdf;
    utils::searchAndGetParam(sdf_nh, "voxel_size", voxel_size_, voxel_size_sdf);

    if (use_tsdf)
    {
      voxblox::TsdfMap::Config config;
      config.tsdf_voxel_size = voxel_size_sdf;
      sdf_map_ = std::make_shared<SdfMap>(config);
    }
    else
    {
      voxblox::EsdfMap::Config config;
      config.esdf_voxel_size = voxel_size_sdf;
      sdf_map_ = std::make_shared<SdfMap>(config);
    }
  }
}

void
ModelData::initFromMesh(const std::string& mesh_path)
{
  models_loaded_ = false;

  setMeshPath(mesh_path);

  convertMeshToPly();

  // computing the octomap requires the point_cloud to be present
  if (have_point_cloud_ || have_octomap_)
  {
    computePointCloudFromMesh();
  }

  if (have_sdf_)
  {
    computeSdfFromMesh();
  }

  if (have_octomap_)
  {
    resetOctomapFromPointCloud();
  }

  // compute bounding box if not set yet
  if((bbx_max_-bbx_min_).norm() < 0.001)
  {
    computeBBX();
  }
}


void ModelData::initFromFiles(std::string mesh_path)
{
  models_loaded_ = true;

  // If mesh path is empty, use default values.
  // In this way, there is no mesh required, when each file name is specified in the config files.
  if (!mesh_path.empty())
  {
    setMeshPath(mesh_path);
  }

  // computing the octomap requires the point_cloud to be present
  if (have_point_cloud_ || have_octomap_)
  {
    readPointCloudFromFile();
  }

  if (have_sdf_)
  {
    readSdfFromFile();
  }

  if (have_octomap_)
  {
    readOctomapFromFile();
  }

  // compute bounding box if not set yet
  if((bbx_max_-bbx_min_).norm() < 0.001)
  {
    computeBBX();
  }
}

void ModelData::saveModels()
{
  if (have_point_cloud_)
  {
    if (pcl::io::savePCDFile(setAndGetPointCloudPath(), *point_cloud_) != 0)
    {
      throw std::invalid_argument("Error while saving PCD file. Please see PCL error for more information.");
    }
  }

  if (have_sdf_)
  {
    try
    {
      sdf_map_->saveToFile(setAndGetSdfPath(), true);
    }
    catch (std::exception& e)
    {
      throw std::invalid_argument("Error while saving the sdf map. Please see voxblox error for more information.");
    }
  }

  if (have_octomap_)
  {
    if (!octomap_->writeBinaryConst(setAndGetOctomapPath()))
    {
      throw std::invalid_argument("Error while saving the octomap. Please see octomap error for more information.");
    }
  }
}


std::string ModelData::getMeshPath()
{
  return model_paths_.at("mesh");
}

std::string ModelData::getPlyMeshPath()
{
  return model_paths_.at("ply_mesh");
}

std::string ModelData::getPointCloudPath()
{
  return model_paths_.at("point_cloud");
}

std::string ModelData::getSdfPath()
{
  return model_paths_.at("sdf");
}

std::string ModelData::getOctomapPath()
{
  return model_paths_.at("octomap");
}


std::shared_ptr<const sensor_msgs::PointCloud2> ModelData::getPointCloud() const
{
  return point_cloud_;
}

std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> ModelData::getPointCloudXYZ() const
{
  return point_cloud_pcl_;
}


std::shared_ptr<const SdfMap> ModelData::getSdf() const
{
  return sdf_map_;
}


std::shared_ptr<const octomap::OcTree> ModelData::getOctomap() const
{
  return octomap_;
}


std::string ModelData::getModelFrame() const
{
  return model_frame_;
}


std::string ModelData::getModelName() const
{
  return model_name_;
}


Point3d ModelData::getBBXMin() const
{
  return bbx_min_;
}

Point3d ModelData::getBBXMax() const
{
  return bbx_max_;
}

bool ModelData::modelsLoaded() const
{
  return models_loaded_;
}




// =========================================== //
// ============= private methods ============= //
// =========================================== //

std::string ModelData::setAndGetPlyMeshPath()
{
  setPath("ply_mesh");
  return getPlyMeshPath();
}

std::string ModelData::setAndGetPointCloudPath()
{
  setPath("point_cloud");
  return getPointCloudPath();
}

std::string ModelData::setAndGetSdfPath()
{
  setPath("sdf");
  return getSdfPath();
}

std::string ModelData::setAndGetOctomapPath()
{
  setPath("octomap");
  return getOctomapPath();
}

void ModelData::setMeshPath(std::string mesh_path)
{
  if (mesh_path.empty())
  {
    throw std::invalid_argument("mesh_path was empty!");
  }

  completePath(mesh_path);
  model_paths_["mesh"] = mesh_path;
}

void ModelData::setPath(const std::string& model_type)
{
  // check if path is already in map
  if (model_paths_.count(model_type) > 0)
  {
    return;
  }

  if (model_type == "mesh")
  {
    throw std::logic_error("Mesh path was tried to set using the wrong method!");
  }

  // Try to retrieve it from parameter server (general model namespace)
  std::string tmp_path;
  std::string path_param_name;
  if (pnh_.searchParam(model_name_ + "_" + model_type + "_path", path_param_name) &&
      pnh_.getParam(path_param_name, tmp_path))
  {
    if (tmp_path.empty())
    {
      throw std::invalid_argument(model_type + "_path was set on parameter server but is an empty string!");
    }

    completePath(tmp_path);
    model_paths_[model_type] = tmp_path;

    return;
  }

  // Last option: create the path
  if (model_file_default_name_.empty())
  {
    char buffer[256];
    auto time = static_cast<std::time_t>(ros::WallTime::now().toSec());
    auto local_time = std::localtime(&time);
    strftime(buffer, sizeof(buffer), "%y-%m-%d_%H-%M", local_time);
    model_file_default_name_ = std::string(buffer) + "_" + model_name_ + "_" + model_type;
  }

  tmp_path = data_directory_ + "/" + model_file_default_name_;

  // add file extension
  if (model_type == "ply_mesh")
  {
    tmp_path += ".ply";
  }
  else if (model_type == "point_cloud")
  {
    tmp_path += ".pcd";
  }
  else if (model_type == "sdf")
  {
    if (sdf_map_->isTsdf())
    {
      tmp_path += ".tsdf";
    }
    else
    {
      tmp_path += ".esdf";
    }
  }
  else if (model_type == "octomap")
  {
    tmp_path += ".bt";
  }
  else
  {
    throw std::invalid_argument("Unknown model type requested: \"" + model_type + "\".");
  }

  model_paths_[model_type] = tmp_path;
}


void ModelData::completePath(std::string& path)
{
  // check if path is absolute or relative path (relative to data directory!)
  path = (path[0] == '/') ? path : data_directory_ + "/" + path;

  // if not done yet, set mode_file_name
  if (model_file_default_name_.empty())
  {
    auto start_pos = path.find_last_of('/') + 1;
    auto end_pos = path.find_last_of('.');
    if (end_pos == std::string::npos)
    {
      throw std::invalid_argument("No file extension given in file path \"" + path + "\"!");
    }
    model_file_default_name_ = model_name_ + "_" + path.substr(start_pos, end_pos - start_pos);
  }
}


void ModelData::computeBBX()
{
  octomap::Pointcloud octocloud;
  octomap::pointCloud2ToOctomap(*point_cloud_, octocloud);

  octomap::point3d lower, upper;
  octocloud.calcBBX(lower, upper);

  // set bounding box for model:
  bbx_min_ = Point3d(lower.x(), lower.y(), lower.z());
  bbx_max_ = Point3d(upper.x(), upper.y(), upper.z());
}


void ModelData::convertMeshToPly()
{
  // service call
  mesh_conversions::ConvertMeshRequest request;
  mesh_conversions::ConvertMeshResponse response;

  request.input_file_name = getMeshPath();
  request.output_file_name = setAndGetPlyMeshPath();


  mesh_conversions_client_.waitForExistence(ros::Duration(10));


  bool service_result = mesh_conversions_client_.call(request, response);

  // check if something went wrong
  if (!service_result)
  {
    throw ros::Exception("Service call of service \"" + mesh_conversions_client_.getService() + "\" failed!");
  }

  if (response.result == mesh_conversions::ConvertMeshResponse::LOAD_FILE_ERROR)
  {
    throw std::invalid_argument(
      "Service \"" + mesh_conversions_client_.getService() + "\" returned a LOAD_FILE_ERROR with message: " +
      response.error_msg);
  }
  else if (response.result == mesh_conversions::ConvertMeshResponse::CONVERSION_ERROR)
  {
    throw std::domain_error(
      "Service \"" + mesh_conversions_client_.getService() + "\" returned a CONVERSION_FILE_ERROR with message: " +
      response.error_msg);
  }
  else if (response.result == mesh_conversions::ConvertMeshResponse::SAVE_FILE_ERROR)
  {
    throw std::invalid_argument(
      "Service \"" + mesh_conversions_client_.getService() + "\" returned a SAVE_FILE_ERROR with message: " +
      response.error_msg);
  }
}


void ModelData::computePointCloudFromMesh()
{
  // prepare required data
  ros::NodeHandle pc_nh(pnh_, "point_cloud");

  std::string input_file = setAndGetPlyMeshPath();

  // load and convert mesh to sampled point cloud
  mesh_to_sampled_point_cloud::MeshToSampledPointCloud mesh_to_sampled_point_cloud(
    input_file,
    pc_nh.param<int>("num_sample_points", 100000),
    pc_nh.param<float>("voxel_filter_leaf_size", 0.1),
    translation_, rotation_euler_deg_, scale_factor_,
    pc_nh.param<bool>("write_normals", false),
    pc_nh.param<bool>("write_colors", false));


  // get point cloud and convert it in the required type
  std::shared_ptr<pcl::PCLPointCloud2> point_cloud = mesh_to_sampled_point_cloud.getPointCloud();

  if (!point_cloud || !point_cloud_ || point_cloud->data.empty())
  {
    throw std::runtime_error("The sampled point cloud was empty!");
  }

//  point_cloud->header.frame_id = "world";
  point_cloud->header.stamp = 0;
  point_cloud->header.seq = 0;

  point_cloud_->header.seq = 0;
  point_cloud_->header.stamp = ros::Time(0);
//  point_cloud_->header.frame_id = "world";
  // Note: In the test cases (not in other execution) in the next line when copying the header, a segmentation fault sometimes occurred. Just to keep in mind.

  pcl_conversions::fromPCL(*point_cloud, *point_cloud_);


  point_cloud_pcl_ = mesh_to_sampled_point_cloud.getPointCloudXYZ();

  publishPointCloud();
}

void ModelData::readPointCloudFromFile()
{
  if (pcl::io::loadPCDFile(setAndGetPointCloudPath(), *point_cloud_) != 0)
  {
    throw std::runtime_error(
      "Could not read Point Cloud from file \"" + getPointCloudPath() + "\". For details see pcl error on console.");
  }
  pcl::fromROSMsg(*point_cloud_, *point_cloud_pcl_);

  publishPointCloud();
}


void ModelData::computeSdfFromMesh()
{
  // prepare required data
  ros::NodeHandle sdf_nh(pnh_, "sdf");

  std::string input_file = setAndGetPlyMeshPath();

  geometry_msgs::Transform transform;
  transform.translation = translation_;
  transform.rotation = rotation_quat_;

  // load and convert mesh to sdf
  mesh_to_sdf::MeshToSdf mesh_to_sdf(
    input_file, transform, scale_factor_,
    sdf_nh.param<bool>("fill_inside", false),
    sdf_nh.param<bool>("floodfill_unoccupied", false),
    sdf_map_->getVoxelSize(),
    sdf_nh.param<float>("truncation_distance", 0.4));


  // get sdf
  if (sdf_map_->isTsdf())
  {
    sdf_map_->setLayer(*mesh_to_sdf.getTsdfMap()->getTsdfLayerPtr());
  }
  else
  {
    sdf_map_->setLayer(*mesh_to_sdf.getEsdfMap()->getEsdfLayerPtr());
  }

  publishSdf();
}

void ModelData::readSdfFromFile()
{
  try
  {
    sdf_map_->loadFromFile(setAndGetSdfPath());
    publishSdf();
  }
  catch (std::exception& e)
  {
    throw std::runtime_error(
      "Could not read sdf map from file \"" + getSdfPath() + "\". For details see voxblox error on console.");
  }
}


void ModelData::resetOctomapFromPointCloud()
{
  // prepare required data
  ros::NodeHandle octomap_nh(pnh_, "octomap");

  // convert pcl point cloud to octomap point cloud
  octomap::Pointcloud octocloud;
  octomap::pointCloud2ToOctomap(*point_cloud_, octocloud);

  // use bounding box for sensor origin in order to ensure, that all voxels in the model are set to free/occupied
  octomap::point3d lower, upper;
  octocloud.calcBBX(lower, upper);
  octomap::point3d sensor_origin(lower.x(), lower.y(), upper.z());

  // set bounding box for model:
  bbx_min_ = Point3d(lower.x(), lower.y(), lower.z());
  bbx_max_ = Point3d(upper.x(), upper.y(), upper.z());

  // insert point cloud in octomap
  octomap_->insertPointCloud(octocloud, sensor_origin);


  publishOctomap();
}


void ModelData::readOctomapFromFile()
{
  bool load_map_result = octomap_->readBinary(setAndGetOctomapPath());

  if (!load_map_result)
  {
    throw std::runtime_error(
      "Could not read octomap from file \"" + getOctomapPath() + "\". For details see octomap error on console.");
  }

  publishOctomap();
}


void ModelData::publishPointCloud()
{
  point_cloud_->header.frame_id = model_frame_;
  point_cloud_->header.stamp = ros::Time::now();

  point_cloud_pub_.publish(*point_cloud_);
}


void ModelData::publishOctomap()
{
  octomap_msgs::Octomap octomap_msg;

  octomap_msgs::binaryMapToMsg(*octomap_, octomap_msg);

  octomap_msg.header.frame_id = model_frame_;
  octomap_msg.header.stamp = ros::Time::now();

  octomap_pub_.publish(octomap_msg);
}


void ModelData::publishSdf()
{
  publishSdfPointCloud();
  publishSdfSlice();
}

void ModelData::publishSdfPointCloud()
{
  // Create a point_cloud_msg with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> point_cloud_msg;

  sdf_map_->createDistancePointcloud(point_cloud_msg);

  point_cloud_msg.header.frame_id = model_frame_;
  pcl_conversions::toPCL(ros::Time::now(), point_cloud_msg.header.stamp);

  sdf_point_cloud_pub_.publish(point_cloud_msg);
}

void ModelData::publishSdfSlice()
{
  pcl::PointCloud<pcl::PointXYZI> point_cloud_msg;

  auto slice_level = pnh_.param<float>("sdf/slice_level", 0.5);

  sdf_map_->createDistancePointcloudSlice(point_cloud_msg, 2, slice_level);

  point_cloud_msg.header.frame_id = model_frame_;
  pcl_conversions::toPCL(ros::Time::now(), point_cloud_msg.header.stamp);
  sdf_slice_pub_.publish(point_cloud_msg);
}
} // end namespace three_dimensional_coverage_path_planning

