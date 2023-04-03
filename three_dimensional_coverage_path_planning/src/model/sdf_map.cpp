#include "three_dimensional_coverage_path_planning/model/sdf_map.h"

#include <voxblox_ros/ptcloud_vis.h>

namespace three_dimensional_coverage_path_planning
{


SdfMap::SdfMap(const voxblox::TsdfMap::Config& config)
{
  tsdf_map_ = std::make_shared<voxblox::TsdfMap>(config);
  tsdf_interpolator_ = std::make_shared<voxblox::Interpolator<voxblox::TsdfVoxel>>(tsdf_map_->getTsdfLayerPtr());
}

SdfMap::SdfMap(const voxblox::EsdfMap::Config& config)
{
  esdf_map_ = std::make_shared<voxblox::EsdfMap>(config);
  esdf_interpolator_ = std::make_shared<voxblox::Interpolator<voxblox::EsdfVoxel>>(esdf_map_->getEsdfLayerPtr());
}


SdfMap::SdfMap(const voxblox::Layer<voxblox::TsdfVoxel>& layer)
{
  tsdf_map_ = std::make_shared<voxblox::TsdfMap>(layer);
  tsdf_interpolator_ = std::make_shared<voxblox::Interpolator<voxblox::TsdfVoxel>>(tsdf_map_->getTsdfLayerPtr());
}

SdfMap::SdfMap(const voxblox::Layer<voxblox::EsdfVoxel>& layer)
{
  esdf_map_ = std::make_shared<voxblox::EsdfMap>(layer);
  esdf_interpolator_ = std::make_shared<voxblox::Interpolator<voxblox::EsdfVoxel>>(esdf_map_->getEsdfLayerPtr());
}


void SdfMap::setLayer(const voxblox::Layer<voxblox::TsdfVoxel>& layer)
{
  *tsdf_map_->getTsdfLayerPtr() = layer;
}

void SdfMap::setLayer(const voxblox::Layer<voxblox::EsdfVoxel>& layer)
{
  *esdf_map_->getEsdfLayerPtr() = layer;
}


bool SdfMap::isTsdf() const
{
  return bool(tsdf_map_);
}

bool SdfMap::isEsdf() const
{
  return bool(esdf_map_);
}

float SdfMap::getVoxelSize() const
{
  if (tsdf_map_)
  {
    return tsdf_map_->getTsdfLayerPtr()->voxel_size();
  }
  else if (esdf_map_)
  {
    return esdf_map_->getEsdfLayerPtr()->voxel_size();
  }
  return 0;
}


void SdfMap::getSdfMap(std::shared_ptr<voxblox::EsdfMap>& esdf_map)
{
  esdf_map = esdf_map_;
}

void SdfMap::getSdfMap(std::shared_ptr<voxblox::TsdfMap>& tsdf_map)
{
  tsdf_map = tsdf_map_;
}


void SdfMap::loadFromFile(const std::string& file_path)
{
  if (tsdf_map_)
  {
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;
    bool load_map_result = voxblox::io::LoadLayer<voxblox::TsdfVoxel>(file_path, &layer_ptr);

    if (!load_map_result)
    {
      throw std::runtime_error(
        "Could not read tsdf map from file \"" + file_path + "\". For details see voxblox error on console.");
    }

    setLayer(*layer_ptr);
  }
  else if (esdf_map_)
  {
    voxblox::Layer<voxblox::EsdfVoxel>::Ptr layer_ptr;
    // Note: multi layer support required?
    bool load_map_result = voxblox::io::LoadLayer<voxblox::EsdfVoxel>(file_path, true, &layer_ptr);

    if (!load_map_result)
    {
      throw std::runtime_error(
        "Could not read esdf map from file \"" + file_path + "\". For details see voxblox error on console.");
    }

    setLayer(*layer_ptr);
  }
}

void SdfMap::saveToFile(const std::string& file_path, bool clear_file)
{
  bool success = false;
  if (tsdf_map_)
  {
    success = tsdf_map_->getTsdfLayerPtr()->saveToFile(file_path, clear_file);
  }
  else if (esdf_map_)
  {
    success = esdf_map_->getEsdfLayerPtr()->saveToFile(file_path, clear_file);
  }

  if (!success)
  {
    throw std::invalid_argument("Error while saving SdfMap. Please voxblox error for more information.");
  }
}


float SdfMap::getDistanceAtPosition(const Eigen::Vector3d& position, bool interpolate) const
{
  float distance;
  bool success = false;
  if (tsdf_map_)
  {
    success = tsdf_interpolator_->getDistance(position.cast<float>(), &distance, interpolate);
  }
  else if (esdf_map_)
  {
    success = esdf_interpolator_->getDistance(position.cast<float>(), &distance, interpolate);
  }

  if (!success)
  {
    throw std::runtime_error(
      "Could not retrieve distance at position (" +
      std::to_string(position.x()) + ", " + std::to_string(position.y()) + ", " + std::to_string(position.z()) +
      ") from SdfMap."
    );
  }

  return distance;
}

void SdfMap::getGradientAtPosition(const Eigen::Vector3d& position, Eigen::Vector3d& gradient, bool interpolate) const
{
  // TODO SdfMap::getGradientAtPosition (uses Interpolator::getGradient up to now, which is inefficient!)
  // gradient method as in sdf_contact_estimation/.../interpolated_voxblox_esdf::getSdf, computes only distance but can be adapted

  Eigen::Vector3f gradient_f;
  bool success = false;
  if (tsdf_map_)
  {
    success = tsdf_interpolator_->getGradient(position.cast<float>(), &gradient_f, interpolate);
  }
  else if (esdf_map_)
  {
    success = esdf_interpolator_->getGradient(position.cast<float>(), &gradient_f, interpolate);
  }

  if (!success)
  {
    throw std::runtime_error(
      "Could not compute gradient at position (" +
      std::to_string(position.x()) + ", " + std::to_string(position.y()) + ", " + std::to_string(position.z()) +
      ") on SdfMap."
    );
  }

  // cast gradient to required type
  gradient = gradient_f.cast<double>();
}

void SdfMap::createDistancePointcloud(pcl::PointCloud<pcl::PointXYZI>& point_cloud_msg)
{
  if (tsdf_map_)
  {
    voxblox::createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &point_cloud_msg);
  }
  else if (esdf_map_)
  {
    voxblox::createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &point_cloud_msg);
  }
}

void
SdfMap::createDistancePointcloudSlice(pcl::PointCloud<pcl::PointXYZI>& point_cloud_msg, unsigned int free_plane_index,
                                      float slice_level)
{
  if (tsdf_map_)
  {
    voxblox::createDistancePointcloudFromTsdfLayerSlice(tsdf_map_->getTsdfLayer(), free_plane_index,
                                                        slice_level, &point_cloud_msg);
  }
  else if (esdf_map_)
  {
    voxblox::createDistancePointcloudFromEsdfLayerSlice(esdf_map_->getEsdfLayer(), free_plane_index,
                                                        slice_level, &point_cloud_msg);
  }
}
} // end namespace three_dimensional_coverage_path_planning