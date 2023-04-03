#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SDF_MAP_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SDF_MAP_H

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace three_dimensional_coverage_path_planning
{

class SdfMap
{
public:

  /**
   * Create an Esdf map with given configuration.
   * @param config
   */
  explicit SdfMap(const voxblox::EsdfMap::Config& config);

  /**
   * Create an Tsdf map with given configuration.
   * @param config
   */
  explicit SdfMap(const voxblox::TsdfMap::Config& config);

  /**
   * Create an Tsdf map with given layer.
   * @param layer
   */
  explicit SdfMap(const voxblox::Layer<voxblox::TsdfVoxel>& layer);

  /**
   * Create an Esdf map with given layer.
   * @param layer
   */
  explicit SdfMap(const voxblox::Layer<voxblox::EsdfVoxel>& layer);

  /**
   * Set layer of map.
   * @param layer
   */
  void setLayer(const voxblox::Layer<voxblox::TsdfVoxel>& layer);

  /**
   * Set layer of map.
   * @param layer
   */
  void setLayer(const voxblox::Layer<voxblox::EsdfVoxel>& layer);

  bool isTsdf() const;

  bool isEsdf() const;

  float getVoxelSize() const;

  /**
   * Get the tsdf map. Nullptr if isTsdf() is false.
   * @param tsdf_map
   */
  void getSdfMap(std::shared_ptr<voxblox::TsdfMap>& tsdf_map);

  /**
   * Get the esdf map. Nullptr if isEsdf() is false.
   * @param esdf_map
   */
  void getSdfMap(std::shared_ptr<voxblox::EsdfMap>& esdf_map);

  /**
   * Load tsdf or esdf map from file.
   * See voxblox::io::LoadLayer.
   * @param file_path
   * @throw std::runtime_error when an error occurs while loading
   */
  void loadFromFile(const std::string& file_path);

  /**
   * Save map to file.
   * See voxblox::Layer::saveToFile.
   * @param file_path
   * @param clear_file
   * @throw std::runtime_error when an error occurs while saving the file
   */
  void saveToFile(const std::string& file_path, bool clear_file);


  /**
   * Get the distance value at the given position.
   * See voxblox::Interpolator::getDistance.
   * @param position
   * @param interpolate true if interpolation should be used.
   * @return distance at position
   * @throw std::runtime_error if distance at given position could not be retrieved
   */
  float getDistanceAtPosition(const Eigen::Vector3d& position, bool interpolate = true) const;

  /**
   * Get the gradient at the given position.
   * Gradients are computed using central differences.
   * @param [in] position
   * @param [out] gradient
   * @param [in] interpolate
   * @throw std::runtime_error if gradient at given position could not be computed.
   */
  void getGradientAtPosition(const Eigen::Vector3d& position, Eigen::Vector3d& gradient, bool interpolate = true) const;


  /**
   * Create point cloud with distance as intensity values.
   * See voxblox::createDistancePointcloudFromTsdfLayer/-EsdfLayer.
   * @param [out] point_cloud_msg
   */
  void createDistancePointcloud(pcl::PointCloud<pcl::PointXYZI>& point_cloud_msg);

  /**
   * Create point cloud slice with distance as intensity values.
   * See voxblox::createDistancePointcloudFromTsdfLayerSlice/-EsdfLayerSlice.
   * @param [out] point_cloud_msg
   * @param [in] free_plane_index 0 = x-axis, 1 = y-axis, 2 = z-axis set to slice-level value
   * @param [in] slice_level
   */
  void createDistancePointcloudSlice(pcl::PointCloud<pcl::PointXYZI>& point_cloud_msg, unsigned int free_plane_index,
                                     float slice_level);


private:

  std::shared_ptr<voxblox::EsdfMap> esdf_map_;
  std::shared_ptr<voxblox::TsdfMap> tsdf_map_;

  std::shared_ptr<voxblox::Interpolator<voxblox::TsdfVoxel>> tsdf_interpolator_;
  std::shared_ptr<voxblox::Interpolator<voxblox::EsdfVoxel>> esdf_interpolator_;

};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SDF_MAP_H
