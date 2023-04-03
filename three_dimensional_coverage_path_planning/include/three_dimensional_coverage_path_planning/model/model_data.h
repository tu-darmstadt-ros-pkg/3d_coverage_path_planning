
#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MODEL_DATA_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MODEL_DATA_H

#include "three_dimensional_coverage_path_planning/utils/types.h"

#include "three_dimensional_coverage_path_planning/model/sdf_map.h"

#include <memory>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <octomap/octomap.h>

#include <sensor_msgs/PointCloud2.h>

namespace three_dimensional_coverage_path_planning
{


class ModelData
{
public:
  /**
   *
   * @param nh
   * @param pnh Private node handle. In model_data it will use the subnamespace "model" of pnh's namespace.
   * @param have_point_cloud Whether the point cloud shall be used (computed, load, provided etc.)
   * @param have_sdf See have_point_cloud, for an sdf map (tsdf or esdf, specified in configuration file
   * @param have_octomap See have_point_cloud, for an octomap
   * @param model_name Model name, used as parameter prefix (see setPath) and visualization topic name prefix.
   */
  ModelData(ros::NodeHandle& nh, ros::NodeHandle& pnh, bool have_point_cloud, bool have_sdf, bool have_octomap,
            std::string model_name);

  /**
   * Initialize all models from the given mesh.
   * @param mesh_path Path to mesh file (e.g. obj-File). Can be relative to data_directory or absolute.
   */
  void initFromMesh(const std::string& mesh_path);

  /**
   * Initialize all models from files (load files).
   * @param mesh_path Path to mesh file (e.g. obj-File). Can be relative to data_directory or absolute.
   *    Is only required, if the model file names depend on this name (e.g. all models have the same file name but different extensions).
   *    First the names are always tried to be retrieved from the parameter server and afterwards using this name.
   */
  void initFromFiles(std::string mesh_path = "");

  /**
   * Save all models to files. The output files are placed in the data_directory or if in the places specified on the parameter server when using absolute file paths.
   */
  void saveModels();

  // TODO init but not from existing data but from incoming data (for realworldmodel)
  // TODO possibility to get data (point_cloud, octomap (?), sdf(?)) from sensors/topics --> e.g. setPointCloud(sensor_msgs::PointCLoud2 ...)


  /**
   * @return absolute path to mesh file
   */
  std::string getMeshPath();

  /**
   * @return absolute path to ply mesh file
   */
  std::string getPlyMeshPath();

  /**
   * @return absolute path to point cloud file
   */
  std::string getPointCloudPath();

  /**
   * @return absolute path to sdf map file
   */
  std::string getSdfPath();


  /**
   * @return absolute path to octomap file
   */
  std::string getOctomapPath();


  std::shared_ptr<const sensor_msgs::PointCloud2> getPointCloud() const;

  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> getPointCloudXYZ() const;

  std::shared_ptr<const SdfMap> getSdf() const;

  std::shared_ptr<const octomap::OcTree> getOctomap() const;

  std::string getModelFrame() const;

  std::string getModelName() const;

  Point3d getBBXMin() const;

  Point3d getBBXMax() const;

  /**
   * @return true if the models have been loaded from file, false if they were recomputed.
   */
  bool modelsLoaded() const;


private:
  // set path methods (they all use setPath and the respective get method)
  std::string setAndGetPlyMeshPath();

  std::string setAndGetPointCloudPath();

  std::string setAndGetSdfPath();

  std::string setAndGetOctomapPath();

  void setMeshPath(std::string mesh_path);

  /**
   * Try to retrieve the model file path first from the parameter server,
   * then creates one using the model_file_default_name_ if it is set
   * and at last sets a date time based file name.
   *
   * The parameter names for the model paths will be checked as model_name + "_" + model_type + "_path".
   *
   * @param model_type allowed values: "ply_mesh", "point_cloud", "sdf", "octomap"
   */
  void setPath(const std::string& model_type);

  /**
   * Check if path is absolute or relative and supplements it to an absolute path if relative.
   * Sets model_file_default_name_ if it was not set before using the file name of path.
   * @param path
   */
  void completePath(std::string& path);

  /**
   * computes the bounding box
   */
  void computeBBX();

  // Mesh methods
  /**
   * Convert mesh to ply using the convert_mesh service provided by the mesh_conversions node.
   */
  void convertMeshToPly();

  // point cloud methods
  /**
   * Compute a point cloud from the ply mesh using the mesh_to_sampled_point_cloud package.
   */
  void computePointCloudFromMesh();

  void readPointCloudFromFile();


  // sdf methods
  /**
   * Compute an sdf map from the ply mesh using the mesh_to_sdf package.
   */
  void computeSdfFromMesh();

  void readSdfFromFile();


  // octomap methods
  /**
   * Compute octomap from computed point cloud.
   */
  void resetOctomapFromPointCloud();

  void readOctomapFromFile();


  // publish methods

  void publishPointCloud();

  void publishOctomap();

  void publishSdf();

  void publishSdfPointCloud();

  void publishSdfSlice();


  // general data
  std::string data_directory_;
  std::string model_file_default_name_;
  std::string model_name_;

  float scale_factor_;
  geometry_msgs::Vector3 translation_;
  geometry_msgs::Vector3 rotation_euler_deg_;
  geometry_msgs::Quaternion rotation_quat_;

  float voxel_size_;

  std::string model_frame_;

  bool models_loaded_;

  Point3d bbx_min_;
  Point3d bbx_max_;

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::ServiceClient mesh_conversions_client_;

  ros::Publisher point_cloud_pub_;
  ros::Publisher octomap_pub_;
  ros::Publisher sdf_point_cloud_pub_;
  ros::Publisher sdf_slice_pub_;


  // model file paths
  std::map<std::string, std::string> model_paths_;

  // model data structures
  std::shared_ptr<SdfMap> sdf_map_;
  std::shared_ptr<octomap::OcTree> octomap_;
  std::shared_ptr<sensor_msgs::PointCloud2> point_cloud_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud_pcl_;

  // which model to hold
  bool have_point_cloud_;
  bool have_octomap_;
  bool have_sdf_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MODEL_DATA_H
