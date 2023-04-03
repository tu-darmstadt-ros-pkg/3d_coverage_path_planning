#include "three_dimensional_coverage_path_planning/viewpoint_computation/candidate_generator/mesh_navigation_candidate_generator.h"
#include "three_dimensional_coverage_path_planning/utils/transformation_helper.h"
#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <tf2_ros/buffer.h>

#include <pcl/filters/random_sample.h>

namespace three_dimensional_coverage_path_planning
{

void MeshNavigationCandidateGenerator::initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model)
{
  CandidateGeneratorBase::initialize(pnh, model);

  // get parameters
  percentage_candidates_to_keep_ = pnh_.param("percentage_candidates_to_keep", 0.025f);
  min_number_of_candidates_to_keep_ = pnh_.param("min_number_of_candidates_to_keep", 100);
  cost_threshold_ = pnh_.param("cost_threshold", 0.2f);


  // mesh_map has a hard-coded namespace it uses for parameters. So here some parameters need to be changed/copied.
  ros::NodeHandle nh_node_ns("~");

  // set model_frame as global frame for mesh_map
  std::string model_frame = nh_node_ns.param<std::string>("model/model_frame", "building");
  nh_node_ns.setParam("mesh_map/global_frame", model_frame);

  // replace mesh_file with complete path
  std::string data_directory = nh_node_ns.param<std::string>("model/data_directory",
                                                             ros::package::getPath(ROS_PACKAGE_NAME) + "/data/");
  std::string mesh_file = nh_node_ns.param<std::string>("mesh_map/mesh_file", "");

  mesh_file = (mesh_file[0] == '/') ? mesh_file : data_directory + "/" + mesh_file;

  nh_node_ns.setParam("mesh_map/mesh_file", mesh_file);


  // tfBuffer is not used in MeshMap, but is required as argument.
  tf2_ros::Buffer tf_buffer;

  // read map from file defined in "~/mesh_map/mesh_file" with other parameters in namespace "~/mesh_map/"
  mesh_map_ptr_ = boost::make_shared<mesh_map::MeshMap>(tf_buffer);

  if (model_->modelsLoaded())
  {
    mesh_map_ptr_->readMap();
  }
  else
  {
    // if the models have not been loaded but computed, also recompute and save the mesh map layers
    mesh_map_ptr_->readMap(true);
    mesh_map_ptr_->writeMapLayers();
  }
}

void MeshNavigationCandidateGenerator::generateCandidatePositions()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_point_cloud;

  std::vector<float> start_list;
  if (pnh_.getParam("start_point_for_iter_vertices", start_list))
  {
    if (start_list.size() != 3)
    {
      throw std::invalid_argument("Parameter \"start_point_for_iter_vertices\" does not have 3 elements!");
    }
    mesh_map::Vector start(start_list[0], start_list[1], start_list[2]);

    // if a start point is given, use this to iterate the mesh and only use vertices,
    // that are reachable from this without crossing vertices with costs higher a given threshold
    try
    {
      candidates_point_cloud = iterVerticesFromStart(start);
    }
    catch (std::runtime_error& e)
    {
      ROS_WARN_STREAM(
        "Error occurred while trying to generate candidates iterating from given start position. Error was: \""
          << e.what() << "\". Iterating all vertices instead.");
      candidates_point_cloud = iterAllVertices();
    }
  }
  else
  {
    // otherwise, add all vertices as candidates, whose cost are lower than a given threshold with a few additional checks
    candidates_point_cloud = iterAllVertices();
  }

  // TEMPORARY
  ROS_WARN_STREAM("Number of candidates: " << candidates_point_cloud->size());

  pcl::PointCloud<pcl::PointXYZ> filtered_candidates_point_cloud;
  auto num_samples = std::max(static_cast<unsigned int>(std::round(
                                static_cast<float>(candidates_point_cloud->size()) * percentage_candidates_to_keep_)),
                              min_number_of_candidates_to_keep_);

  pcl::RandomSample<pcl::PointXYZ> random;
  random.setInputCloud(candidates_point_cloud);
  random.setSeed(std::rand());
  random.setSample(num_samples);
  random.filter(filtered_candidates_point_cloud);



  // convert from waypoints to viewpoints
  TransformationHelper transformation_helper(pnh_);

  for (auto& pc_waypoint: filtered_candidates_point_cloud)
  {
    Point3d waypoint_position;
    utils::toEigenVector3d(pc_waypoint, waypoint_position);
    Pose3d waypoint(waypoint_position, model_->getModelFrame());
    candidate_viewpoints_.push_back(transformation_helper.transformWaypointToViewpoint(waypoint));

//    ROS_INFO_STREAM("Waypoint: " << utils::to_string(waypoint_position) << ", viewpoint: "
//                                 << utils::to_string(candidate_viewpoints.back().getPose().position));
  }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr MeshNavigationCandidateGenerator::iterAllVertices()
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // add all vertices with costs lower threshold to potential candidate_viewpoints list (as waypoints!)
  for (auto vertex_handle: mesh_map_ptr_->mesh().vertices())
  {
    if (mesh_map_ptr_->vertexCosts()[vertex_handle] > cost_threshold_)
    {
      continue;
    }

    // check if vertex is on the bottom of mesh (z-value of normals of surrounding faces is negative)
    // (there the costs are also below threshold, but robot still cannot drive on the ceiling)
    bool on_bottom = false;

    auto faces = mesh_map_ptr_->mesh().getFacesOfVertex(vertex_handle);
    for (auto& face: faces)
    {
      auto normal = mesh_map_ptr_->faceNormals()[face];

      if (normal.getZ() < 0)
      {
        on_bottom = true;
        break;
      }
    }
    if (on_bottom)
    {
      continue;
    }

    // compute position of vertex
    auto vertex_position = mesh_map_ptr_->mesh().getVertexPosition(vertex_handle);

    // Check if it is on/close to upper bounding box z-value.
    //    This would be the roof and hence not accessible.
    //    Also, after transforming the waypoint to a viewpoint (which is typically higher than the waypoint),
    //    the viewpoint lies outside the models and this results in several problems, e.g. in voxblox.
    //    Other possibility to do this would be checking it after transformation to a viewpoint.
    if ((model_->getBBXMax().z() - vertex_position.z) < 0.05)
    {
      continue;
    }

    // every check earlier passed, so add candidate
    candidates_point_cloud->emplace_back(vertex_position.x, vertex_position.y, vertex_position.z);
  }

  return candidates_point_cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
MeshNavigationCandidateGenerator::iterVerticesFromStart(const mesh_map::Vector& start)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::queue<lvr2::VertexHandle> vertex_queue;

  std::unordered_set<lvr2::VertexHandle> seen_vertices;

  // get vertexHandle of start
  auto start_optional_handle = mesh_map_ptr_->getNearestVertexHandle(start);

  if (!start_optional_handle)
  {
    throw std::runtime_error("No handle found for start point: (" + std::to_string(start.x) + ", " +
                             std::to_string(start.y) + ". " + std::to_string(start.z) + ").");
  }
  auto start_handle = start_optional_handle.unwrap();

  // add start to queue, seen_vertices and candidates
  vertex_queue.push(start_handle);
  seen_vertices.insert(start_handle);

  auto vertex_position_start = mesh_map_ptr_->mesh().getVertexPosition(start_handle);
  candidates_point_cloud->emplace_back(vertex_position_start.x, vertex_position_start.y, vertex_position_start.z);


  if (mesh_map_ptr_->vertexCosts()[start_handle] > cost_threshold_)
  {
    throw std::runtime_error(
      "Costs at start point are greater than threshold! Start position: (" + std::to_string(vertex_position_start.x) +
      ", " + std::to_string(vertex_position_start.y) + ". " + std::to_string(vertex_position_start.z) + "), costs: " +
      std::to_string(mesh_map_ptr_->vertexCosts()[start_handle]) + ", threshold: " + std::to_string(cost_threshold_) +
      ".");
  }
  else
  {
    ROS_INFO_STREAM("Iterate vertices from start: Start position: ("
                      << vertex_position_start.x << ", " << vertex_position_start.y
                      << ". " << vertex_position_start.z << "), costs: " <<
                      mesh_map_ptr_->vertexCosts()[start_handle] << ", threshold: "
                      << cost_threshold_ << ".)");
  }

  while (!vertex_queue.empty())
  {
    // pop first element and add to candidates
    auto current_vertex = vertex_queue.front();
    vertex_queue.pop();


    // get neighbors
    std::vector<lvr2::VertexHandle> neighbors;
    mesh_map_ptr_->mesh().getNeighboursOfVertex(current_vertex, neighbors);

    for (auto& neighbor: neighbors)
    {
      // check if neighbor was already seen (e.g. added to vertex_queue, rejected, etc.)
      if (seen_vertices.find(neighbor) != seen_vertices.end())
      {
        continue;
      }

      // check costs
      if (mesh_map_ptr_->vertexCosts()[neighbor] > cost_threshold_)
      {
        seen_vertices.insert(neighbor);
        continue;
      }

      // if all checks passed, add to queue and seen list
      vertex_queue.push(neighbor);
      seen_vertices.insert(neighbor);

      // add neighbor to candidates list
      auto vertex_position = mesh_map_ptr_->mesh().getVertexPosition(neighbor);
      candidates_point_cloud->emplace_back(vertex_position.x, vertex_position.y, vertex_position.z);
    }
  }

  // check if enough candidates have been generated.
  // If not, this is often a sign that something has gone wrong (e.g. the starting point is (nearly) isolated).
  if (candidates_point_cloud->size() < min_number_of_candidates_to_keep_)
  {
    throw std::runtime_error("Only " + std::to_string(candidates_point_cloud->size()) + " candidates have been generated but at least " + std::to_string(min_number_of_candidates_to_keep_) + " are required (min_number_of_candidates_to_keep)!");
  }


  return candidates_point_cloud;
}
} // end namespace three_dimensional_coverage_path_planning

PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::MeshNavigationCandidateGenerator,
                       three_dimensional_coverage_path_planning::CandidateGeneratorBase)