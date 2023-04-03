#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_UTILS_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_UTILS_H

#include <ros/node_handle.h>

#include <Eigen/Geometry>
#include <octomap/octomap_types.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

namespace three_dimensional_coverage_path_planning
{

namespace utils
{

// === ros specifics === //
/**
 * Search a parameter (see ros::NodeHandle::searchParam) and retrieve its value into result.
 * If it cannot be found or retrieved, result will be set to the given default value.
 * @tparam T parameter type
 * @param nh node handle on which the parameter should be searched
 * @param param_name
 * @param [in] default_value
 * @param [out] result
 */
template<typename T>
static void searchAndGetParam(ros::NodeHandle& nh, const std::string& param_name, const T& default_value, T& result)
{
  std::string complete_param_name;
  if (!nh.searchParam(param_name, complete_param_name) || !nh.getParam(complete_param_name, result))
  {
    result = default_value;
  }
  //ROS_ERROR_STREAM("Search param: " << param_name << ", namespace: " << nh.getNamespace() << ", complete_param_name: " << complete_param_name << ", result value: " << result);
}


/**
 * Search a parameter (see ros::NodeHandle::searchParam) and retrieve its value into result.
 * If it cannot be found or retrieved, an error is thrown.
 * @tparam T parameter type
 * @param nh node handle on which the parameter should be searched
 * @param param_name
 * @param [out] result
 */
template<typename T>
static void searchAndGetParam(ros::NodeHandle& nh, const std::string& param_name, T& result)
{
  std::string complete_param_name;
  if (!nh.searchParam(param_name, complete_param_name) || !nh.getParam(complete_param_name, result))
  {
    throw std::runtime_error(
      "Parameter " + param_name + " could not be found in searchAndGetParam, starting in namespace " +
      nh.getNamespace() + ".");
  }
//  ROS_ERROR_STREAM("Search param: " << param_name << ", namespace: " << nh.getNamespace() << ", complete_param_name: "
//                                    << complete_param_name << ", result value: " << result << ".");
}



// === point conversions === //

static void toEigenVector3d(const pcl::PointXYZ& input, Eigen::Vector3d& output)
{
  output.x() = input.x;
  output.y() = input.y;
  output.z() = input.z;
}

static void toEigenVector3d(const geometry_msgs::Point& input, Eigen::Vector3d& output)
{
  output.x() = input.x;
  output.y() = input.y;
  output.z() = input.z;
}

static void toPclPoint(const Eigen::Vector3d& input, pcl::PointXYZI& output)
{
  output.x = static_cast<float>(input.x());
  output.y = static_cast<float>(input.y());
  output.z = static_cast<float>(input.z());
}


static void toOctomapPoint3d(const pcl::PointXYZ& input, octomap::point3d& output)
{
  output.x() = static_cast<float>(input.x);
  output.y() = static_cast<float>(input.y);
  output.z() = static_cast<float>(input.z);
}

static void toOctomapPoint3d(const geometry_msgs::Point& input, octomap::point3d& output)
{
  output.x() = static_cast<float>(input.x);
  output.y() = static_cast<float>(input.y);
  output.z() = static_cast<float>(input.z);
}

static void toOctomapPoint3d(const Eigen::Vector3d& input, octomap::point3d& output)
{
  output.x() = static_cast<float>(input.x());
  output.y() = static_cast<float>(input.y());
  output.z() = static_cast<float>(input.z());
}


static void toMsgPoint(const Eigen::Vector3d& input, geometry_msgs::Point& output)
{
  output.x = input.x();
  output.y = input.y();
  output.z = input.z();
}

static void toMsgPoint(const octomap::point3d& input, geometry_msgs::Point& output)
{
  output.x = input.x();
  output.y = input.y();
  output.z = input.z();
}

static geometry_msgs::Point toMsgPoint(const Eigen::Vector3d& input)
{
  geometry_msgs::Point output;
  output.x = input.x();
  output.y = input.y();
  output.z = input.z();
  return output;
}

static geometry_msgs::Point toMsgPoint(const octomap::point3d& input)
{
  geometry_msgs::Point output;
  output.x = input.x();
  output.y = input.y();
  output.z = input.z();
  return output;
}


// === rotation conversions === //

static void toMsgQuaternion(const Eigen::Quaterniond& input, geometry_msgs::Quaternion& output)
{
  output.x = input.x();
  output.y = input.y();
  output.z = input.z();
  output.w = input.w();
}

static geometry_msgs::Quaternion toMsgQuaternion(const Eigen::Quaterniond& input)
{
  geometry_msgs::Quaternion output;
  output.x = input.x();
  output.y = input.y();
  output.z = input.z();
  output.w = input.w();
  return output;
}

static void toEigenQuaternion(const geometry_msgs::Quaternion& input, Eigen::Quaterniond& output)
{
  output.x() = input.x;
  output.y() = input.y;
  output.z() = input.z;
  output.w() = input.w;
}

/**
 * Convert euler angles rotation (i.e. three independent rotations around the original axes) to quaternion.
 * @param [in] euler_deg euler angles in degrees
 * @param [out] quaternion
 */
static void rotationEulerToQuaternion(const geometry_msgs::Vector3& euler_deg, geometry_msgs::Quaternion& quaternion)
{
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(euler_deg.x * M_PI / 180, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(euler_deg.y * M_PI / 180, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(euler_deg.z * M_PI / 180, Eigen::Vector3d::UnitZ());

  toMsgQuaternion(q, quaternion);
}

/**
 * Convert euler angles rotation (i.e. three independent rotations around the original axes) to quaternion.
 * @param [in] euler euler angles in radian
 * @param [out] quaternion
 */
static void rotationEulerToQuaternion(const Eigen::Vector3d& euler, Eigen::Quaterniond& quaternion)
{
  quaternion = Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX())
               * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());
}



// === math methods === //

/**
 * Rounds a given variable to the specified number of decimal places.
 * @param [in,out] inout
 * @param [in] decimal_places
 */
static void round(double& inout, int decimal_places = 0)
{
  int factor = static_cast<int>(std::pow(10, decimal_places));
  inout = std::round(inout * factor) / factor;
}

/**
 * Rounds each element of a given vector to the specified number of decimal places.
 * @param [in,out] inout
 * @param [in] decimal_places
 */
static void round(Eigen::Vector3d& inout, int decimal_places = 0)
{
  inout = inout.unaryExpr([decimal_places](double d)
                          {
                            utils::round(d, decimal_places);
                            return d;
                          });
}

/**
 * Rounds each element of a given vector to the specified number of decimal places.
 * @param [in,out] inout
 * @param [in] decimal_places
 */
static void floor(Eigen::Vector2d& inout)
{
  inout = inout.unaryExpr([](double d) { return std::floor(d); });
}

/**
 * Returns the maximum element of the absolute value of the given vector.
 * @param input
 * @return
 */
static double absMax(const Eigen::Vector3d& input)
{
  auto abs_vector = input.cwiseAbs();
  return abs_vector.maxCoeff();
}

/**
 * Computes the angle between a vector and a plane.
 * @param plane_normal Normal of plane.
 * @param vector
 * @return
 */
static double anglePlaneVector(const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& vector)
{
  return std::asin(plane_normal.dot(vector) / (plane_normal.norm() * vector.norm()));
}


static double distanceBetweenPositions(const geometry_msgs::Point& pos1, const geometry_msgs::Point& pos2)
{
  return std::sqrt(std::pow((pos1.x - pos2.x), 2) + std::pow((pos1.y - pos2.y), 2) + std::pow((pos1.z - pos2.z), 2));
}


// === comparator methods === //

/**
 * Elementwise comparison of lhs and rhs, returns comparison of first unequal elements.
 * If all elements are equal, 0 is returned.
 * @param lhs
 * @param rhs
 * @param precision every difference between elements of lhs and rhs < precision are counted as equal
 * @return negative value, zero, or a positive value if lhs is less than, equal to, or greater than rhs
 */
static double compare(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs, double precision = 0)
{
  Eigen::Vector3d difference = lhs - rhs;
  for (int i = 0; i < 3; i++)
  {
    if (std::abs(difference[i]) > precision)
    {
      return difference[i];
    }
  }
  return 0;
}

/**
 * Elementwise comparison of lhs and rhs, returns comparison of first unequal elements.
 * If all elements are equal, 0 is returned.
 * @param lhs
 * @param rhs
 * @param precision every difference between elements of lhs and rhs < precision are counted as equal
 * @return negative value, zero, or a positive value if lhs is less than, equal to, or greater than rhs
 */
static double compare(const Eigen::Quaterniond& lhs, const Eigen::Quaterniond& rhs, double precision = 0)
{
  Eigen::Vector4d difference = lhs.coeffs() - rhs.coeffs();
  for (int i = 0; i < 4; i++)
  {
    if (std::abs(difference[i]) < precision)
    {
      return difference[i];
    }
  }
  return 0;
}



// === to_string methods ===

static std::string to_string(const geometry_msgs::Point& p)
{
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + ")";
}

static std::string to_string(const geometry_msgs::Quaternion& p)
{
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + ", " +
         std::to_string(p.w) + ")";
}
} // end namespace utils


} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_UTILS_H
