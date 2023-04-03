#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_COLORS_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_COLORS_H

#include <std_msgs/ColorRGBA.h>


namespace three_dimensional_coverage_path_planning
{

namespace colors
{
// the colors need to be methods, as std_msgs::ColorRGBA does not have a constructor with rgba values and hence cannot be initialized otherwise

static std_msgs::ColorRGBA color255(float r, float g, float b, float a = 255)
{
  std_msgs::ColorRGBA color;
  color.r = r / 255.0f;
  color.g = g / 255.0f;
  color.b = b / 255.0f;
  color.a = (a == 1) ? 1.0f : (a / 255.0f);
  return color;
}


static std_msgs::ColorRGBA color(float r, float g, float b, float a = 1.0f)
{
  if (r > 1 || g > 1 || b > 1 || a > 1)
  {
    return color255(r, g, b, a);
  }

  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}


static std_msgs::ColorRGBA red()
{
  return color(255, 0, 0);
}

static std_msgs::ColorRGBA green()
{
  return color(0, 255, 0);
}

static std_msgs::ColorRGBA blue()
{
  return color(0, 0, 255);
}

static std_msgs::ColorRGBA yellow()
{
  return color(255, 255, 0);
}

static std_msgs::ColorRGBA violet()
{
  return color(255, 0, 255);
}

static std_msgs::ColorRGBA cyan()
{
  return color(0, 255, 255);
}

static std_msgs::ColorRGBA orange()
{
  return color(255, 128, 0);
}

/**
 * Convert hsv color to rgb color.
 * @param h in [0, 360]
 * @param s in [0,1]
 * @param v in [0,1]
 * @return
 */
static std_msgs::ColorRGBA hsvToRgb(float h, float s, float v)
{
  if (h < 0 || h > 360 || s < 0 || s > 1 || v < 0 || v > 1)
  {
    ROS_ERROR_STREAM("The given HSV values are not valid!");
    return {};
  }

  float C = s * v;
  float h1 = h / 60;
  float X = C * (1 - std::abs(std::fmod(h1, 2) - 1));

  float r1, g1, b1;
  if (0 <= h1 && h1 < 1)
  {
    r1 = C;
    g1 = X;
    b1 = 0;
  }
  else if (1 <= h1 && h1 < 2)
  {
    r1 = X;
    g1 = C;
    b1 = 0;
  }
  else if (2 <= h1 && h1 < 3)
  {
    r1 = 0;
    g1 = C;
    b1 = X;
  }
  else if (3 <= h1 && h1 < 4)
  {
    r1 = 0;
    g1 = X;
    b1 = C;
  }
  else if (4 <= h1 && h1 < 5)
  {
    r1 = X;
    g1 = 0;
    b1 = C;
  }
  else if (5 <= h1 && h1 < 6)
  {
    r1 = C;
    g1 = 0;
    b1 = X;
  }

  float m = v - C;

  std_msgs::ColorRGBA rgba;
  rgba.a = 1;
  rgba.r = r1 + m;
  rgba.g = g1 + m;
  rgba.b = b1 + m;

  return rgba;
}


static std_msgs::ColorRGBA generateColor(float value, float min, float max)
{
  // Do not use complete color palette but crop at 5/6 of 360°, in order to ensure that highest and lowest hue values are not the same.
  // See https://answers.ros.org/question/182444/rviz-colorbar/
  return hsvToRgb(300 * ((value - min) / (max - min)), 1.0f, 1.0f);
}

/**
 * Generate a given number of equidistant colors.
 */
class ColorGenerator
{
public:
  explicit ColorGenerator(int num_colors)
  {
    // Do not use complete color palette but crop at 5/6 of 360°, in order to ensure that first and last color are not the same.
    // See https://answers.ros.org/question/182444/rviz-colorbar/
    inc_ = 300.0f / static_cast<float>(num_colors);

    // set to -inc so that first color given by nextColor has hue 0
    last_hue_ = -inc_;
  }

  std_msgs::ColorRGBA nextColor()
  {
    last_hue_ = last_hue_ + inc_;

    // reset if overflow
    if (last_hue_ > 360)
    {
      last_hue_ = 0;
    }

    return hsvToRgb(last_hue_, 1.0f, 1.0f);
  }

private:
  float inc_;
  float last_hue_;
};
} // end namespace colors
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_COLORS_H
