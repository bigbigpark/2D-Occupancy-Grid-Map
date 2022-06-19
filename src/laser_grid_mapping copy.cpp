/**
 * @file laser_grid_mapping.cpp
 * @author Seongchang Park (scsc1125@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-15 17:02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

#include <vector>
#include <queue>
#include <algorithm>

#include <laser_grid_mapping/tic_toc.hpp>

#define   END    "\033[0m"
#define   RED    "\033[1;31m"
#define   GREEN  "\033[1;32m"
#define   BLUE   "\033[1;34m"

using namespace std;

bool temp_isFirst = false;

// p(x) = 1 - \frac{1}{1 + e^l(x)}
double l2p(double l)
{
  return 1 - (1.0 / (1 + exp(l)));
}

// Log-odds representation
// l(x) = log(\frac{p(x)}{1 - p(x)})
double p2l(double p)
{
  return log(p / (1 - p));
}

class GridMapping
{
public:
  GridMapping(/* args */)
  {

  }
  ~GridMapping()
  {

  }
private:
  friend class GridMappingROS;

  double map_center_x_; // [m]
  double map_center_y_; // [m]
  double map_size_x_;   // [m]
  double map_size_y_;   // [m]
  double map_resolution_; // [m]
  double laser_min_angle_;  // [m]
  double laser_max_angle_;  // [m]
  double laser_resolution_; // [m]
  double laser_max_dist_;   // [m]
  double sensor_model_l_occ_; 
  double sensor_model_l_free_;  
  double sensor_model_l_prior_; 

  int num_map_rows_;
  int num_map_cols_;

  vector<vector<double>> grid_map_;
  
  void initMapParam(double map_center_x, double map_center_y, double map_size_x, double map_size_y, double map_resolution,
                    double laser_min_angle, double laser_max_angle, double laser_resolution, double laser_max_dist,
                    double sensor_model_p_occ, double sensor_model_p_free, double sensor_model_p_prior)
  {
    map_center_x_         = map_center_x;
    map_center_y_         = map_center_y;
    map_size_x_           = map_size_x;
    map_size_y_           = map_size_y;
    map_resolution_       = map_resolution;
    laser_min_angle_      = laser_min_angle;
    laser_max_angle_      = laser_max_angle;
    laser_resolution_     = laser_resolution;
    laser_max_dist_       = laser_max_dist;
    sensor_model_l_occ_   = p2l(sensor_model_p_occ);
    sensor_model_l_free_  = p2l(sensor_model_p_free);
    sensor_model_l_prior_ = p2l(sensor_model_p_prior);

    num_map_rows_ = map_size_y / map_resolution_;
    num_map_cols_ = map_size_x / map_resolution_;

    // Set grid map size
    grid_map_.resize(num_map_rows_);
    for (auto& grid :  grid_map_)
    {
      grid.resize(num_map_cols_);
    }

    // Set prior to all grid map
    for(int i = 0; i < num_map_rows_; i++)
    {
      for(int j = 0; j < num_map_cols_; j++)
      {
        grid_map_[i][j] = sensor_model_l_prior_ * 1;
      }
    }
  }

  vector<pair<double, double>> ij_to_xy(double i, double j)
  {
    vector<pair<double, double>> xy;
    double x = j*map_resolution_ + map_center_x_;
    double y = i*map_resolution_ + map_center_y_;

    xy.push_back(make_pair(x, y));

    return xy;
  }

  vector<pair<double, double>> xy_to_ij(double x, double y)
  {
    // Transform xy to ij coordinates`
    vector<pair<double, double>> ij;
    double i = (y - map_center_y_) / map_resolution_;
    double j = (x - map_center_x_) / map_resolution_;

    ij.push_back(make_pair(i, j));

    return ij;
  }

  bool isInside(int i, int j)
  {
    // ij값이 Grid map 안에 있는지 검사
    return i < grid_map_.size() && j < grid_map_[0].size() && i >= 0 && j >= 0;
  }

  // Bresenham method is used to plot the lines
  vector<pair<double, double>> bresenham(double i0, double j0, double i1, double j1, double d)
  {
    vector<pair<double, double>> ipjp;

    double dx = fabs(j1 - j0);
    double dy = fabs(i1 - i0);

    double sx = -1;
    if (j0 < j1) sx = 1;

    double sy = -1;
    if (i0 < i1) sy = 1;

    double jp = j0;
    double ip = i0;
    double error = dx + dy;

    while (true)
    {
      // Occupied condition
      if ((jp == j1 && ip == i1) || sqrt((jp-j0)*(jp-j0) + (ip-i0)*(ip-i0)) >= d || !isInside((int) ip, (int) jp))
      {
        ipjp.push_back(make_pair(ip, jp));
        return ipjp;
      }
      else if (grid_map_[(int)ip][(int)jp] == 100)
      {
        ipjp.push_back(make_pair(ip, jp));
        return ipjp;
      }

      // Free space update
      if (isInside(ip, jp))
      {
        grid_map_[(int)ip][(int)jp] += sensor_model_l_free_ - sensor_model_l_prior_;
      }

      double e2 = 2*error;
      if (e2 >= dy)
      {
        error += dy;
        jp += sx;
      }
      if (e2 <= dx)
      {
        error += dx;
        ip += sy;
      }
    }
  }

  void raycast_update(double x0, double y0, double theta, double d, bool isFirst)
  {
    if (isinf(d) || isnan(d)) return;

    /**
     * @brief Transform a laser measurement to the world frame
     */
    double x1, y1;
    x1 = x0 + d*cos(theta);
    y1 = y0 + d*sin(theta);

    if (!temp_isFirst && isFirst)
    {
      cout << "x1    : " << x1 << endl;
      cout << "y1    : " << y1 << endl;
      cout << "theta : " << theta*180.0/M_PI << endl;
      temp_isFirst = true;
    }

    double i0, j0, i1, j1;
    i0 = xy_to_ij(x0, y0)[0].first;
    j0 = xy_to_ij(x0, y0)[0].second;
    i1 = xy_to_ij(x1, y1)[0].first;
    j1 = xy_to_ij(x1, y1)[0].second;

    double d_cells = d / map_resolution_;

    auto ipjp = bresenham(i0, j0, i1, j1, d_cells);

    if ( (!isnan(d)) && d != laser_max_dist_ && isInside(ipjp[0].first, ipjp[0].second) )
    {
      grid_map_[(int)ipjp[0].first][(int)ipjp[0].second] += sensor_model_l_occ_ - sensor_model_l_prior_;
    }

    return;
  }


  vector<vector<double>> update(double x0, double y0, double dtheta, vector<float> ranges)
  {
    int i = 0;
    
    bool isFirst = false;
    for(auto& z : ranges)
    {
      if (!isinf(z) && !isFirst)
      {
        isFirst = true;
        cout << "ranges.size()    : " << ranges.size() << endl;
        cout << "z: " << z << endl;
        cout << "laser_min_angle  : " << laser_min_angle_ << endl;
        cout << "        i        : " << i << endl;
        cout << "laser_resolution : " << laser_resolution_ << endl;
        cout << "     dtheta      :" << laser_min_angle_ + i*laser_resolution_ << endl;
      }
      raycast_update(x0, y0, (dtheta + laser_min_angle_ + i*laser_resolution_), z, isFirst);
      i++;
    }
    
    return grid_map_;
  }
};

class GridMappingROS
{
public:
  GridMappingROS(ros::NodeHandle& nh): nh_(nh)
  {
    nh_.param("grid_map/sensor_model_p_occ", sensor_model_p_occ_, -1.0);
    nh_.param("grid_map/sensor_model_p_free", sensor_model_p_free_, -1.0);
    nh_.param("grid_map/sensor_model_p_prior", sensor_model_p_prior_, -1.0);
    // nh_.param("grid_map/robot_frame", robot_frame_,"base_link");
    // nh_.param("grid_map/map_frame", map_frame_,"map");
    nh_.param("grid_map/map_center_x", map_center_x_, -1.0);
    nh_.param("grid_map/map_center_y", map_center_y_, -1.0);
    nh_.param("grid_map/map_size_x", map_size_x_, -1.0);
    nh_.param("grid_map/map_size_y", map_size_y_, -1.0);
    nh_.param("grid_map/map_resolution", map_resolution_, -1.0);
    nh_.param("grid_map/map_publish_freq", map_publish_freq_, -1.0);
    nh_.param("grid_map/update_movement", update_movement_, -1.0);

    cout << GREEN << "  ==> Initialization succed ! " << END << endl;;
    cout << "p_occ            : " << sensor_model_p_occ_ << endl;
    cout << "p_free           : " << sensor_model_p_free_ << endl;
    cout << "p_prior          : " << sensor_model_p_prior_ << endl;
    cout << "robot_frame      : " << robot_frame_ << endl;
    cout << "map_center_x     : " << map_center_x_ << endl;
    cout << "map_center_y     : " << map_center_y_ << endl;
    cout << "map_size_x       : " << map_size_x_ << endl;
    cout << "map_size_y       : " << map_size_y_ << endl;
    cout << "map_resolution   : " << map_resolution_ << endl;
    cout << "map_publish_freq : " << map_publish_freq_ << endl;
    cout << "update_movement  : " << update_movement_ << endl;

    grid_map_msgs.header.frame_id = map_frame_;
    grid_map_msgs.info.resolution = map_resolution_;
    grid_map_msgs.info.width = map_size_x_ / map_resolution_;
    grid_map_msgs.info.height = map_size_y_ / map_resolution_;
    grid_map_msgs.info.origin.position.x = map_center_x_;
    grid_map_msgs.info.origin.position.y = map_center_y_;

    laser_sub_ = nh_.subscribe("/scan", 1, &GridMappingROS::laserCallback, this);
    grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);

    ros::spin();
  }
  ~GridMappingROS()
  {

  }

private:
  bool isGridMappingInitialized = false;
  ros::Time map_last_publish_ = ros::Time();
  double prev_robot_x_ = -99999999.0;
  double prev_robot_y_ = -99999999.0;
  double sensor_model_p_occ_;
  double sensor_model_p_free_;
  double sensor_model_p_prior_;
  string robot_frame_ = "base_link";
  string map_frame_ = "map";
  double map_center_x_;
  double map_center_y_;
  double map_size_x_;
  double map_size_y_;
  double map_resolution_;
  double map_publish_freq_;
  double update_movement_;
  
  tf::StampedTransform transform;

  // Grid Mapping
  GridMapping GM_;

  // ROS
  ros::NodeHandle nh_;
  ros::Publisher grid_map_pub_;
  ros::Subscriber laser_sub_;
  nav_msgs::OccupancyGrid grid_map_msgs;
  tf::TransformListener tf_listener_;

  void laserCallback(const sensor_msgs::LaserScan::Ptr& msg)
  {
    TicToc t_whole_callback;
    cout << BLUE << "   --- Callback --- " << END << endl;

    if (!isGridMappingInitialized)
    {
      initGridMapping(msg->angle_min, msg->angle_max, msg->angle_increment, msg->range_max);
    }

    tf_listener_.waitForTransform(map_frame_, robot_frame_, msg->header.stamp, ros::Duration(10.0));
    try
    {
      tf_listener_.lookupTransform(map_frame_, robot_frame_, msg->header.stamp, transform);
      auto translation = transform.getOrigin();
      auto rotation = transform.getRotation();
      double roll, pitch, yaw;
  
      tf::Matrix3x3 m(rotation);
      m.getRPY(roll, pitch, yaw);
      
      cout << "x     : " << translation.getX() << endl;
      cout << "y     : " << translation.getY() << endl;
      cout << "z     : " << translation.getZ() << endl;
      cout << "roll  : " << roll << endl;
      cout << "pitch : " << pitch << endl;
      cout << "yaw   : " << yaw * 180.0/M_PI << endl;

      // Check map update condition
      if ( (translation[0] - prev_robot_x_)*(translation[0] - prev_robot_x_) 
          + (translation[1] - prev_robot_y_)*(translation[1] - prev_robot_y_) >= update_movement_*update_movement_)
      {
        auto grid_map = GM_.update(translation[0], translation[1], yaw, msg->ranges);
        prev_robot_x_ = translation[0];
        prev_robot_y_ = translation[1];

        // Publish the grid map if only the condition is satisfied
        if (map_last_publish_.toSec() + 1.0/map_publish_freq_ < ros::Time::now().toSec())
        {
          map_last_publish_ = ros::Time::now();
          publishOccupancyGridMapROS(grid_map, msg->header.stamp);
        }
      }
    }
    catch(const std::exception& e)
    {
      cerr << e.what() << endl;
    }
    cout << "t_whole_callback: " << RED << t_whole_callback.toc() << END << endl;;
  }

  void laserCallbackLocal(const sensor_msgs::LaserScan::Ptr& msg)
  {
    TicToc t_whole_callback;
    cout << BLUE << "   --- Callback --- " << END << endl;

    cout << "t_whole_callback: " << RED << t_whole_callback.toc() << END << endl;
  }

  void publishOccupancyGridMapROS(vector<vector<double>> grid_map, ros::Time& stamp)
  {
    vector<vector<double>> grid_map_p(grid_map);
    vector<vector<int8_t>> grid_map_int_8;

    grid_map_int_8.resize(grid_map.size());
    for(int row = 0; row < grid_map.size(); row++)
    {
      grid_map_int_8[row].resize(grid_map[row].size());

      for(int col = 0; col < grid_map[row].size(); col++)
      {
        // The map data, in row-major order, starting with (0,0).  Occupancy
        // probabilities are in the range [0,100].  Unknown is -1.
        grid_map_p[row][col] = l2p(grid_map[row][col]);
        grid_map_int_8[row][col] = 100 * grid_map_p[row][col];
      }
    }

    // Publish
    int row = 0, col = 0;
    for(int i = 0; i < grid_map_msgs.info.width * grid_map_msgs.info.height; i++)
    {
      if (i > 0 && i % (int) grid_map_msgs.info.width == 0) row++;
      col++;
      if (col > 0 && col % (int) grid_map_msgs.info.height == 0) col -= (int) grid_map_msgs.info.height;

      grid_map_msgs.data.push_back(grid_map_int_8[row][col]);
    }
    // for(int i = 0; i < grid_map_msgs.info.width * 2; i++)
    // {
    //   grid_map_msgs.data.push_back((int8_t) i);
    // }

    grid_map_msgs.header.stamp = stamp;
    grid_map_pub_.publish(grid_map_msgs);

    grid_map_msgs.data.clear();
  }

  void initGridMapping(double angle_min, double angle_max, double angle_increment, double range_max)
  {
    GM_.initMapParam(map_center_x_, map_center_y_, map_size_x_, map_size_y_, map_resolution_, angle_min, angle_max,
    angle_increment, range_max, sensor_model_p_occ_, sensor_model_p_free_, sensor_model_p_prior_);

    isGridMappingInitialized = true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_grid_mapping_node");
  ros::NodeHandle nh("~");

  GridMappingROS GMR(nh);

  return 0;
}