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
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cmath>

#include <vector>
#include <queue>
#include <mutex>
#include <algorithm>

#include <spdlog/spdlog.h>

#include <laser_grid_mapping/tic_toc.hpp>

#define   END    "\033[0m"
#define   RED    "\033[1;31m"
#define   GREEN  "\033[1;32m"
#define   BLUE   "\033[1;34m"

using namespace std;
using namespace spdlog;

sensor_msgs::LaserScan g_scan_msg;
ros::Publisher g_scan_pub;

double l2p(double l)
{
  return 1 - (1.0 / (1 + exp(l)));
}

double p2l(double p)
{
  return log(p / (1 - p));
}

double pi_to_pi(double angle)
{
  while(angle >= M_PI)
  {
    angle -= 2.*M_PI;
  }

  while(angle < -M_PI)
  {
    angle += 2.*M_PI;
  }

  return angle;
}

class GridMapping
{
public:
  GridMapping(ros::NodeHandle& nh): nh_(nh)
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

    grid_map_msgs_.header.frame_id = map_frame_;
    grid_map_msgs_.info.resolution = map_resolution_;
    grid_map_msgs_.info.width = map_size_x_ / map_resolution_;
    grid_map_msgs_.info.height = map_size_y_ / map_resolution_;
    grid_map_msgs_.info.origin.position.x = map_center_x_;
    grid_map_msgs_.info.origin.position.y = map_center_y_;

    laser_sub_ = nh_.subscribe("/scan", 1, &GridMapping::laserCallback, this);
    odom_sub_ = nh_.subscribe("/odometry", 1, &GridMapping::odomCallback, this);
    grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);

  }
  ~GridMapping()
  {

  }
  queue<nav_msgs::Odometry> odom_msgs_;
  queue<sensor_msgs::LaserScan> laser_msgs_;

  void process()
  {
    TicToc t_whole;

    buf_.lock();
    auto odom_msgs = odom_msgs_.front();
    odom_msgs_.pop();

    auto laser_msgs = laser_msgs_.front();
    laser_msgs_.pop();
    buf_.unlock();

    if (!isGridMappingInitialized) initGridMapParam(laser_msgs);

    // tf
    tf::Quaternion q (odom_msgs.pose.pose.orientation.x,
                      odom_msgs.pose.pose.orientation.y,
                      odom_msgs.pose.pose.orientation.z,
                      odom_msgs.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Relative pose from {W} to {B}
    Eigen::Vector3f d_pose (odom_msgs.pose.pose.position.x, odom_msgs.pose.pose.position.y, yaw);
    cout << "d_pose:\n" << d_pose << endl;

    // Check map update condition
    if ( (d_pose[0] - prev_robot_x_)*(d_pose[0] - prev_robot_x_) 
        + (d_pose[1] - prev_robot_y_)*(d_pose[1] - prev_robot_y_) >= update_movement_*update_movement_)
    {
      updateGridMap(d_pose, laser_msgs.ranges);

      prev_robot_x_ = d_pose[0];
      prev_robot_y_ = d_pose[1];

      // Publish grid map to RViz
      if (map_last_publish_.toSec() + 1.0/map_publish_freq_ < ros::Time::now().toSec())
      {
        map_last_publish_ = ros::Time::now();

        publishGridMapROS();
      }
    }

    info("t_whole: {}{}{}{}", RED, t_whole.toc(), END, "\n");
  }

private:
  bool isGridMappingInitialized = false;
  ros::Time map_last_publish_ = ros::Time();
  string robot_frame_ = "base_link";
  string map_frame_ = "map";
  double prev_robot_x_ = -99999999.0;
  double prev_robot_y_ = -99999999.0;
  
  double sensor_model_p_occ_;
  double sensor_model_p_free_;
  double sensor_model_p_prior_;
  double sensor_model_l_occ_;
  double sensor_model_l_free_;
  double sensor_model_l_prior_;

  double map_center_x_;
  double map_center_y_;
  double map_size_x_;
  double map_size_y_;
  double map_resolution_;
  double map_publish_freq_;

  int num_occupied = 0;
  int num_free = 0;
  int num_valid_laser = 0;

  // [ROS sensor_msgs/LaserScan] http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html
  float angle_min_;  // start angle of the scan [rad]
  float angle_max_;  // end angle of the scan [rad]
  float angle_increment_;  // angular distance between measurements [rad]
  float range_min_;  // minimum range value [m]
  float range_max_;  // maximum range value [m]

  double update_movement_;

  // Grid Map
  vector<vector<double>> grid_map;

  mutex buf_;

  // ROS
  ros::NodeHandle nh_;
  ros::Publisher grid_map_pub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  nav_msgs::OccupancyGrid grid_map_msgs_;

  vector<pair<double, double >> xy2ij (double x, double y)
  {
    vector<pair<double, double>> ij;

    // int i = (x - map_center_x_) / map_resolution_;
    // int j = (y - map_center_y_) / map_resolution_;
    double i = (y - map_center_y_) / map_resolution_;
    double j = (x - map_center_x_) / map_resolution_;
    ij.push_back(make_pair(i, j));

    return ij;
  }

  bool isInside(int i, int j)
  {
    return i >= 0 && j >= 0 && i < grid_map.size() && j < grid_map[0].size();
  }

  void printGridMap()
  {
    cout << fixed << setprecision(2);
    for(const auto& row: grid_map)
    {
      for(const auto& col: row)
      {
        if (col > 0.3)
          cout << BLUE << col << END << " ";
        else
          cout << col << " ";
      }
      cout << endl;
    }
  }

  vector<int8_t> flatten(const vector<vector<double>> &grid_map)
  {
    vector<int8_t> flattened_v;

    for(auto& grid : grid_map)
    {
      flattened_v.insert(flattened_v.end(), grid.begin(), grid.end());
    }

    return flattened_v;
  }

  void publishGridMapROS()
  {
    vector<vector<double>> temp_grid_map(grid_map);
    
    // Convert log-odds to probability form for all grid
    for(int row = 0; row < grid_map.size(); row++)
    {
      for(int col = 0; col < grid_map[row].size(); col++)
      {
        temp_grid_map[row][col] = 100*l2p(grid_map[row][col]);
      }
    }

    // Flatten 2D vector to 1D vector
    auto grid_map_ros = flatten(temp_grid_map);

    // Convert to grid_map_ros messages
    grid_map_msgs_.header.stamp = ros::Time::now();
    grid_map_msgs_.data = grid_map_ros;

    grid_map_pub_.publish(grid_map_msgs_);
  }

  void bresenham(int x0, int y0, int x1, int y1, double r, int temp_count, int index)
  {
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int xp = x0, yp = y0;
    int tx, ty;

    int r0_sq = r*r, r1_sq;
    while (true)
    {

      r1_sq = (xp-x0)*(xp-x0) + (yp-y0)*(yp-y0);
      if ( (xp == x1 && yp == y1) || r1_sq >= r0_sq || !isInside(xp, yp) )
      {
        tx = xp;
        ty = yp;
        break;
      }
      else if (grid_map[xp][yp] == 100)
      {
        tx = xp;
        ty = yp;
        break;
      }

      // Last point in the one scan
      if (temp_count == index)
      {
        trace("(xp, yp): ({}, {})", xp, yp );
      }

      // Plot
      if (isInside(xp, yp))
      {
        // if (temp_count == index) 
        {
          grid_map[xp][yp] += (sensor_model_l_free_ - sensor_model_l_prior_);

          num_free++;
        }
      }

      e2 = 2*err;
      if (e2 >= dy)
      {
        err += dy;
        xp += sx;
      }
      if (e2 <= dx)
      {
        err += dx;
        yp += sy;
      }
    }
    if ( !isnan(r) && r != range_max_ && isInside(tx, ty) )
    {
      grid_map[tx][ty] += (sensor_model_l_occ_ - sensor_model_l_prior_);

      num_occupied++;
    }
  }

  void raycastUpdate(const Eigen::Vector3f& d_pose, double theta, double r, int temp_count, int index)
  {
    // if (isinf(r) && r > 0) r = range_max_;
    if (isinf(r) || isnan(r)) return;

    double x0 = d_pose[0];
    double y0 = d_pose[1];
    double x1 = x0 + r*cos(theta);  // @param theta [rad]
    double y1 = y0 + r*sin(theta);

    double i0 = xy2ij(x0, y0)[0].first;
    double j0 = xy2ij(x0, y0)[0].second;
    double i1 = xy2ij(x1, y1)[0].first;
    double j1 = xy2ij(x1, y1)[0].second;

    // Last point in the one scan
    if (temp_count == index)
    {
      debug("(x0,y0) = ({}, {})", x0, y0);
      debug("(x1,y1) = ({}, {})", x1, y1);
      debug("   r    = {}", r);
      debug("(i0,j0) = ({}, {})", i0, j0);
      debug("(i1,j1) = ({}, {})", i1, j1);
      debug("r / res = {}", r / map_resolution_);
    }

    bresenham(i0, j0, i1, j1, r / map_resolution_, temp_count, index);
  }

  void updateGridMap(const Eigen::Vector3f& d_pose, vector<float>& ranges)
  {
    g_scan_msg.header.frame_id = "base_link";
    g_scan_msg.header.stamp = ros::Time::now();
    g_scan_msg.angle_min = angle_min_;
    g_scan_msg.angle_max = angle_max_;
    g_scan_msg.angle_increment = angle_increment_;
    g_scan_msg.range_min = range_min_;
    g_scan_msg.range_max = range_max_;
    g_scan_msg.ranges.resize(ranges.size());

    sensor_msgs::LaserScan temp_msg;
    temp_msg.header.frame_id = "base_link";
    temp_msg.header.stamp = ros::Time::now();
    temp_msg.angle_min = angle_min_;
    temp_msg.angle_max = angle_max_;
    temp_msg.angle_increment = angle_increment_;
    temp_msg.range_min = range_min_;
    temp_msg.range_max = range_max_;
    temp_msg.ranges.resize(ranges.size());

    double offset = 0.0;
    int k = 0;
    int h = 0;
    bool isFirstMeet = false;
    int index = -1;

    for (int i = ranges.size() - 1; i >= 0; i--)
    {
      double r = ranges[i];
      if (!isnan(r) && !isinf(r))
      {
        if (!isFirstMeet)
        {
          index = i;
        }
        if (!isFirstMeet || h < 10)
        {
          h++;
          isFirstMeet = true;

          float local_x = r * cos(pi_to_pi(d_pose[2] + angle_min_ + i*angle_increment_ + offset));
          float local_y = r * sin(pi_to_pi(d_pose[2] + angle_min_ + i*angle_increment_ + offset));
          double d =  sqrt(local_x*local_x + local_y*local_y);
          trace("local_x: {}", local_x);
          trace("local_y: {}", local_y);
          trace("r: {}", d);
          temp_msg.ranges[i] = d;
        }
      }
      else
      {
        // nan or inf
        temp_msg.ranges[i] = r;
      }
      
      k++;
    }
    g_scan_pub.publish(temp_msg);

    info("num of num_total_laser : {}{}{}", GREEN, ranges.size(), END);
    int i = 0;
    for(auto& r: ranges)
    {
      if (!isinf(r) && !isnan(r)) num_valid_laser++;

      raycastUpdate(d_pose, pi_to_pi(d_pose[2] + angle_min_ + i*angle_increment_), r, i, index);

      i++;
    }

    info("num of num_valid_laser : {}{}{}", GREEN, num_valid_laser, END);
    info("num of occupied cell   : {}{}{}", GREEN, num_occupied, END);
    info("num of free cell       : {}{}{}", GREEN, num_free, END);
  }

  void initGridMapParam(const sensor_msgs::LaserScan& msg)
  {
    sensor_model_l_occ_ = p2l(sensor_model_p_occ_);
    sensor_model_l_free_ = p2l(sensor_model_p_free_);
    sensor_model_l_prior_ = p2l(sensor_model_p_prior_);

    angle_min_ = msg.angle_min;
    angle_max_  = msg.angle_max;
    angle_increment_ = msg.angle_increment;
    range_min_  = msg.range_min;
    range_max_  = msg.range_max;

    isGridMappingInitialized = true;

    grid_map_msgs_.header.frame_id = map_frame_;
    grid_map_msgs_.info.resolution = map_resolution_;
    grid_map_msgs_.info.width   = map_size_x_ / map_resolution_;
    grid_map_msgs_.info.height  = map_size_y_ / map_resolution_;
    grid_map_msgs_.info.origin.position.x = map_center_x_;
    grid_map_msgs_.info.origin.position.y = map_center_y_;

    grid_map.resize(grid_map_msgs_.info.width);
    for(auto& row: grid_map)
    {
      row.resize(grid_map_msgs_.info.height);
    }

    // Set prior to all grid map
    for(int i = 0; i < grid_map_msgs_.info.width; i++)
    {
      for(int j = 0; j < grid_map_msgs_.info.height; j++)
      {
        grid_map[i][j] = sensor_model_l_prior_ * 1;
      }
    }
  }

  void laserCallback(const sensor_msgs::LaserScan::Ptr& msg)
  {
    buf_.lock();
    laser_msgs_.push(*msg);
    buf_.unlock();
  }

  void odomCallback(const nav_msgs::Odometry::Ptr& msg)
  {
    buf_.lock();
    odom_msgs_.push(*msg);
    buf_.unlock();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_grid_mapping_node");
  ros::NodeHandle nh("~");
  ros::Rate r(10);

  spdlog::set_level(spdlog::level::debug);

  g_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/temp_scan", 1);

  GridMapping GM(nh);

  while (ros::ok())
  { 
    ros::spinOnce();

    if (!GM.laser_msgs_.empty() && !GM.odom_msgs_.empty())
    {
      GM.process();
    }
    r.sleep();
  }

  return 0;
}