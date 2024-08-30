/*************************************************************************
	> File Name: main.cpp
	> Author: Quentin MUNCH
	> Created Time: 29/08/2024
 ************************************************************************/

#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#define PI 3.141592653

using Traj = std::vector<std::array<float, 5>>; // T = [x, y, th, v, w] 
using Obstacle = std::vector<std::array<float, 2>>; // obs = [x_lidar, y_lidar] 
using State = std::array<float, 5>; //
using Window = std::array<float, 4>;
using Point = std::array<float, 2>;
using Control = std::array<float, 2>;

class Config{
public:
  // linear-angular velocity/acceleration bound for the window
  float max_v = 1.0; // m/s
  float min_v = -0.5; // m/s
  float max_w = 40.0 * PI / 180.0; // rad/s
  float max_accel_v = 0.2; // m/s²
  float max_accel_w = 40.0 * PI / 180.0; // rad/s²

  // Measured linear-angular speed resolution 
  float v_reso = 0.01; // m/s
  float w_reso = 0.1 * PI / 180.0; // rad/s

  // Trajectory prediction time and time cst
  float dt = 0.1; // sec
  float predict_time = 3.0; // sec
  
  // Control gain 
  float goal_cost_gain = 1.0;
  float speed_cost_gain = 1.0;

  // Robot parameters
  bool circular_robot = true;
  float robot_radius = 1.0;
  float robot_width = 0.5;
  float robot_lengh = 1.2;
};


// predict next robot position Xt given a command Ut
State motion(State x, Control u, float dt){
  x[2] += u[1] * dt;
  x[0] += u[0] * std::cos(x[2]) * dt;
  x[1] += u[0] * std::sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];
  return x;
};


// Compute dynamic window
Window calc_dynamic_window(State x, Config config){
  return {{
    std::max((x[3] - config.max_accel_v * config.dt), config.min_v),
    std::min((x[3] + config.max_accel_v * config.dt), config.max_v),
    std::max((x[4] - config.max_accel_w * config.dt), -config.max_w),
    std::min((x[4] + config.max_accel_w * config.dt), config.max_w)
  }};
};


// Compute futur robot trajectory given the motion model 
Traj calc_trajectory(State x, float v, float y, Config config){
  Traj traj;
  traj.push_back(x);
  float time = 0.0;
  while (time <= config.predict_time){
    x = motion(x, std::array<float, 2>{{v, y}}, config.dt);
    traj.push_back(x);
    time += config.dt;
  }
  return traj;
};


// Compute obstacle cost given robot footprint and obstacle list 
float calc_obstacle_cost(Traj traj, Obstacle ob, Config config){
  // calc obstacle cost inf: collistion, 0:free
  int skip_n = 2;
  float minr = std::numeric_limits<float>::max();
  for (unsigned int ii=0; ii<traj.size(); ii+=skip_n){
    for (unsigned int i=0; i<ob.size(); i++){
      // get obstacle coordinate
      float ox = ob[i][0];
      float oy = ob[i][1];
      // check if the obstacle fall in the robot circular footprint
      if (config.circular_robot == true){
        float dx = traj[ii][0] - ox;
        float dy = traj[ii][1] - oy;
        float r = std::sqrt(dx*dx + dy*dy);
        if (r <= config.robot_radius){
          return std::numeric_limits<float>::max();
        }
        if (minr >= r){
          minr = r;
        }
      }
      // check if the obstacle fall in the robot rectangular footprint
      else{
        // Rotation matrix components
        float theta = traj[ii][2];
        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);
        // Robot corners in the robot's local frame
        std::array<Point, 4> robot_corners = {
          Point{{config.robot_lengh/2, config.robot_width/2}},
          Point{{config.robot_lengh/2, -config.robot_width/2}},
          Point{{-config.robot_lengh/2, -config.robot_width/2}},
          Point{{-config.robot_lengh/2, config.robot_width/2}}
        };
        // Project corners to the global frame
        for (auto& corner : robot_corners) {
          float x_corner = traj[ii][0] + corner[0] * cos_theta - corner[1] * sin_theta;
          float y_corner = traj[ii][1] + corner[0] * sin_theta + corner[1] * cos_theta;
          corner = {x_corner, y_corner};
        }
        // Check if the obstacle falls within the robot's rectangular footprint
        float x_min = std::min({robot_corners[0][0], robot_corners[1][0], robot_corners[2][0], robot_corners[3][0]});
        float x_max = std::max({robot_corners[0][0], robot_corners[1][0], robot_corners[2][0], robot_corners[3][0]});
        float y_min = std::min({robot_corners[0][1], robot_corners[1][1], robot_corners[2][1], robot_corners[3][1]});
        float y_max = std::max({robot_corners[0][1], robot_corners[1][1], robot_corners[2][1], robot_corners[3][1]});
        if (ox >= x_min && ox <= x_max && oy >= y_min && oy <= y_max) {
          return std::numeric_limits<float>::max(); // Collision detected
        }
        // Compute the minimum distance to the obstacle for cost calculation
        for (const auto& corner : robot_corners) {
          float dx = corner[0] - ox;
          float dy = corner[1] - oy;
          float r = std::sqrt(dx * dx + dy * dy);
          if (minr >= r) {
              minr = r;
          }
        }
      }
    }
  }
  return 1.0 / minr;
};

// Compute cost to goal reaching
float calc_to_goal_cost(Traj traj, Point goal, Config config){
  float goal_magnitude = std::sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
  float traj_magnitude = std::sqrt(std::pow(traj.back()[0], 2) + std::pow(traj.back()[1], 2));
  float dot_product = (goal[0] * traj.back()[0]) + (goal[1] * traj.back()[1]);
  float error = dot_product / (goal_magnitude * traj_magnitude);
  float error_angle = std::acos(error);
  float cost = config.goal_cost_gain * error_angle;

  return cost;
};


Traj calc_final_input(
  State x, Control& u,
  Window dw, Config config, Point goal,
  std::vector<std::array<float, 2>>ob){

    float min_cost = 10000.0;
    Control min_u = u;
    min_u[0] = 0.0;
    Traj best_traj;

    // evalucate all trajectory with sampled input in dynamic window
    for (float v=dw[0]; v<=dw[1]; v+=config.v_reso){
      for (float y=dw[2]; y<=dw[3]; y+=config.w_reso){

        Traj traj = calc_trajectory(x, v, y, config);

        float to_goal_cost = calc_to_goal_cost(traj, goal, config);
        float speed_cost = config.speed_cost_gain * (config.max_v - traj.back()[3]);
        float ob_cost = calc_obstacle_cost(traj, ob, config);
        float final_cost = to_goal_cost + speed_cost + ob_cost;

        if (min_cost >= final_cost){
          min_cost = final_cost;
          min_u = Control{{v, y}};
          best_traj = traj;
        }
      }
    }
    u = min_u;
    return best_traj;
};


Traj dwa_control(State x, Control & u, Config config,
  Point goal, Obstacle ob){
    // # Dynamic Window control
    Window dw = calc_dynamic_window(x, config);
    Traj traj = calc_final_input(x, u, dw, config, goal, ob);

    return u, traj;
  }

cv::Point2i cv_offset(
    float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/2;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
};


int main(){
  State x({{0.0, 0.0, PI/8.0, 0.0, 0.0}});
  Point goal({{10.0,10.0}});
  Obstacle ob({
    {{-1, -1}},
    {{0, 2}},
    {{4.0, 2.0}},
    {{5.0, 4.0}},
    {{5.0, 5.0}},
    {{5.0, 6.0}},
    {{5.0, 9.0}},
    {{8.0, 9.0}},
    {{7.0, 9.0}},
    {{12.0, 12.0}}
  });

  Control u({{0.0, 0.0}});
  Config config;
  Traj traj;
  traj.push_back(x);

  bool terminal = false;

  cv::namedWindow("DWA", cv::WINDOW_NORMAL);
  int count = 0;

  for(int i=0; i<1000 && !terminal; i++){
    Traj ltraj = dwa_control(x, u, config, goal, ob);
    x = motion(x, u, config.dt);
    traj.push_back(x);


    // visualization
    cv::Mat bg(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
    cv::circle(bg, cv_offset(goal[0], goal[1], bg.cols, bg.rows),
               30, cv::Scalar(255,0,0), 5);
    for(unsigned int j=0; j<ob.size(); j++){
      cv::circle(bg, cv_offset(ob[j][0], ob[j][1], bg.cols, bg.rows),
                 20, cv::Scalar(0,0,0), -1);
    }
    for(unsigned int j=0; j<ltraj.size(); j++){
      cv::circle(bg, cv_offset(ltraj[j][0], ltraj[j][1], bg.cols, bg.rows),
                 7, cv::Scalar(0,255,0), -1);
    }
    cv::circle(bg, cv_offset(x[0], x[1], bg.cols, bg.rows),
               30, cv::Scalar(0,0,255), 5);


    cv::arrowedLine(
      bg,
      cv_offset(x[0], x[1], bg.cols, bg.rows),
      cv_offset(x[0] + std::cos(x[2]), x[1] + std::sin(x[2]), bg.cols, bg.rows),
      cv::Scalar(255,0,255),
      7);

    if (std::sqrt(std::pow((x[0] - goal[0]), 2) + std::pow((x[1] - goal[1]), 2)) <= config.robot_radius){
      terminal = true;
      for(unsigned int j=0; j<traj.size(); j++){
        cv::circle(bg, cv_offset(traj[j][0], traj[j][1], bg.cols, bg.rows),
                    7, cv::Scalar(0,0,255), -1);
      }
    }


    cv::resizeWindow("DWA", 640, 480);
    cv::imshow("DWA", bg);
    cv::waitKey(5);

    // std::string int_count = std::to_string(count);
    // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);

    count++;
  }
}
