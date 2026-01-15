#include "mcl_localization/mcl.hpp"
#include "mcl_localization/landmark_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>

class MCLNode : public rclcpp::Node
{
public:
  MCLNode()
  : Node("mcl_node"), mcl_(500)
  {
    mcl_.initializeUniform(-10.0, 10.0, -10.0, 10.0);
    
    // Load landmarks from CSV file
    std::string csv_path = "/home/studamr/ros2_mcl_ws/src/mcl_localization/landmarks.csv";
    if (!landmark_mgr_.loadFromCSV("/home/studamr/ros2_mcl_ws/src/mcl_localization/landmarks.csv"))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load landmarks from: %s", csv_path.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu landmarks from CSV", 
                  landmark_mgr_.getLandmarks().size());
    }
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/robot_noisy", 10,
      std::bind(&MCLNode::odomCallback, this, std::placeholders::_1));
      
    landmarks_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/landmarks_observed", 10,
      std::bind(&MCLNode::landmarksCallback, this, std::placeholders::_1));
      
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mcl_pose", 10);
      
    // Publisher for top 10 particles
    particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/mcl_particles_top40", 10);

      set_params_service_ = this->create_service<mcl_localization::srv::Particles>(
            "Particles",
            [this](const std::shared_ptr<mcl_localization::srv::Particles::Request> request,
                   std::shared_ptr<mcl_localization::srv::Particles::Response> response)
            {
                RCLCPP_INFO(this->get_logger(), "UKF Params updated: ");
                  // update the number of partices
              mcl_.setNumParticles(request->particle_num);

                response->success = true;
            });
  }

private:
  MCL mcl_;
  LandmarkManager landmark_mgr_;  // Landmark manager for CSV loading
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Service<mcl
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double last_theta_ = 0.0;
  bool first_odom_ = true;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    // FIXED: Proper quaternion to yaw conversion
    auto q = msg->pose.pose.orientation;
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double theta = std::atan2(siny, cosy);

    if (!first_odom_)
    {
      double dx = x - last_x_;
      double dy = y - last_y_;
      double dtheta = theta - last_theta_;
      
      // Normalize angle difference
      while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
      while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
      
      mcl_.motionUpdate(dx, dy, dtheta);
    }
    
    first_odom_ = false;
    last_x_ = x;
    last_y_ = y;
    last_theta_ = theta;
  }

  void landmarksCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Check if landmarks are loaded
    if (landmark_mgr_.getLandmarks().empty())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No landmarks loaded from CSV file");
      return;
    }
    
    // Parse observations from PointCloud2
    std::vector<int> obs_ids;
    std::vector<double> obs_x, obs_y;
    
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    
    // Check if ID field exists in PointCloud2
    bool has_id = false;
    for (const auto& field : msg->fields)
    {
      if (field.name == "id")
      {
        has_id = true;
        break;
      }
    }
    
    if (has_id)
    {
      // Parse with IDs (preferred method)
      sensor_msgs::PointCloud2ConstIterator<int> iter_id(*msg, "id");
      
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_id)
      {
        obs_ids.push_back(*iter_id);
        obs_x.push_back(*iter_x);
        obs_y.push_back(*iter_y);
      }
    }
    else
    {
      // No IDs in PointCloud - assume sequential ordering matches CSV
      RCLCPP_WARN_ONCE(this->get_logger(), 
                       "PointCloud2 has no 'id' field - assuming sequential IDs");
      int id = 0;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
      {
        obs_ids.push_back(id++);
        obs_x.push_back(*iter_x);
        obs_y.push_back(*iter_y);
      }
    }
    
    // Match observations to landmarks from CSV
    std::vector<double> matched_obs_x, matched_obs_y;
    std::vector<double> matched_lm_x, matched_lm_y;
    
    for (size_t i = 0; i < obs_ids.size(); ++i)
    {
      int id = obs_ids[i];
      
      // Check if this landmark exists in our CSV map
      if (landmark_mgr_.hasLandmark(id))
      {
        auto lm_pos = landmark_mgr_.getLandmark(id);
        
        matched_obs_x.push_back(obs_x[i]);
        matched_obs_y.push_back(obs_y[i]);
        matched_lm_x.push_back(lm_pos.first);   // x from CSV
        matched_lm_y.push_back(lm_pos.second);  // y from CSV
      }
      else
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Observed landmark ID %d not found in CSV map", id);
      }
    }
    
    // Only update if we have valid matches
    if (!matched_obs_x.empty())
    {
      mcl_.measurementUpdate(matched_obs_x, matched_obs_y, 
                            matched_lm_x, matched_lm_y, 1);
      mcl_.resample();
      
      // Publish estimated pose
      auto est = mcl_.estimatePose();
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now();
      pose.header.frame_id = "map";
      pose.pose.position.x = est.x;
      pose.pose.position.y = est.y;
      pose.pose.orientation.z = std::sin(est.theta / 2.0);
      pose.pose.orientation.w = std::cos(est.theta / 2.0);
      pose_pub_->publish(pose);
      
      // Publish top 10 particles
      publishTopParticles();
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No landmark matches found - cannot update MCL");
    }
  }
  
  void publishTopParticles()
  {
    auto particles = mcl_.particles();
    
    // Sort particles by weight (descending)
    std::vector<MCL::Particle> sorted_particles = particles;
    std::sort(sorted_particles.begin(), sorted_particles.end(),
              [](const MCL::Particle& a, const MCL::Particle& b) {
                return a.weight > b.weight;
              });
    
    // Create PoseArray with top 10
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = now();
    pose_array.header.frame_id = "map";
    
    int count = std::min(1, static_cast<int>(sorted_particles.size()));
    for (int i = 0; i < count; ++i)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = sorted_particles[i].x;
      pose.position.y = sorted_particles[i].y;
      pose.orientation.z = std::sin(sorted_particles[i].theta / 2.0);
      pose.orientation.w = std::cos(sorted_particles[i].theta / 2.0);
      pose_array.poses.push_back(pose);
    }
    
    particles_pub_->publish(pose_array);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCLNode>());
  rclcpp::shutdown();
  return 0;
}