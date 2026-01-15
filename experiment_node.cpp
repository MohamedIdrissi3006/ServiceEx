#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <algorithm>
#include <thread>

using namespace std::chrono_literals;

class ExperimentNode : public rclcpp::Node
{
public:
    ExperimentNode()
        : Node("experiment_node")
    {
        RCLCPP_INFO(this->get_logger(), "Experiment Node Initialized");

        // -------------------------------------------------
        // SUBSCRIBE TO /robot_estimated_pose
        // -------------------------------------------------
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mcl_pose",
            rclcpp::QoS(10),
            std::bind(&ExperimentNode::odomCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /mcl_pose");

        // -------------------------------------------------
        // SUBSCRIBE TO GROUND TRUTH
        // -------------------------------------------------
        ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_gt",
            rclcpp::QoS(10),
            std::bind(&ExperimentNode::groundTruthCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_gt");


        // -------------------------------------------------
        // Run Experiments
        // -------------------------------------------------
    if(gt_received && est_received){
        MCLExperiment();
        }
        // B2MeasurementNoiseExperiment();
        // B3SimulationParamExperiment();
    }

private:
    // ROS interface
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
    // rclcpp::Client<kalman_positioning::srv::SetUKFParams>::SharedPtr set_params_client_;
bool gt_received = false;
bool est_received = false;
    // Data buffers
    struct GroundTruthEntry {
        double time;
        double x;
        double y;
    };
    std::vector<std::pair<double, double>> estimated_positions_;
    std::vector<GroundTruthEntry> ground_truth_;
    std::vector<std::pair<double, double>> time_series_rmse_;

    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        estimated_positions_.emplace_back(x, y);

        // Find nearest ground truth by timestamp
        if (!ground_truth_.empty())
        {
            auto nearest = std::min_element(
                ground_truth_.begin(), ground_truth_.end(),
                [t](const GroundTruthEntry &a, const GroundTruthEntry &b) {
                    return std::abs(a.time - t) < std::abs(b.time - t);
                });

            est_received = true;
            double dx = x - nearest->x;
            double dy = y - nearest->y;
            double rmse = std::sqrt(dx * dx + dy * dy);
            time_series_rmse_.emplace_back(t, rmse);
        }
    }

    void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        ground_truth_.push_back({t, x, y});
        gt_received = true;
    }

    // Compute RMSE over all collected points
    double computeRMSE()
    {
        if (time_series_rmse_.empty())
            return 0.0;

        double sum = 0.0;
        for (auto &p : time_series_rmse_)
            sum += p.second * p.second;

        return std::sqrt(sum / time_series_rmse_.size());
    }

    // Call service to set UKF parameters
    void setUKFParameters(int particle_num)
    {
        auto request = std::make_shared<mcl_localization::srv::Particles::Request>();
        request->particle_num = particle_num;
     

        auto future = set_params_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Updated Particles Num")
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call set_ukf_params service");
        }
    }

    void resetSimulator()
    {
        RCLCPP_INFO(this->get_logger(), "Simulator reset (placeholder)");
        // Spin for a bit to process any remaining messages
        for (int i = 0; i < 10; i++)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        estimated_positions_.clear();
        ground_truth_.clear();
        time_series_rmse_.clear();
    }

    // NEW: Active data collection function
    void collectDataForDuration(int duration_seconds)
    {
        auto start_time = this->now();
        auto duration = rclcpp::Duration(duration_seconds, 0);
        
        size_t last_count = 0;
        RCLCPP_INFO(this->get_logger(), "Starting data collection for %d seconds...", duration_seconds);
        
        while ((this->now() - start_time) < duration)
        {
            // Actively process callbacks
            rclcpp::spin_some(this->get_node_base_interface());
            
            // Small sleep to prevent CPU spinning
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // Progress update every 10 seconds or 100 samples
            if (estimated_positions_.size() >= last_count + 100)
            {
                last_count = estimated_positions_.size();
                double elapsed = (this->now() - start_time).seconds();
                RCLCPP_INFO(this->get_logger(), 
                           "  Collected %zu samples (%.1f seconds elapsed)...", 
                           estimated_positions_.size(), elapsed);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Data collection complete: %zu samples in %d seconds", 
                   estimated_positions_.size(), duration_seconds);
    }

    // ALTERNATIVE: Collect until minimum samples
    void collectDataUntilSamples(int min_samples, int timeout_seconds = 120)
    {
        auto start_time = this->now();
        auto timeout = rclcpp::Duration(timeout_seconds, 0);
        
        RCLCPP_INFO(this->get_logger(), "Collecting data until %d samples (timeout: %ds)...", 
                   min_samples, timeout_seconds);
        
        while (estimated_positions_.size() < static_cast<size_t>(min_samples))
        {
            if ((this->now() - start_time) > timeout)
            {
                RCLCPP_WARN(this->get_logger(), 
                           "Timeout reached. Only collected %zu samples", 
                           estimated_positions_.size());
                break;
            }
            
            // Actively process callbacks
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // Progress update every 100 samples
            if (estimated_positions_.size() % 100 == 0 && estimated_positions_.size() > 0)
            {
                RCLCPP_INFO(this->get_logger(), "  Collected %zu / %d samples...", 
                           estimated_positions_.size(), min_samples);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Collection complete: %zu samples", 
                   estimated_positions_.size());
    }

    void saveExperimentResults(const std::string &filename)    {
        std::ofstream csv("/home/studamr/ros2_mcl_ws/src/mcl_localization/results/" + filename);
        csv << std::fixed << std::setprecision(5);
       
        csv << "time,x_est,y_est,x_gt,y_gt,rmse\n";

        size_t N = estimated_positions_.size();
        for (size_t i = 0; i < N; i++)
        {
            double t = (i < time_series_rmse_.size()) ? time_series_rmse_[i].first : 0.0;
            double rmse = (i < time_series_rmse_.size()) ? time_series_rmse_[i].second : 0.0;

            double x_gt = 0.0, y_gt = 0.0;
            if (i < estimated_positions_.size() && !ground_truth_.empty())
            {
                // Match by nearest timestamp
                auto nearest = std::min_element(
                    ground_truth_.begin(), ground_truth_.end(),
                    [t](const GroundTruthEntry &a, const GroundTruthEntry &b) {
                        return std::abs(a.time - t) < std::abs(b.time - t);
                    });
                x_gt = nearest->x;
                y_gt = nearest->y;
            }

            csv << t << "," << estimated_positions_[i].first << ","
                << estimated_positions_[i].second << ","
                << x_gt << "," << y_gt << "," << rmse << "\n";
        }
        csv.close();
        
        double final_rmse = computeRMSE();
        RCLCPP_INFO(this->get_logger(), "Saved results to results/%s (RMSE: %.4f)", 
                   filename.c_str(), final_rmse);
    }

    // Experiments
    void MCLExperiment()
    {
    
    std::vector<int> particles_data = {100,200,500,1000,2000};
        
       for(int mn : particles_data){
            resetSimulator();
            setUKFParameters(mn);
            RCLCPP_INFO(this->get_logger(), 
                       "===== MCL Experiment: particles = %d =====", mn);   
                       
            
            
            // Option 1: Collect for fixed duration (recommended)
            collectDataForDuration(15);  // 60 seconds
            
          
            
            saveExperimentResults("MCL_Experiment_100.csv");
        
    }
    }
    // void B2MeasurementNoiseExperiment()
    // {
    //     std::vector<double> measurement_noises = {0.01, 0.05, 0.1, 0.2, 0.5};
    //     double best_process_noise = 0.05;
        
    //     for (double mn : measurement_noises)
    //     {
    //         resetSimulator();
    //         setUKFParameters(best_process_noise, mn);
    //         RCLCPP_INFO(this->get_logger(), 
    //                    "===== B2 Experiment: measurement_noise = %.4f =====", mn);
            
    //         // Option 1: Collect for fixed duration (recommended)
    //         collectDataForDuration(60);  // 60 seconds
            
    //         // Option 2: Collect until minimum samples (alternative)
    //         // collectDataUntilSamples(500);  // 500 samples
            
    //         saveExperimentResults("B2_measurement_noise_" + std::to_string(mn) + ".csv", 
    //                             best_process_noise, mn);
    //     }
    // }

    // void B3SimulationParamExperiment()
    // {
    //     std::vector<double> process_noises = {0.1};
    //     double measurement_noise = 0.5;
        
    //     for (double pn : process_noises)
    //     {
    //         resetSimulator();
    //         setUKFParameters(pn, measurement_noise);
    //         RCLCPP_INFO(this->get_logger(), 
    //                    "===== B3 Experiment  :");
            
    //         // Option 1: Collect for fixed duration (recommended)
    //         collectDataForDuration(10);  // 60 seconds
            

    //         saveExperimentResults("B3_Experiment_5___.0___" + std::to_string(pn) + ".csv", 
    //                             pn, measurement_noise);
    //     }
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExperimentNode>());
    rclcpp::shutdown();
    return 0;
}