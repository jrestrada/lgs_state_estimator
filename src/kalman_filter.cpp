#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kalman_filter_node"){
        // Subscribe to the three topics that provide the measurement inputs
        sub1_ = this->create_subscription<std_msgs::msg::Float64>(
            "tether_length", 10, std::bind(&KalmanFilterNode::measurement1_callback, this, std::placeholders::_1));
        sub2_ = this->create_subscription<std_msgs::msg::Float64>(
            "crawler_extensions", 10, std::bind(&KalmanFilterNode::measurement2_callback, this, std::placeholders::_1));
        sub3_ = this->create_subscription<std_msgs::msg::Float64>(
            "imu_integration", 10, std::bind(&KalmanFilterNode::measurement3_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::Float64>("estimated_position_topic", 10);
        // Define system and measurement noise covariance matrices
        // Q = 0.01; // system noise
        // R = 1.0; // measurement noise

        // Define the state vector and its initial estimate
        x_ = Vector2d::Zero(); // [position, velocity]

        // Define the state transition matrix
        F_ = Matrix2d::Identity();
        F_(0, 1) = 1;

        // Define the measurement function
        h_ = [this](const VectorXd& x) -> VectorXd {
            VectorXd z(3); // [position, velocity, acceleration]
            z << x(0), x(1), acceleration_;
            return z;
        };

        // Define the measurement Jacobian matrix
        H_ = [this](const VectorXd& x) -> MatrixXd {
            MatrixXd H(3, 2);
            H << 1, 0, 1, 0, -1, 1;
            return H;
        };

        // Define the process noise covariance matrix
        Qk_ = MatrixXd(2, 2);
        Qk_ << Q, 0, 0, Q;

        // Define the initial state covariance matrix
        P_ = Matrix2d::Identity();
    }

private:
    void measurement1_callback(const std_msgs::msg::Float64::SharedPtr msg){
        measurement1_ = msg->data;
        filter();
    }

    void measurement2_callback(const std_msgs::msg::Float64::SharedPtr msg){
        measurement2_ = msg->data;
        filter();
    }

    void measurement3_callback(const std_msgs::msg::Float64::SharedPtr msg){
        measurement3_ = msg->data;
        filter();
    }

    void filter(){
        if (!measurement1_ || !measurement2_ || !measurement3_) {
            return; // not all measurements have been received yet
        }

        // Define the measurement vector
        VectorXd z(3); // [position, velocity, acceleration]
        z << *measurement1_, *measurement2_, *measurement3_;

        // Define the measurement covariance matrix
        MatrixXd Rk(3, 3);
        Rk << R, 0, 0, 0, R, 0, 0, 0, R;

        // Prediction step
        // Predict the state estimate using the state transition matrix and the previous state estimate
        x_ = F_ * x_;
    
        // Predict the state covariance matrix using the state transition matrix, the previous state covariance matrix, and the process noise covariance matrix
        P_ = F_ * P_ * F_.transpose() + Qk_;
    
        // Update step
        // Compute the innovation vector
        VectorXd y = z - h_(x_);
    
        // Compute the innovation covariance matrix
        MatrixXd S = H_(x_) * P_ * H_(x_).transpose() + Rk;
    
        // Compute the Kalman gain
        MatrixXd K = P_ * H_(x_).transpose() * S.inverse();
    
        // Update the state estimate using the Kalman gain and the innovation vector
        x_ = x_ + K * y;
    
        // Update the state covariance matrix using the Kalman gain and the innovation covariance matrix
        P_ = (MatrixXd::Identity(2, 2) - K * H_(x_)) * P_;
    
        // Reset the measurement flags
        measurement1_.reset();
        measurement2_.reset();
        measurement3_.reset();
    
        // Publish the estimated position
          auto msg = std::make_unique<std_msgs::msg::Float64>();
          msg->data = x_(0);
          pub_->publish(std::move(msg));
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub2_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub3_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;

    double acceleration_ = 0; // constant acceleration assumed
    std::optional<double> measurement1_;
    std::optional<double> measurement2_;
    std::optional<double> measurement3_;

    Vector2d x_; // [position, velocity]
    Matrix2d F_;
    MatrixXd Qk_;
    Matrix2d P_;
    const double Q = 0.01; // system noise
    const double R = 1.0; // measurement noise
    std::function<VectorXd(const VectorXd&)> h_;
    std::function<MatrixXd(const VectorXd&)> H_;
};


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KalmanFilterNode>();
  // node->pub_ = node->create_publisher<std_msgs::msg::Float64>("estimated_position_topic", 10);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}