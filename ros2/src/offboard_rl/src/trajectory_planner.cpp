#include <iostream>
#include <thread>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h>   

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrajectoryPlanner : public rclcpp::Node
{
public:
    TrajectoryPlanner() : Node("trajectory_planner")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos,
            std::bind(&TrajectoryPlanner::vehicle_local_position_callback, this, std::placeholders::_1));

        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            qos,
            std::bind(&TrajectoryPlanner::vehicle_attitude_callback, this, std::placeholders::_1));

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        timer_offboard_ = this->create_wall_timer(
            100ms, std::bind(&TrajectoryPlanner::activate_offboard, this));

        timer_trajectory_publish_ = this->create_wall_timer(
            20ms, std::bind(&TrajectoryPlanner::publish_trajectory_setpoint, this)); // 50 Hz

        keyboard_thread = std::thread(&TrajectoryPlanner::keyboard_listener, this);
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;

    std::thread keyboard_thread;

    bool set_point_received{false};
    bool offboard_active{false};
    bool trajectory_computed{false};

    // coefficients of s(t) = x0 t^5 + ... + x5
    Eigen::Vector<double, 6> x;

    double T{0.0};          // total time of the trajectory
    double t{0.0};          // current time
    Eigen::Vector4d pos_i;  // initial (x,y,z,yaw) 
    Eigen::Vector4d pos_f;  // final (x,y,z,yaw)

    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};
    double offboard_counter{0};

    std::vector<Eigen::Vector4d> user_waypoints_;
    std::vector<Eigen::Vector4d> waypoints_;

    std::vector<double> segment_lengths_;
    std::vector<double> cumulative_lengths_;
    double total_length_{0.0};


    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        current_attitude_ = *msg;
    }


    void keyboard_listener() //inputs from keyboard
    {
        while (rclcpp::ok() && !set_point_received)
        {
            int N = 0;
            std::cout << "Enter number of waypoints (>= 1) and total time T [s]: ";
            {
                std::string line;
                std::getline(std::cin, line);
                std::istringstream iss(line);
                if (!(iss >> N >> T) || N < 1 || T <= 0.0) {
                    std::cout << "Invalid input. Please enter: N (>=1) and T>0.\n";
                    continue;
                }
            }

            user_waypoints_.clear();
            user_waypoints_.reserve(N);

            for (int i = 0; i < N; ++i) {
                double x_, y_, z_, yaw_;
                std::cout << "Enter waypoint " << i+1
                          << " as: x y z yaw (meters, meters, meters, radians): ";
                std::string line;
                std::getline(std::cin, line);
                std::istringstream iss(line);
                if (!(iss >> x_ >> y_ >> z_ >> yaw_)) {
                    std::cout << "Invalid input, please re-enter this waypoint.\n";
                    --i;
                    continue;
                }

                Eigen::Vector4d wp;
                wp(0) = x_;
                wp(1) = y_;
                wp(2) = -z_;   // PX4 uses NED frame
                wp(3) = yaw_;
                user_waypoints_.push_back(wp);
            }

            std::cout << "Waypoints received. Total time T = " << T << " s\n";
            std::cout << "Activating offboard mode soon...\n";
            std::cout << "----------------------------------------" << std::endl;

            set_point_received = true;
        }
    }


    void build_path_from_current_pose()
    {
        waypoints_.clear();
        segment_lengths_.clear();
        cumulative_lengths_.clear();

        // pos_i (x,y,z,yaw)
        pos_i(0) = current_position_.x;
        pos_i(1) = current_position_.y;
        pos_i(2) = current_position_.z;

        auto rpy = utilities::quatToRpy(
            Vector4d(current_attitude_.q[0],
                     current_attitude_.q[1],
                     current_attitude_.q[2],
                     current_attitude_.q[3]));
        pos_i(3) = rpy[2];

        waypoints_.push_back(pos_i);

        for (const auto &wp : user_waypoints_) {
            waypoints_.push_back(wp);
        }

        // pos_f
        pos_f = waypoints_.back();

        // total lenght calculation
        total_length_ = 0.0;
        const int M = static_cast<int>(waypoints_.size());
        segment_lengths_.resize(M - 1);

        for (int i = 0; i < M - 1; ++i) {
            Eigen::Vector3d p0 = waypoints_[i].head<3>();
            Eigen::Vector3d p1 = waypoints_[i+1].head<3>();
            double len = (p1 - p0).norm();
            if (len < 1e-3) len = 1e-3;  
            segment_lengths_[i] = len;
            total_length_ += len;
        }

        cumulative_lengths_.resize(M);
        double s = 0.0;
        for (int i = 0; i < M; ++i) {
            cumulative_lengths_[i] = s;
            if (i < M - 1) {
                s += segment_lengths_[i];
            }
        }

        std::cout << "Path built with " << M << " points, total length s_f = "
                  << total_length_ << " m" << std::endl;
    }


    void activate_offboard()
    {
        if (set_point_received)
        {
            if (offboard_counter == 10) {
                // change to offboard after 1s of messages
                VehicleCommand msg{};
                msg.param1 = 1;
                msg.param2 = 6;
                msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
                msg.target_system = 1;
                msg.target_component = 1;
                msg.source_system = 1;
                msg.source_component = 1;
                msg.from_external = true;
                msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                vehicle_command_publisher_->publish(msg);
  
                msg.param1 = 1.0;
                msg.param2 = 0.0;
                msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
                msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                vehicle_command_publisher_->publish(msg);

                build_path_from_current_pose();

                t = 0.0;
                trajectory_computed = false;

                offboard_active = true;
            }

            OffboardControlMode msg{};
            msg.position = true;
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_publisher_->publish(msg);

            if (offboard_counter < 11)
                offboard_counter++;
        }
    }


    void publish_trajectory_setpoint()
    {
        if (!set_point_received || !offboard_active || t > T) {
            return;
        }

        double dt = 1.0 / 50.0; // 20 ms (50 Hz)
        TrajectorySetpoint msg{compute_trajectory_setpoint(t)};
        trajectory_setpoint_publisher_->publish(msg);
        t += dt;
    }


    TrajectorySetpoint compute_trajectory_setpoint(double t_now)
    {
        if (!trajectory_computed)
        {
            Eigen::VectorXd b(6);
            Eigen::Matrix<double, 6, 6> A;

            // s(0)=0, s'(0)=0, s''(0)=0, s(T)=s_f, s'(T)=0, s''(T)=0
            double s_f = total_length_;

            b << 0.0, 0.0, 0.0, s_f, 0.0, 0.0;

            A << 0,        0,        0,       0,    0, 1,
                 0,        0,        0,       0,    1, 0,
                 0,        0,        0,       1,    0, 0,
                 std::pow(T,5), std::pow(T,4), std::pow(T,3), std::pow(T,2), T, 1,
                 5*std::pow(T,4), 4*std::pow(T,3), 3*std::pow(T,2), 2*T, 1, 0,
                 20*std::pow(T,3), 12*std::pow(T,2), 6*T, 1, 0, 0;

            x = A.inverse() * b;
            trajectory_computed = true;
        }

        // 2) s(t), s_dot(t), s_ddot(t)
        double s, s_d, s_dd;

        double tt = t_now;
        if (tt > T) tt = T;

        s   = x(0) * std::pow(tt, 5.0)
            + x(1) * std::pow(tt, 4.0)
            + x(2) * std::pow(tt, 3.0)
            + x(3) * std::pow(tt, 2.0)
            + x(4) * tt
            + x(5);

        s_d = 5.0 * x(0) * std::pow(tt, 4.0)
            + 4.0 * x(1) * std::pow(tt, 3.0)
            + 3.0 * x(2) * std::pow(tt, 2.0)
            + 2.0 * x(3) * tt
            +       x(4);

        s_dd = 20.0 * x(0) * std::pow(tt, 3.0)
             + 12.0 * x(1) * std::pow(tt, 2.0)
             +  6.0 * x(2) * tt
             +       x(3);

        // saturation
        if (tt >= T - 1e-6) {
            s = total_length_;
            s_d = 0.0;
            s_dd = 0.0;
        }

        Eigen::Vector4d ref_traj_pos, ref_traj_vel, ref_traj_acc;

        if (s <= 0.0) {
            ref_traj_pos = pos_i;
            ref_traj_vel.setZero();
            ref_traj_acc.setZero();
        } else if (s >= total_length_) {
            ref_traj_pos = pos_f;
            ref_traj_vel.setZero();
            ref_traj_acc.setZero();
        } else {
 
            int seg_idx = 0;
            const int M = static_cast<int>(waypoints_.size());
            for (int i = 0; i < M - 1; ++i) {
                if (s >= cumulative_lengths_[i] && s < cumulative_lengths_[i+1]) {
                    seg_idx = i;
                    break;
                }
            }

            double s_i = cumulative_lengths_[seg_idx];
            double seg_len = segment_lengths_[seg_idx];

            double lambda = (s - s_i) / seg_len; // ∈ [0,1]

            Eigen::Vector4d Pi = waypoints_[seg_idx];
            Eigen::Vector4d Pf = waypoints_[seg_idx + 1];

            Eigen::Vector3d pi_xyz = Pi.head<3>();
            Eigen::Vector3d pf_xyz = Pf.head<3>();
            Eigen::Vector3d dir = (pf_xyz - pi_xyz).normalized();

            Eigen::Vector3d pos_xyz = pi_xyz + lambda * (pf_xyz - pi_xyz);
            Eigen::Vector3d vel_xyz = s_d * dir;
            Eigen::Vector3d acc_xyz = s_dd * dir;

            double yaw_i = Pi(3);
            double yaw_f = Pf(3);
            double yaw_err = utilities::angleError(yaw_f, yaw_i);
            double yaw = yaw_i + lambda * yaw_err;

            ref_traj_pos << pos_xyz(0), pos_xyz(1), pos_xyz(2), yaw;
            ref_traj_vel << vel_xyz(0), vel_xyz(1), vel_xyz(2), 0.0;
            ref_traj_acc << acc_xyz(0), acc_xyz(1), acc_xyz(2), 0.0;
        }

        TrajectorySetpoint msg{};
        msg.position = {float(ref_traj_pos(0)),
                        float(ref_traj_pos(1)),
                        float(ref_traj_pos(2))};
        msg.velocity = {float(ref_traj_vel(0)),
                        float(ref_traj_vel(1)),
                        float(ref_traj_vel(2))};
        msg.acceleration = {float(ref_traj_acc(0)),
                            float(ref_traj_acc(1)),
                            float(ref_traj_acc(2))};
        msg.yaw = float(ref_traj_pos(3)); // [-PI:PI]
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        return msg;
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting trajectory_planner node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}
