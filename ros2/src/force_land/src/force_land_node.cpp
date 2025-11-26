#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
    public:
    ForceLand() : Node("force_land"), current_height(0.0), current_nav_state(0), 
                  safety_triggered(false), override_active(false)
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        subscription_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos, std::bind(&ForceLand::height_callback, this, std::placeholders::_1));

        subscription_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status",
            qos, std::bind(&ForceLand::status_callback, this, std::placeholders::_1));

        subscription_land_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected",
            qos, std::bind(&ForceLand::land_detected_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&ForceLand::check_logic, this));
    }

    private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subscription_status_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr subscription_land_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    float current_height;
    uint8_t current_nav_state;
    
    
    bool safety_triggered; 
    bool override_active;  

    const uint8_t NAVIGATION_STATE_AUTO_LAND = 18; 

    
    void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) 
    {
        current_height = -msg->z;
        
        
        std::cout << "Current drone height: " << current_height << " meters" << std::endl;
    }

    void status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
    {
        current_nav_state = msg->nav_state;
    }

    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
    {
        
        if (msg->landed && (safety_triggered || override_active))
        {
            std::cout << ">>> LANDED. System reset. Ready for new flight." << std::endl;
            safety_triggered = false;
            override_active = false;
        }
    }

    void check_logic()
    {
        
        if (override_active) {
            return; 
        }

        
        if (current_height > 20.0)
        {
            
            safety_triggered = true;

            
            if (current_nav_state != NAVIGATION_STATE_AUTO_LAND)
            {
                std::cout << "!!! DRONE HEIGHT > 20m !!! FALL BELOW 20m! !!!" << std::endl;
                send_land_command();
            }
        }
        
        else 
        {
            
            if (safety_triggered)
            {
                
                if (!override_active) {
                    std::cout << ">>> Back in Safe Zone (<20m). Immunity Active. You can now climb back. <<<" << std::endl;
                    override_active = true;
                }
            }
        }
    }

    void send_land_command()
    {
        auto command = px4_msgs::msg::VehicleCommand();
        command.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
        command.target_system = 1;
        command.target_component = 1;
        command.source_system = 1;
        command.source_component = 1;
        command.from_external = true;
        this->publisher_->publish(command);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting Final Node: Strict Enforcement & Immunity Logic..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceLand>());
    rclcpp::shutdown();
    return 0;
}
