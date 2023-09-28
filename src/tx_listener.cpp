#include <cstring>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <independent_steering_n/msg/tx_float.hpp>
#include <can_plugins2/msg/frame.hpp>

namespace nhk2024::independent_steering_n::tx_listener
{
    using ::independent_steering_n::msg::TxFloat;
    using can_plugins2::msg::Frame;

    class TxListener : public rclcpp::Node
    {
        rclcpp::Publisher<TxFloat>::SharedPtr pub;
        rclcpp::Subscription<Frame>::SharedPtr sub{};

        public:
        TxListener(const rclcpp::NodeOptions& options):
            rclcpp::Node("tx_listener", options),
            pub{this->create_publisher<TxFloat>("converted_can_tx", 10)}
        {
            auto p = this->create_subscription<Frame>("can_tx", 10, std::bind(&TxListener::callback, this, std::placeholders::_1));
            sub = p;
        }

        void callback(const Frame::ConstSharedPtr msg)
        {
            if(msg->dlc != sizeof(float)) return;

            TxFloat converted{};
            converted.id =msg->id;
            std::memcpy(&converted.data, msg->data.data(), sizeof(float));
            pub->publish(converted);
        }
    };
}

int main(int argc, char * argv[])
{
	std::cout << "node started." << std::endl;
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exec{};
	rclcpp::NodeOptions options{};

	std::cout << "initilalizing..." << std::endl;

	auto steering = std::make_shared<nhk2024::independent_steering_n::tx_listener::TxListener>(options);
	exec.add_node(steering);
	
	std::cout << "start spinning." << std::endl;

	exec.spin();
	
	std::cout << "node ended." << std::endl;

	rclcpp::shutdown();
}
