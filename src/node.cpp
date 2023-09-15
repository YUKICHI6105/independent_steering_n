#include <independent_steering_n/node.hpp>


int main(int argc, char * argv[])
{
	std::cout << "node started." << std::endl;
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exec{};
	rclcpp::NodeOptions options{};

	std::cout << "initilalizing..." << std::endl;

	auto steering = std::make_shared<nhk2024::independent_steering_n::node::Node>(options);
	exec.add_node(steering);
	
	std::cout << "start spinning." << std::endl;

	exec.spin();
	
	std::cout << "node ended." << std::endl;

	rclcpp::shutdown();
}
