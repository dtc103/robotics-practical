#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <online_statistics/online_statistics.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserListener: public rclcpp::Node{
	public:
		LaserListener();
		void laser_topic_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
		void print_laser_info(sensor_msgs::msg::LaserScan::SharedPtr msg);
		void print_statistics(sensor_msgs::msg::LaserScan::SharedPtr msg);
	private:
		void update_statistics(sensor_msgs::msg::LaserScan::SharedPtr msg);
	private:
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
		std::shared_ptr<OnlineStatistics> statistics;
};

LaserListener::LaserListener():Node("laser_node"){
	this->subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/robot_0/base_scan", 10, std::bind(&LaserListener::laser_topic_callback, this, std::placeholders::_1));
	this->statistics = std::make_shared<OnlineStatistics>();
}

void LaserListener::laser_topic_callback(sensor_msgs::msg::LaserScan::SharedPtr msg){
	print_laser_info(msg);
	update_statistics(msg);
	print_statistics(msg);

}

void LaserListener::print_laser_info(sensor_msgs::msg::LaserScan::SharedPtr msg){
	std::stringstream sstream;
	sstream << "Angle min: " << msg->angle_min << " rad" << std::endl;
	sstream << "Angle max: " <<msg->angle_max << " rad" << std::endl;
	sstream << "Angle increment: " << msg->angle_increment << " rad" << std::endl;

	RCLCPP_INFO(this->get_logger(), "%s", sstream.str().c_str());
}

void LaserListener::print_statistics(sensor_msgs::msg::LaserScan::SharedPtr msg){
	std::stringstream sstream;
	sstream << "Mean distance: " << msg->angle_min << " m" << std::endl;
	sstream << "Mean std: " <<msg->angle_max << "" << std::endl;
	sstream << "Angle increment: " << msg->angle_increment << " rad" << std::endl;

	RCLCPP_INFO(this->get_logger(), "%s", sstream.str().c_str());
}

void LaserListener::update_statistics(sensor_msgs::msg::LaserScan::SharedPtr msg){

}


int main(int argc, char ** argv)
{
	(void) argc;
	(void) argv;

	rclcpp::init(argc, argv);
	auto node = std::make_shared<LaserListener>();
	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
