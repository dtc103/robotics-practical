#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <online_statistics/online_statistics.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

//#define RAY_DEBUG

class LaserListener: public rclcpp::Node{
	public:
		LaserListener();
		void laser_topic_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
		void print_laser_info(sensor_msgs::msg::LaserScan::SharedPtr msg);
		void print_statistics(sensor_msgs::msg::LaserScan::SharedPtr msg);
	private:
		void update_statistics(sensor_msgs::msg::LaserScan::SharedPtr msg);
		void print_5_closest_ray_idxs(sensor_msgs::msg::LaserScan::SharedPtr msg);
	private:
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
		std::shared_ptr<OnlineStatistics> statistics;
		size_t const ray_idx = 124;
};

LaserListener::LaserListener():Node("laser_node"){
	this->subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/robot_0/base_scan", 10, std::bind(&LaserListener::laser_topic_callback, this, std::placeholders::_1));
	this->statistics = std::make_shared<OnlineStatistics>();
}

void LaserListener::laser_topic_callback(sensor_msgs::msg::LaserScan::SharedPtr msg){
#ifndef RAY_DEBUG
	print_laser_info(msg);
	update_statistics(msg);
	print_statistics(msg);
#else
	this->print_5_closest_ray_idxs(msg);
#endif
}

void LaserListener::print_laser_info(sensor_msgs::msg::LaserScan::SharedPtr msg){
	std::stringstream sstream;
	sstream << "Angle min: " << msg->angle_min << " rad" << std::endl;
	sstream << "Angle max: " <<msg->angle_max << " rad" << std::endl;
	sstream << "Angle increment: " << msg->angle_increment << " rad" << std::endl;

	sstream << "num of rays: " << std::vector<double>(std::begin(msg->ranges), std::end(msg->ranges)).size() << std::endl;

	RCLCPP_INFO(this->get_logger(), "%s", sstream.str().c_str());
}

void LaserListener::print_statistics(sensor_msgs::msg::LaserScan::SharedPtr msg){
	std::stringstream sstream;
	sstream << "Mean distance: " << this->statistics->getMean() << " m" << std::endl;
	sstream << "Mean std: " << this->statistics->getStandardDeviation() << "" << std::endl;
	sstream << "Distance for ray "<< this->ray_idx << ": " << msg->ranges[this->ray_idx] << " m" << std::endl;

	RCLCPP_INFO(this->get_logger(), "%s", sstream.str().c_str());
}

void LaserListener::update_statistics(sensor_msgs::msg::LaserScan::SharedPtr msg){
	if (std::isnan(msg->ranges[this->ray_idx]) || msg->ranges[this->ray_idx] < msg->range_min || msg->ranges[this->ray_idx] > msg->range_max){
		return;
	}
	this->statistics->update(msg->ranges[this->ray_idx]);
}

void LaserListener::print_5_closest_ray_idxs(sensor_msgs::msg::LaserScan::SharedPtr msg){
	std::vector<double> data = std::vector<double>(std::begin(msg->ranges), std::end(msg->ranges));
	std::vector<size_t> indices(data.size());
    std::iota(indices.begin(), indices.end(), 0);

    std::partial_sort(
        indices.begin(), indices.begin() + 5, indices.end(),
        [&](size_t i1, size_t i2) { return data[i1] < data[i2]; });

    indices.resize(5);

	std::stringstream sstream;
	sstream << "5 smalles elements: " << std::endl;
	sstream << "Index: " << indices[0] << " with value: " << data[indices[0]] << std::endl;
	sstream << "Index: " << indices[1] << " with value: " << data[indices[1]] << std::endl;
	sstream << "Index: " << indices[2] << " with value: " << data[indices[2]] << std::endl;
	sstream << "Index: " << indices[3] << " with value: " << data[indices[3]] << std::endl;
	sstream << "Index: " << indices[4] << " with value: " << data[indices[4]] << std::endl;

	RCLCPP_INFO(this->get_logger(), "%s", sstream.str().c_str());
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
