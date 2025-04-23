#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <atomic>
#include <csignal>

class SimpleTimeNode: public rclcpp::Node{
public:    
    SimpleTimeNode();

    void thread_callback();
	void stop_node();

private:
    enum class State {FORWARD, BACKWARD};
    
	// robot stuff
    State robot_state;
	double robot_speed = 0.5;
	double driving_time = 2;

	// ros stuff
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    std::atomic<bool> stop_event;
	std::thread worker_thread;

};

SimpleTimeNode::SimpleTimeNode() : rclcpp::Node("simple_time_publisher"), stop_event(false){
	publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot_0/cmd_vel", rclcpp::QoS(10).reliable());
	robot_state = State::FORWARD;
	worker_thread = std::thread(&SimpleTimeNode::thread_callback, this);
}

void SimpleTimeNode::stop_node(){
	this->stop_event = true;

	std::this_thread::sleep_for(std::chrono::seconds(2));
}

void SimpleTimeNode::thread_callback(){
	while(rclcpp::ok() && !stop_event){
		geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();

		auto start_time = this->now();
		
		while(this->now() - start_time < rclcpp::Duration::from_seconds(this->driving_time) && !stop_event){
			if (robot_state == State::FORWARD){
				msg.linear.x = this->robot_speed;
			}
			if (robot_state == State::BACKWARD) {
				msg.linear.x = -this->robot_speed;
			}
			this->publisher->publish(msg);
		}

		if(this->robot_state == State::BACKWARD) {
			this->robot_state = State::FORWARD;
		}
		else {
			this->robot_state = State::BACKWARD;
		}

		msg.linear.x = 0.0;
		this->publisher->publish(msg);

		rclcpp::Rate rate(1);
		rate.sleep();
	}

	geometry_msgs::msg::Twist stop_msg = geometry_msgs::msg::Twist();
	stop_msg.linear.x = 0.0;
	stop_msg.linear.y = 0.0;
	stop_msg.linear.z = 0.0;

	this->publisher->publish(stop_msg);
	std::cout << "published this message" << std::endl;
}

std::shared_ptr<SimpleTimeNode> global_node = nullptr;

void signal_handler(int signum){
	RCLCPP_INFO(global_node->get_logger(), "Interrupt signal received");
	if (global_node != nullptr){
		global_node -> stop_node();
		std::chrono::seconds(2);
	}
	rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  	(void) argc;
  	(void) argv;
	
	rclcpp::init(argc, argv);


	global_node = std::make_shared<SimpleTimeNode>();

	signal(SIGINT, signal_handler);

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(global_node);

	executor.spin();

	if(rclcpp::ok()){
		rclcpp::shutdown();
	}

  	return 0;
}
