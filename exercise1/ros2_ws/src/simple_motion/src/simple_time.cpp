#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <atomic>

class SimpleTimeNode: public rclcpp::Node{
public:    
    SimpleTimeNode();
    ~SimpleTimeNode();

    void thread_callback();

private:
    enum class State {FORWARD, BACKWARD, FW_STOP, BW_STOP};
    
    State robot_state;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    std::atomic<bool> stop_event;
};

SimpleTimeNode::SimpleTimeNode() : rclcpp::Node("simple_time_publisher"), stop_event(false){
	publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
	robot_state = State::BW_STOP;
	std::thread t(std::bind(&SimpleTimeNode::thread_callback, this));
	t.detach();
}

SimpleTimeNode::~SimpleTimeNode(){
	stop_event = false;
}

void SimpleTimeNode::thread_callback(){
	while(rclcpp::ok() && !stop_event){
		geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();

		if (this->robot_state == State::FORWARD){
			this->robot_state = State::FW_STOP;
			msg.linear.x = 0.0;
			this->publisher->publish(msg);

			rclcpp::Rate rate(1); //sleep for 1 sec (remember: input is Hz, not sec)
			rate.sleep();
			continue;
		}

		if (this->robot_state == State::BACKWARD){
			this->robot_state = State::BW_STOP;
			msg.linear.x = 0.0;
			this->publisher->publish(msg);

			rclcpp::Rate rate(1);
			rate.sleep();
			continue;
		}

		if(this->robot_state == State::BW_STOP) {
			this->robot_state = State::FORWARD;
		}
		if(this->robot_state == State::FW_STOP) {
			this->robot_state = State::BACKWARD;
		}

		auto start_time = this->now();
		
		while(this->now() - start_time < rclcpp::Duration::from_seconds(2.0)){
			if (robot_state == State::FORWARD){
				msg.linear.x = 0.5;
			}
			if (robot_state == State::BACKWARD) {
				msg.linear.x = -0.5;
			}
			this->publisher->publish(msg);
		}
	}
}

int main(int argc, char ** argv)
{
  	(void) argc;
  	(void) argv;

	rclcpp::init(argc, argv);

	auto node = std::make_shared<SimpleTimeNode>();

	rclcpp::spin(node);
	
	rclcpp::shutdown();

  	return 0;
}
