#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <atomic>

using namespace rclcpp;

class SimpleTimeNode: public Node{
public:
	enum class State {FORWARD, BACKWARD, FW_STOP, BW_STOP};

	SimpleTimeNode() : Node("simple_time_publisher"), stop_event(false){
		publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		robot_state = State::BW_STOP;
		std::thread t(std::bind(&SimpleTimeNode::timer_callback, this));
		t.detach();
	}

	~SimpleTimeNode(){
		stop_event = false;
	}

	void timer_callback(){
		while(rclcpp::ok() && !stop_event){
			std::cout << "Enter loop" << std::endl;
			if (this->robot_state == State::FORWARD){
				this->robot_state = State::FW_STOP;
				Rate rate(1); //sleep for 1 sec (remember: input is Hz, not sec)
				rate.sleep();
				continue;
			}

			if (this->robot_state == State::BACKWARD){
				this->robot_state = State::BW_STOP;
				Rate rate(1);
				rate.sleep();
				continue;
			}

			if(this->robot_state == State::BW_STOP) {
				this->robot_state = State::FORWARD;
			}
			if(this->robot_state == State::FW_STOP) {
				this->robot_state = State::BACKWARD;
			}

			geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
			auto start_time = this->now();
			
			while(this->now() - start_time < Duration::from_seconds(2.0)){
				std::cout << (this->now() - start_time).seconds() << std::endl;
				if (robot_state == State::FORWARD){
					msg.linear.x = 0.5;
				}
				if (robot_state == State::BACKWARD) {
					msg.linear.x = -0.5;
				}
				this->publisher->publish(msg);
			}

			std::cout << "Finish Loop" << std::endl;
		}
	}

private:
	State robot_state;
	Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
	std::atomic<bool> stop_event;	
	
};

int main(int argc, char ** argv)
{
  	(void) argc;
  	(void) argv;

	init(argc, argv);

	std::cout << "Initializing Node" << std::endl;
	auto node = std::make_shared<SimpleTimeNode>();

	std::cout << "Creating spin" << std::endl;
	spin(node);

	shutdown();

  	return 0;
}
