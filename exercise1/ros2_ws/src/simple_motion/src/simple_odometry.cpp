#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <array>
#include <functional>

class SimpleOdoNode: public rclcpp::Node{
    public:    
        SimpleOdoNode();
        ~SimpleOdoNode();
    
        void thread_callback();
        void pose_callback(nav_msgs::msg::Odometry::UniquePtr);

    private:
        double distance_to_start(std::array<double, 3>);
    
    private:
        enum class State {FORWARD, BACKWARD};

        //robot stuff
        State robot_state;
        std::array<double, 3> position; 
        std::mutex arr_mutex;

        double robot_speed = 1.0;
        double goal_distance = 2;

        // ros2 stuff
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;
        std::atomic<bool> stop_event;
    };
    
SimpleOdoNode::SimpleOdoNode(): rclcpp::Node("simple_time_publisher"), stop_event(false){
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot_0/cmd_vel", 10);
    subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/robot_0/odom", 10, std::bind(&SimpleOdoNode::pose_callback, this, std::placeholders::_1));
    robot_state = State::FORWARD;
    std::thread t(std::bind(&SimpleOdoNode::thread_callback, this));
    t.detach();
}

SimpleOdoNode::~SimpleOdoNode(){
    stop_event = false;
}

void SimpleOdoNode::thread_callback(){
    while(rclcpp::ok() && !stop_event){
		geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();

		auto start_pos = this->position;
		
		while(distance_to_start(start_pos) <= this->goal_distance && !stop_event){
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
}

double SimpleOdoNode::distance_to_start(std::array<double, 3> start_position){
    std::array<double, 3> curr_pos;
    {
        std::lock_guard<std::mutex> lock(this->arr_mutex);
        curr_pos = this->position;
    }
    double distance = std::sqrt(std::pow(curr_pos[0] - start_position[0], 2) + std::pow(curr_pos[1] - start_position[1], 2) + std::pow(curr_pos[2] - start_position[2], 2));
    std::cout << curr_pos[0] << " " << curr_pos[1] << " " << curr_pos[2] << std::endl;
    std::cout << start_position[0] << " " << start_position[1] << " " << start_position[2] << std::endl;
    std::cout << distance << std::endl << std::endl;
    return distance;
}

void SimpleOdoNode::pose_callback(nav_msgs::msg::Odometry::UniquePtr msg){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    {
        std::lock_guard<std::mutex> lock(this->arr_mutex);
        this->position = {x, y, z};
    }
}


int main(int argc, char ** argv)
{
  	(void) argc;
  	(void) argv;

	rclcpp::init(argc, argv);

	auto node = std::make_shared<SimpleOdoNode>();

	rclcpp::spin(node);
	
	rclcpp::shutdown();

}
