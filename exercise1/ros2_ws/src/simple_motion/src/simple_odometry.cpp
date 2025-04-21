#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

class SimpleOdoNode: public rclcpp::Node{
    public:    
        SimpleOdoNode();
        ~SimpleOdoNode();
    
        void thread_callback();
        void pose_callback(geometry_msgs::msg::Pose);
    
    private:
        enum class State {FORWARD, BACKWARD, FW_STOP, BW_STOP};

        State robot_state;
        std::array<float, 3> position; 

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber;
        std::atomic<bool> stop_event;
    };
    
SimpleOdoNode::SimpleOdoNode(): rclcpp::Node("simple_time_publisher"), stop_event(false){
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscriber = this->create_subscription<geometry_msgs::msg::Pose>("/odom", 10, std::bind(&SimpleOdoNode::pose_callback, this));
    robot_state = State::BW_STOP;
    std::thread t(std::bind(&SimpleOdoNode::thread_callback, this));
    t.detach();
}

SimpleOdoNode::~SimpleOdoNode(){
    stop_event = false;
}

void SimpleOdoNode::thread_callback(){
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

void SimpleOdoNode::pose_callback(geometry_msgs::msg::Pose msg){

}


int main(int argc, char ** argv)
{
  	(void) argc;
  	(void) argv;

	// rclcpp::init(argc, argv);

	// //auto node = std::make_shared<SimpleTimeNode>();

	// //rclcpp::spin(node);
	
	// rclcpp::shutdown();

}
