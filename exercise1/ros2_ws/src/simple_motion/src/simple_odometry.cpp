#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <array>

class SimpleOdoNode: public rclcpp::Node{
    public:    
        SimpleOdoNode();
        ~SimpleOdoNode();
    
        void thread_callback();
        void pose_callback(geometry_msgs::msg::Pose);
        double distance_to_start(std::array<double, 3>);
    
    private:
        enum class State {FORWARD, BACKWARD, FW_STOP, BW_STOP};

        //robot stuff
        State robot_state;
        std::array<double, 3> position; 
        std::mutex arr_mutex;

        double robot_speed = 0.5;
        double goal_distance = 2;

        // ros2 stuff
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber;
        std::atomic<bool> stop_event;
    };
    
SimpleOdoNode::SimpleOdoNode(): rclcpp::Node("simple_time_publisher"), stop_event(false){
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscriber = this->create_subscription<geometry_msgs::msg::Pose>("/odom", 10, std::bind(&SimpleOdoNode::pose_callback, this));
    robot_state = State::BW_STOP;
    std::thread t(std::bind(&SimpleOdoNode::thread_callback, this));
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

        std::array<double, 3> start_position;
        {
            std::lock_guard<std::mutex> lock(this->arr_mutex);
            start_position = this->position;
        }
        
        while(distance_to_start(start_position) < this->goal_distance){
            if (robot_state == State::FORWARD){
                msg.linear.x = this->robot_speed;
            }
            if (robot_state == State::BACKWARD) {
                msg.linear.x = -this->robot_speed;
            }
            this->publisher->publish(msg);
        }
    }
}

double SimpleOdoNode::distance_to_start(std::array<double, 3> start_position){
    std::array<double, 3> curr_pos;
    {
        std::lock_guard<std::mutex> lock(this->arr_mutex);
        curr_pos = this->position;
    }

    return std::sqrt(std::pow(curr_pos[0] - start_position[0], 2) + std::pow(curr_pos[1] - start_position[1], 2) + std::pow(curr_pos[2] - start_position[2], 2));
}

void SimpleOdoNode::pose_callback(geometry_msgs::msg::Pose msg){
    double x = msg.position.x;
    double y = msg.position.y;
    double z = msg.position.z;
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
