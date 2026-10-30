#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "grab_demo/srv/grab_object.hpp"
#include "grab_demo/srv/place_object.hpp"

class Nav_grab: public rclcpp::Node
{
private:
  using Nav2_client = nav2_msgs::action::NavigateToPose;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Nav2_client>;

  // Clients
  rclcpp_action::Client<Nav2_client>::SharedPtr goal_client;
  rclcpp::Client<grab_demo::srv::GrabObject>::SharedPtr grab_client;
  rclcpp::Client<grab_demo::srv::PlaceObject>::SharedPtr place_client;

  // Requests
  std::shared_ptr<grab_demo::srv::GrabObject::Request> grab_req;

  enum class Stage { TO_A, TO_B };
  Stage stage_;

public:
  Nav_grab(const std::string &node_name) : Node(node_name)
  {
    goal_client = rclcpp_action::create_client<Nav2_client>(this, "navigate_to_pose");
    grab_client = this->create_client<grab_demo::srv::GrabObject>("/obj_grab_service");
    place_client = this->create_client<grab_demo::srv::PlaceObject>("/obj_place_service");

    grab_req = std::make_shared<grab_demo::srv::GrabObject::Request>();
    stage_ = Stage::TO_A;
  }

  void send_goal()
  {
    if (!goal_client->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      rclcpp::shutdown();
    }

    // Goal A
    auto goal = Nav2_client::Goal();
    goal.pose.header.stamp = this->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 0.35; // A.x
    goal.pose.pose.position.y = 0.0;  // A.y
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
 
    RCLCPP_INFO(this->get_logger(), "Sending goal A");

    auto send_goal_options = rclcpp_action::Client<Nav2_client>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Nav_grab::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&Nav_grab::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&Nav_grab::result_callback, this, std::placeholders::_1);

    goal_client->async_send_goal(goal, send_goal_options);
  }

  void goal_response_callback(const std::shared_ptr<ClientGoalHandle> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(ClientGoalHandle::SharedPtr, const std::shared_ptr<const Nav2_client::Feedback> /*feedback*/)
  {
    // Optionally log feedback
  }

  void result_callback(const ClientGoalHandle::WrappedResult &result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
      {
        sleep(1);
        if (stage_ == Stage::TO_A)
        {
          RCLCPP_INFO(this->get_logger(), "Arrived at A. Grabbing object...");
          grab_req->obj_link = "aruco_link";
          // After grab completes, send goal B
          grab_client->async_send_request(
              grab_req,
              [this](rclcpp::Client<grab_demo::srv::GrabObject>::SharedFuture /*future*/) {
                RCLCPP_INFO(this->get_logger(), "Grab done. Sending goal B...");
                send_goal_B();
              });
        }
        else if (stage_ == Stage::TO_B)// Stage::TO_B
        {
          RCLCPP_INFO(this->get_logger(), "Arrived at B. Placing object...");
          auto place_req = std::make_shared<grab_demo::srv::PlaceObject::Request>();
          place_client->async_send_request(place_req);
        }
        break;
      }
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Result received");
  }

  void send_goal_B()
  {

    
    auto goalB = Nav2_client::Goal();
    goalB.pose.header.stamp = this->now();
    goalB.pose.header.frame_id = "map";
    goalB.pose.pose.position.x = 1.0;  // B.x (adjust as needed)
    goalB.pose.pose.position.y = 0.0;  // B.y (adjust as needed)
    goalB.pose.pose.position.z = 0.0;
    goalB.pose.pose.orientation.x = 0.0;
    goalB.pose.pose.orientation.y = 0.0;
    goalB.pose.pose.orientation.z = 0.0;
    goalB.pose.pose.orientation.w = 1.0;

    stage_ = Stage::TO_B;

    auto optsB = rclcpp_action::Client<Nav2_client>::SendGoalOptions();
    optsB.goal_response_callback = std::bind(&Nav_grab::goal_response_callback, this, std::placeholders::_1);
    optsB.feedback_callback = std::bind(&Nav_grab::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    optsB.result_callback = std::bind(&Nav_grab::result_callback, this, std::placeholders::_1);

    goal_client->async_send_goal(goalB, optsB);
  }

};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav_grab>("nav_grab_node");
  node->send_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
