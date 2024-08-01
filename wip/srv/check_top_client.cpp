#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class CheckTopClient : public rclcpp::Node
{
public:
  CheckTopClient()
      : Node("check_top_client")
  {
    client_ = this->create_client<std_srvs::srv::Trigger>("/is_ball_on_top");

    while (!client_->wait_for_service(500ms))
    {
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
  }

  void send_request()
  {
    if (future_ && !future_->is_ready())
    {
      RCLCPP_INFO(this->get_logger(), "Previous service call timed out or was not completed. Canceling...");
      future_.reset();
      timeout_timer_->cancel();
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    future_ = client_->async_send_request(request, std::bind(&CheckTopClient::process_response, this, std::placeholders::_1));

    timeout_timer_ = this->create_wall_timer(1s, std::bind(&CheckTopClient::check_timeout, this));
  }

private:
  void process_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
    if (timeout_timer_ && timeout_timer_->is_canceled() == false)
    {
      timeout_timer_->cancel();
    }

    auto response = future.get();
    if (response)
    {
      RCLCPP_INFO(this->get_logger(), "Success: %d", response->success);
      RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Service call was unsuccessful.");
    }
  }

  void check_timeout()
  {
    if (future_ && !future_->is_ready())
    {
      RCLCPP_INFO(this->get_logger(), "Service call timed out.");
      future_.reset();
    }
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CheckTopClient>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
