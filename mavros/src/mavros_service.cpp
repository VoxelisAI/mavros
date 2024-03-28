#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <chrono>
#include "mavros_msgs/srv/stream_rate.hpp"
using namespace std::chrono_literals;

class SetStreamRateClient : public rclcpp::Node {
public:
    SetStreamRateClient() : Node("set_stream_rate_client") {
        client_ = this->create_client<mavros_msgs::srv::StreamRate>("/mavros/set_stream_rate");
        this->call_set_stream_rate_service();
    }

private:
    rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedPtr client_;

    void call_set_stream_rate_service() {
        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Service /mavros/set_stream_rate not available, waiting again...");
        }

        auto request = std::make_shared<mavros_msgs::srv::StreamRate::Request>();
        request->stream_id = 0;
        request->message_rate = 100;
        request->on_off = true;

        auto future = client_->async_send_request(request, std::bind(&SetStreamRateClient::handle_service_response, this, std::placeholders::_1));
    }

    void handle_service_response(rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedFuture future) {
        auto response = future.get();
        // Since there's no 'success' field, we assume successful completion if we get here.
        RCLCPP_INFO(this->get_logger(), "Stream rate service called successfully.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetStreamRateClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
