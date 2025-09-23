// ------------------- [ ROS + OpenCV + WebSocket TLS Streamer ] -------------------
// Subscribes to /camera/image_raw, encodes as JPEG, and streams to WebSocket clients (WSS).
// ----------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <websocketpp/config/asio.hpp>         // รองรับ TLS
#include <websocketpp/server.hpp>
#include <boost/asio/ssl/context.hpp>

#include <thread>
#include <vector>
#include <mutex>
#include <atomic>

typedef websocketpp::server<websocketpp::config::asio> server;

std::vector<websocketpp::connection_hdl> clients;
std::mutex client_mutex;
server ws_server;
std::atomic<bool> stop_server{false};



// ⚙️ TLS context สำหรับเข้ารหัส WSS
boost::asio::ssl::context_ptr on_tls_init(websocketpp::connection_hdl) {
    auto ctx = std::make_shared<boost::asio::ssl::context>(boost::asio::ssl::context::tlsv12);

    try {
        ctx->set_options(
            boost::asio::ssl::context::default_workarounds |
            boost::asio::ssl::context::no_sslv2 |
            boost::asio::ssl::context::no_sslv3 |
            boost::asio::ssl::context::single_dh_use);

        ctx->use_certificate_chain_file("/home/leoss/ptR1_ws/src/test_package/Cert/cert.pem"); // ✅ เปลี่ยน path ให้ตรง
        ctx->use_private_key_file("/home/leoss/ptR1_ws/src/test_package/Cert/key.pem", boost::asio::ssl::context::pem);
    }
    catch (std::exception& e) {
        ROS_ERROR("TLS initialization failed: %s", e.what());
    }

    return ctx;
}

void on_open(websocketpp::connection_hdl hdl) {
    std::lock_guard<std::mutex> lock(client_mutex);
    clients.push_back(hdl);
    ROS_INFO("New client connected (WSS)");
}

void on_close(websocketpp::connection_hdl hdl) {
    std::lock_guard<std::mutex> lock(client_mutex);

    clients.erase(
        std::remove_if(clients.begin(), clients.end(),
            [&hdl](const websocketpp::connection_hdl& other) {
                return !hdl.owner_before(other) && !other.owner_before(hdl);
            }),
        clients.end()
    );

    ROS_INFO("Client disconnected");
}

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<uchar> buf;
    cv::imencode(".jpg", cv_ptr->image, buf);

    std::lock_guard<std::mutex> lock(client_mutex);
    for (auto& hdl : clients) {
        try {
            ws_server.send(hdl, buf.data(), buf.size(), websocketpp::frame::opcode::binary);
        } catch (websocketpp::exception const& e) {
            ROS_WARN("Send failed: %s", e.what());
        }
    }
}

void run_websocket_server() {
    ws_server.set_tls_init_handler(on_tls_init);
    ws_server.set_open_handler(on_open);
    ws_server.set_close_handler(on_close);

    ws_server.init_asio();
    ws_server.listen(8181);  // WSS: https://localhost:8181
    ws_server.start_accept();

    ws_server.run_one();
    while (!stop_server && ros::ok()) {
        ws_server.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ws_server.stop_listening();
    ws_server.stop();
    ROS_INFO("WSS server stopped");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wss_image_stream_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, image_callback);

    std::thread ws_thread(run_websocket_server);
    ROS_INFO("🔒 WebSocket Secure (WSS) server started on port 8181");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();

    stop_server = true;
    ws_thread.join();

    ROS_INFO("Node shutdown complete");
    return 0;
}
