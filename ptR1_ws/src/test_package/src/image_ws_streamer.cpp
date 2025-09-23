




//xxxxxxxxxxxxxxxxxxxxxxxxxxx




// ------------------- [ ROS + OpenCV + WebSocket Streamer ] -------------------
// This node subscribes to /camera/image_raw, encodes it as JPEG, and streams
// the image as binary data to connected WebSocket clients in real-time.
// ------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/Image.h>       // สำหรับ message ของกล้องจาก ROS
#include <cv_bridge/cv_bridge.h>     // สำหรับแปลง ROS image → OpenCV image
#include <opencv2/opencv.hpp>        // ใช้ OpenCV encode ภาพเป็น JPEG

#include <websocketpp/config/asio_no_tls.hpp> // ไลบรารี WebSocket++ (ไม่มี TLS)
#include <websocketpp/server.hpp>

#include <thread>                    // สำหรับสร้าง thread แยก
#include <vector>                    // สำหรับเก็บรายชื่อ client
#include <mutex>                     // ป้องกันการเข้าถึง clients พร้อมกัน (thread safety)
#include <atomic>                    // สำหรับ flag ที่ thread แยกแชร์กันแบบปลอดภัย

// ใช้ WebSocket++ server แบบไม่มี TLS
typedef websocketpp::server<websocketpp::config::asio> server;

// --------------------------- [ Global Variables ] ---------------------------

// รายชื่อ client ที่เชื่อมต่ออยู่
std::vector<websocketpp::connection_hdl> clients;

// ป้องกัน clients vector ถูกเข้าถึงพร้อมกัน
std::mutex client_mutex;

// instance ของ WebSocket server
server ws_server;

// flag ที่ใช้บอก thread ให้หยุดทำงาน
std::atomic<bool> stop_server{false};

// --------------------------- [ Callback Functions ] ---------------------------

// เมื่อมี client เชื่อมต่อใหม่
void on_open(websocketpp::connection_hdl hdl) {
    std::lock_guard<std::mutex> lock(client_mutex);
    clients.push_back(hdl);  // บันทึก client
    ROS_INFO("New client connected");
}

// เมื่อมี client ตัดการเชื่อมต่อ
void on_close(websocketpp::connection_hdl hdl) {
    std::lock_guard<std::mutex> lock(client_mutex);

    // ลบ client ที่ตรงกับ hdl โดยใช้ owner_before() แทน ==
    clients.erase(
        std::remove_if(clients.begin(), clients.end(),
            [&hdl](const websocketpp::connection_hdl& other) {
                return !hdl.owner_before(other) && !other.owner_before(hdl);
            }),
        clients.end()
    );

    ROS_INFO("Client disconnected");
}

// เมื่อได้รับภาพใหม่จาก ROS topic
void image_callback(const sensor_msgs::ImageConstPtr& msg) {

    if (clients.empty()) {
        return; // ไม่มี client → ไม่ต้อง encode ก็ได้ (save CPU)
    }
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // แปลงภาพจาก ROS message เป็น OpenCV Mat (BGR8)
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // แปลงภาพเป็น JPEG เก็บใน vector
    std::vector<uchar> buf;
    cv::imencode(".jpg", cv_ptr->image, buf);



    // ส่งภาพ JPEG ให้ client ทุกตัวที่เชื่อมต่ออยู่
    std::lock_guard<std::mutex> lock(client_mutex);
    for (auto& hdl : clients) {
        try {
            ws_server.send(hdl, buf.data(), buf.size(), websocketpp::frame::opcode::binary);
        } catch (const websocketpp::exception& e) {
            ROS_WARN("Failed to send to client: %s", e.what());
        }
    }
}

// --------------------------- [ WebSocket Thread ] ---------------------------

// ฟังก์ชันที่ใช้รัน WebSocket server ใน thread แยก
void run_websocket_server() {
    ws_server.set_open_handler(on_open);
    ws_server.set_close_handler(on_close);

    ws_server.init_asio();         // เตรียม asio I/O
    ws_server.listen(8181);        // เปิดพอร์ต 8181
    ws_server.start_accept();      // เริ่มรับ connection ใหม่

    // ใช้ run_one แทน run เพื่อให้ควบคุม loop เองได้
    ws_server.run_one();

    // loop ตรวจสอบและประมวลผล connection จนกว่า flag จะถูกตั้ง
    while (!stop_server && ros::ok()) {
        ws_server.poll();  // ประมวลผล WebSocket event
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // เมื่อหยุด: ปิด server
    ws_server.stop_listening();   // ไม่รับ connection ใหม่
    ws_server.stop();             // ปิด server
    ROS_INFO("WebSocket server stopped");
}

// --------------------------- [ Main Function ] ---------------------------

int main(int argc, char** argv) {
    // เริ่ม ROS node
    ros::init(argc, argv, "image_ws_node");
    ros::NodeHandle nh;

    // subscribe image จากกล้อง
    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, image_callback);

    // สร้าง thread สำหรับรัน WebSocket server
    std::thread ws_thread(run_websocket_server);
    ROS_INFO("WebSocket server started");

    // ใช้ AsyncSpinner เพื่อไม่ block main thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // รอจนกว่า ROS จะ shutdown (กด Ctrl+C หรือถูก kill)
    ros::waitForShutdown();

    // เมื่อ ROS shutdown แล้ว → สั่งให้ WebSocket หยุดทำงาน
    stop_server = true;

    // รอให้ thread WebSocket ปิดตัวเสร็จ
    ws_thread.join();

    ROS_INFO("Node shutdown complete");
    return 0;
}
