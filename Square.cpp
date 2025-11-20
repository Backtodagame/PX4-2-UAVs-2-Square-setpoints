#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <memory> // Cho shared_ptr

using namespace mavsdk;
using namespace std;
using namespace std::chrono_literals;

// Hằng số cho nhiệm vụ
constexpr float TAKEOFF_ALTITUDE_M = 5.0f;
constexpr float SQUARE_SIDE_M = 10.0f;
constexpr float TIMEOUT_S = 30.0f; 

// Hàm tiện ích để chờ UAV đạt đến vị trí
// Lưu ý: Hàm này vẫn cần Offboard setpoint liên tục, nhưng chúng ta sẽ chấp nhận rủi ro này 
// vì việc gửi setpoint liên tục trong luồng chờ làm phức tạp code mẫu.
void wait_for_position(Telemetry& telemetry, float target_north, float target_east, float target_down, int uav_id) {
    auto start_time = chrono::steady_clock::now();
    
    cout << "UAV " << uav_id << ": Waiting for target (N: " << target_north << ", E: " << target_east << ", D: " << target_down << ")..." << endl;

    while (true) {
        this_thread::sleep_for(100ms);

        // Lấy vị trí NED hiện tại (tương đối so với Home)
        auto current_pos_ned = telemetry.position_velocity_ned();

        // Tính toán khoảng cách lỗi
        float error_n = abs(current_pos_ned.position.north_m - target_north);
        float error_e = abs(current_pos_ned.position.east_m - target_east);
        float error_d = abs(current_pos_ned.position.down_m - target_down);

        // Điều kiện dừng: Khi UAV nằm trong khoảng dung sai (ví dụ: 0.5 mét)
        if (error_n < 0.5f && error_e < 0.5f && error_d < 0.5f) {
            cout << "UAV " << uav_id << ": Target reached (N: " << current_pos_ned.position.north_m << ", E: " << current_pos_ned.position.east_m << ")." << endl;
            this_thread::sleep_for(1s); 
            break; 
        }

        // Kiểm tra timeout
        auto elapsed = chrono::duration<double>(chrono::steady_clock::now() - start_time).count();
        if (elapsed > TIMEOUT_S) {
            cerr << "UAV " << uav_id << ": Timeout: UAV did not reach target." << endl;
            throw runtime_error("Offboard command timed out");
        }
    }
}

// Hàm nhiệm vụ chính cho mỗi UAV, chạy trong luồng riêng
void run_square_mission(shared_ptr<System> system, int uav_id, float start_east_offset) {
    
    // Khởi tạo các plugin riêng cho UAV này
    Action action{system};
    Telemetry telemetry{system};
    Offboard offboard{system};

    cout << "\n--- Starting Mission for UAV " << uav_id << " (Offset: " << start_east_offset << "m) ---" << endl;
    
    // 1. Kiểm tra sẵn sàng
    cout << "UAV " << uav_id << ": Waiting for MAVLink system to be ready..." << endl;
    while (!telemetry.health_all_ok()) {
        this_thread::sleep_for(1s);
    }
    cout << "UAV " << uav_id << ": ✅ System ready." << endl;

    try {
        // 2. ARM
        cout << "UAV " << uav_id << ": -- Arming..." << endl;
        auto arm_result = action.arm();
        if (arm_result != Action::Result::Success) {
            cerr << "UAV " << uav_id << ": Arming failed: " << arm_result << endl;
            return;
        }
        this_thread::sleep_for(2s);
        
        // 3. Gửi lệnh Offboard đầu tiên
        cout << "UAV " << uav_id << ": -- Sending initial position setpoint (stay put)..." << endl;
        // Setpoint ban đầu phải tính cả offset
        offboard.set_position_ned({0.0f, start_east_offset, 0.0f, 0.0f}); 

        // 4. Bật chế độ Offboard
        cout << "UAV " << uav_id << ": -- Starting Offboard mode..." << endl;
        auto offboard_result = offboard.start();
        if (offboard_result != Offboard::Result::Success) {
            cerr << "UAV " << uav_id << ": Starting Offboard failed: " << offboard_result << endl;
            return;
        }
        this_thread::sleep_for(1s);

        // 5. Cất cánh lên độ cao 5m
        cout << "UAV " << uav_id << ": -- Takeoff to " << TAKEOFF_ALTITUDE_M << "m..." << endl;
        // Điểm đích: N=0, E=start_east_offset, D=-5m
        float target_d = -TAKEOFF_ALTITUDE_M;
        offboard.set_position_ned({0.0f, start_east_offset, target_d, 0.0f});
        wait_for_position(telemetry, 0.0f, start_east_offset, target_d, uav_id);
        
        // --- 6. BAY HÌNH VUÔNG 10m (sử dụng tọa độ NED đã được bù trừ) ---
        cout << "UAV " << uav_id << ": -- Starting Square Flight (10m side) --" << endl;
        
        float current_n = 0.0f;
        float current_e = start_east_offset; // Bắt đầu từ offset
        
        // Góc 1: 10m về phía Bắc
        current_n = SQUARE_SIDE_M;
        cout << "UAV " << uav_id << ": Flying to Corner 1 (N:" << current_n << ", E:" << current_e << ")..." << endl;
        offboard.set_position_ned({current_n, current_e, target_d, 0.0f});
        wait_for_position(telemetry, current_n, current_e, target_d, uav_id);

        // Góc 2: 10m về phía Đông (Thêm 10m vào offset)
        current_e = start_east_offset + SQUARE_SIDE_M;
        cout << "UAV " << uav_id << ": Flying to Corner 2 (N:" << current_n << ", E:" << current_e << ")..." << endl;
        offboard.set_position_ned({current_n, current_e, target_d, 0.0f});
        wait_for_position(telemetry, current_n, current_e, target_d, uav_id);

        // Góc 3: 10m về phía Nam (Quay về N=0)
        current_n = 0.0f;
        cout << "UAV " << uav_id << ": Flying to Corner 3 (N:" << current_n << ", E:" << current_e << ")..." << endl;
        offboard.set_position_ned({current_n, current_e, target_d, 0.0f});
        wait_for_position(telemetry, current_n, current_e, target_d, uav_id);

        // Góc 4: 10m về phía Tây (Quay về E=start_east_offset)
        current_e = start_east_offset;
        cout << "UAV " << uav_id << ": Flying to Corner 4 (N:" << current_n << ", E:" << current_e << ")..." << endl;
        offboard.set_position_ned({current_n, current_e, target_d, 0.0f});
        wait_for_position(telemetry, current_n, current_e, target_d, uav_id);
        
        cout << "UAV " << uav_id << ": Square flight completed." << endl;
        this_thread::sleep_for(2s);

        // 7. Tắt chế độ Offboard
        cout << "UAV " << uav_id << ": -- Stopping Offboard mode..." << endl;
        offboard.stop();

        // 8. LANDING (Hạ cánh)
        cout << "UAV " << uav_id << ": -- Landing..." << endl;
        auto land_result = action.land();
        if (land_result != Action::Result::Success) {
            cerr << "UAV " << uav_id << ": Landing failed: " << land_result << endl;
        }
        
        // Chờ UAV hạ cánh
        while (telemetry.in_air()) {
            this_thread::sleep_for(1s);
        }

    } catch (const exception& e) {
        cerr << "\nUAV " << uav_id << ": An error occurred during the mission: " << e.what() << endl;
        offboard.stop();
    }
    cout << "UAV " << uav_id << ": ✅ Mission done and landed!" << endl;
}

shared_ptr<System> find_system_by_id(Mavsdk& mavsdk, uint8_t system_id) {
    auto start_time = chrono::steady_clock::now();
    cout << "Waiting for System ID " << (int)system_id << " to connect..." << endl;
    
    while (chrono::duration<double>(chrono::steady_clock::now() - start_time).count() < TIMEOUT_S) {
        for (auto system : mavsdk.systems()) {
            // Lấy ID MAVLink của hệ thống
            if (system->get_system_id() == system_id) {
                cout << "✅ Found System ID " << (int)system_id << "." << endl;
                return system;
            }
        }
        this_thread::sleep_for(500ms);
    }
    
    cerr << "Timeout: System ID " << (int)system_id << " not found." << endl;
    return nullptr;
}

int main() {
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    
    // Kết nối đến 2 cổng khác nhau
    mavsdk.add_any_connection("udp://:14540");
    mavsdk.add_any_connection("udp://:14541");

    
    auto system1 = find_system_by_id(mavsdk,1);

    auto system2 = find_system_by_id(mavsdk,2);
    if (system1 == nullptr || system2 == nullptr) {
        cerr << "\n--- MISSING SYSTEMS: Cannot start mission ---" << endl;
        return 1;
    }

    
    // Luồng 1: UAV 1 (ID 1), không dịch chuyển, bay quanh vị trí Home của nó
    thread t1(run_square_mission, system1, 1, 0.0f); 

    // Luồng 2: UAV 2 (ID 2), dịch chuyển 20m về phía Đông
    // Đảm bảo khoảng cách offset lớn hơn kích thước hình vuông (10m)
    thread t2(run_square_mission, system2, 2, 20.0f); 

    // Chờ cả hai luồng hoàn thành nhiệm vụ
    t1.join();
    t2.join();

    cout << "\n--- ALL MISSIONS COMPLETED ---" << endl;
    return 0;
}
