#include <iostream>
#include <array>
#include <chrono>
#include <thread>
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace rb;

int main() {
    podo::Cobot robot("10.0.2.7");
    podo::ResponseCollector rc;

    // 로봇 연결 및 기본 설정
    robot.set_operation_mode(rc, podo::OperationMode::Simulation);
    robot.set_speed_bar(rc, 0.5);
    robot.flush(rc);

    constexpr int LOOP_HZ = 125;
    constexpr double LOOP_DT = 1.0 / LOOP_HZ;

    // 테스트 속도: Joint 1을 +10 deg/s로 2초, -10 deg/s로 2초, 정지
    array<double, 6> dq_deg = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    cout << "[INFO] move_speed_j 테스트 시작!" << endl;

    // 1단계: +10 deg/s로 2초


    for (int i = 0; i < 2 * LOOP_HZ; ++i) {
        robot.move_speed_j(rc, {-2, -2, -2, -2, -2, -2}, 0.01, 0.02, 0.1, 0.1);
        this_thread::sleep_for(chrono::microseconds(int(LOOP_DT * 1e6)));
    }

    for (int i = 0; i < 2 * LOOP_HZ; ++i) {
        robot.move_speed_j(rc, {7, 7, 7, 7, 7, 7}, 0.02, 0.04, 0.1, 0.1);
        this_thread::sleep_for(chrono::microseconds(int(LOOP_DT * 1e6)));
    }

    for (int i = 0; i < 2 * LOOP_HZ; ++i) {
        robot.move_speed_j(rc, { -3, -3, -3, -3, -3, -3 }, 0.02, 0.04, 0.1, 0.1);
        this_thread::sleep_for(chrono::microseconds(int(LOOP_DT * 1e6)));
    }

  

    //cout << "[INFO] 테스트 완료. Joint 1이 앞뒤로 움직였는지 확인하세요." << endl;

    return 0;
}