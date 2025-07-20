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

    // �κ� ���� �� �⺻ ����
    robot.set_operation_mode(rc, podo::OperationMode::Real);
    robot.set_speed_bar(rc, 0.5);
    robot.flush(rc);

    constexpr int LOOP_HZ = 125;
    constexpr double LOOP_DT = 1.0 / LOOP_HZ;

    // �׽�Ʈ �ӵ�: Joint 1�� +10 deg/s�� 2��, -10 deg/s�� 2��, ����
    array<double, 6> dq_deg = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    cout << "[INFO] move_speed_j �׽�Ʈ ����!" << endl;

    // 1�ܰ�: +10 deg/s�� 2��
    dq_deg[0] = 10.0;
    for (int i = 0; i < 2 * LOOP_HZ; ++i) {
        robot.move_speed_j(rc, dq_deg, 0.02, 0.04, 0.1, 0.1);
        this_thread::sleep_for(chrono::microseconds(int(LOOP_DT * 1e6)));
    }

    // 2�ܰ�: -10 deg/s�� 2��
    dq_deg[0] = -10.0;
    for (int i = 0; i < 2 * LOOP_HZ; ++i) {
        robot.move_speed_j(rc, dq_deg, 0.02, 0.04, 0.1, 0.1);
        this_thread::sleep_for(chrono::microseconds(int(LOOP_DT * 1e6)));
    }

    // 3�ܰ�: ���� ���
    dq_deg[0] = 0.0;
    for (int i = 0; i < 1 * LOOP_HZ; ++i) {
        robot.move_speed_j(rc, dq_deg, 0.02, 0.04, 0.1, 0.1);
        this_thread::sleep_for(chrono::microseconds(int(LOOP_DT * 1e6)));
    }

    cout << "[INFO] �׽�Ʈ �Ϸ�. Joint 1�� �յڷ� ���������� Ȯ���ϼ���." << endl;

    return 0;
}