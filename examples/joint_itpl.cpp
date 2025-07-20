#include <iostream>
#include <vector>
#include <array>
#include <algorithm>  // std::copy
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace rb;

int main() {
    try {
        // 1. �κ� ����
        podo::Cobot robot("10.0.2.7");
        podo::ResponseCollector rc;

        robot.set_operation_mode(rc, podo::OperationMode::Simulation); // or Real
        robot.set_speed_bar(rc, 0.2);
        robot.flush(rc);

        // 2. ���� ��� �ʱ�ȭ
        robot.move_itpl_clear(rc);

        // 3. ���� �� (deg)���� �̵� ��� ����
        vector<vector<double>> waypoints = {
            {0, -30, 60, 0, 45, 0},
            {10, -40, 65, 10, 50, 5},
            {20, -50, 70, 15, 55, 10},
            {30, -60, 75, 20, 60, 15}
        };

        for (size_t i = 0; i < waypoints.size(); ++i) {
            std::array<double, 6> point;
            std::copy(waypoints[i].begin(), waypoints[i].end(), point.begin());

            double blend = (i == waypoints.size() - 1) ? 0.0 : 0.1;
            robot.move_itpl_add(rc, point, 200, 400, blend);  // �ӵ�, ���ӵ�, ����
        }

        // 4. ��� ����: �ùٸ� MoveITPLOption ���
        robot.move_itpl_run(rc, 200, rb::podo::MoveITPLOption::Smooth);

        // 5. �̵� ����/���� ���
        if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success)
            robot.wait_for_move_finished(rc);

        rc.error().throw_if_not_empty();
        cout << "������ ��� ���������̼� (Smooth ���) �Ϸ�!" << endl;

    }
    catch (const std::exception& e) {
        cerr << "���� �߻�: " << e.what() << endl;
        return 1;
    }

    return 0;
}
