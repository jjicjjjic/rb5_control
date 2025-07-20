#include <iostream>
#include <vector>
#include <array>
#include <algorithm>  // std::copy
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace rb;

int main() {
    try {
        // 1. 로봇 연결
        podo::Cobot robot("10.0.2.7");
        podo::ResponseCollector rc;

        robot.set_operation_mode(rc, podo::OperationMode::Simulation); // or Real
        robot.set_speed_bar(rc, 0.2);
        robot.flush(rc);

        // 2. 기존 경로 초기화
        robot.move_itpl_clear(rc);

        // 3. 관절 값 (deg)으로 이동 경로 설정
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
            robot.move_itpl_add(rc, point, 200, 400, blend);  // 속도, 가속도, 블렌딩
        }

        // 4. 경로 실행: 올바른 MoveITPLOption 사용
        robot.move_itpl_run(rc, 200, rb::podo::MoveITPLOption::Smooth);

        // 5. 이동 시작/종료 대기
        if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success)
            robot.wait_for_move_finished(rc);

        rc.error().throw_if_not_empty();
        cout << "관절값 기반 이터폴레이션 (Smooth 모드) 완료!" << endl;

    }
    catch (const std::exception& e) {
        cerr << "오류 발생: " << e.what() << endl;
        return 1;
    }

    return 0;
}
