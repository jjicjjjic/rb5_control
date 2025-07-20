
#include <iostream>
#include "rbpodo/rbpodo.hpp"

using namespace rb;
using namespace std;

int main() {
    try {
        // 1️⃣ 로봇 객체 생성
        podo::Cobot robot("10.0.2.7");
        podo::ResponseCollector rc;

        // 2️⃣ 시뮬레이션 모드 설정 (실제 로봇일 경우 Real로 변경 가능)
        //robot.set_operation_mode(rc, podo::OperationMode::Real);
        robot.set_operation_mode(rc, podo::OperationMode::Simulation);

        // 3️⃣ 속도 바 설정 (0.2 = 20%)
        robot.set_speed_bar(rc, 0.2);

        // 4️⃣ 기존 MoveITPL 데이터 초기화
        robot.move_itpl_clear(rc);

        // 5️⃣ 이동할 3개 점 추가 (각 포인트는 x, y, z)
        robot.move_itpl_add(rc, { 400, -450, 200 }, 100);
        robot.move_itpl_add(rc, { 340.0, -500.0, 350.0 }, 100);
        robot.move_itpl_add(rc, { 285.384, 	-555.692, 200.0 }, 100);

        // 6️⃣ 부드러운 이동 실행 (가속도 200, MoveITPLOption::Smooth)
        robot.move_itpl_run(rc, 200, podo::MoveITPLOption::Smooth);

        // 7️⃣ 이동이 시작될 때까지 대기 후, 이동 완료될 때까지 기다림
        if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success) {
            robot.wait_for_move_finished(rc);
        }

        cout << "MoveITPL execution completed!" << endl;

        // 오류 체크
        rc.error().throw_if_not_empty();
    }
    catch (const std::exception& e) {
        cerr << e.what() << endl;
        return 1;
    }
    return 0;
}

