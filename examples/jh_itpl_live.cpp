#include <iostream>
#include <thread>
#include "rbpodo/rbpodo.hpp"

using namespace rb;
using namespace std;

int main() {
    try {
        // 1️⃣ 로봇 객체 생성
        podo::Cobot robot("10.0.2.7");
        podo::ResponseCollector rc;

        // 2️⃣ 시뮬레이션 모드 설정 (실제 로봇일 경우 Real로 변경 가능)
        robot.set_operation_mode(rc, podo::OperationMode::Simulation);

        // 3️⃣ 속도 바 설정 (0.2 = 20%)
        robot.set_speed_bar(rc, 0.2);

        // 4️⃣ 기존 MoveITPL 데이터 초기화
        robot.move_itpl_clear(rc);

        string choice;
        bool firstRun = true;  // 첫 실행인지 확인
        podo::RobotState robot_state;

        while (true) {
            // 새로운 명령창 출력
            cout << "Enter new xyz target positions (or type 'n' to stop): ";

            // ✅ cin 입력을 받을 수 있는 변수를 먼저 선언
            vector<double> temp_point(3);
            cin >> temp_point[0];

            if (cin.fail()) {  // 사용자가 'n'을 입력하면 종료
                cin.clear();
                string stop_check;
                cin >> stop_check;
                if (stop_check == "n" || stop_check == "N") {
                    cout << "Exiting the program." << endl;
                    break;
                }
                cout << "Invalid input. Please enter valid xyz coordinates." << endl;
                continue;
            }

            cin >> temp_point[1] >> temp_point[2];

            // ✅ 입력값을 `Point`로 변환 후 MoveITPL에 추가
            podo::StandardVector::Point target_point = { temp_point[0], temp_point[1], temp_point[2] };
            robot.move_itpl_add(rc, target_point, 100);

            // 첫 번째 실행이면 이동 시작
            if (firstRun) {
                robot.move_itpl_run(rc, 200, podo::MoveITPLOption::Smooth);
                firstRun = false;
            }

            // 이동 중에 실시간으로 새로운 좌표 추가 가능하도록 루프
            while (true) {
                robot.get_robot_state(rc, robot_state);

                if (robot_state == podo::RobotState::Moving) {
                    cout << "🚀 Robot is moving... Enter next point: ";

                    vector<double> next_point(3);
                    cin >> next_point[0];

                    if (cin.fail()) {  // 사용자가 'n'을 입력하면 종료
                        cin.clear();
                        string stop_check;
                        cin >> stop_check;
                        if (stop_check == "n" || stop_check == "N") {
                            cout << "Exiting the program." << endl;
                            return 0;
                        }
                        cout << "Invalid input. Please enter valid xyz coordinates." << endl;
                        continue;
                    }

                    cin >> next_point[1] >> next_point[2];

                    // ✅ 입력값을 `Point`로 변환 후 MoveITPL에 추가
                    podo::StandardVector::Point next_target_point = { next_point[0], next_point[1], next_point[2] };
                    robot.move_itpl_add(rc, next_target_point, 100);
                }
                else {
                    cout << "🛑 Robot is idle. Waiting for next command..." << endl;
                    break;
                }

                // MoveITPL이 부드럽게 움직일 수 있도록 살짝 대기
                this_thread::sleep_for(100ms);
            }
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
