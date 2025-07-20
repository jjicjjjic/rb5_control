#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "rbpodo/rbpodo.hpp"
#include <string>
#include <thread>
#include <mutex>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <nlohmann/json.hpp>  // nlohmann/json 라이브러리 (https://github.com/nlohmann/json)

using namespace rb;
using namespace std::chrono_literals;
using namespace Eigen;
using namespace std;

using json = nlohmann::json;

#define PORT 12345
#define BUFFER_SIZE 1024

// 로봇의 좌표와 속도 데이터를 저장하는 구조체
std::vector<double> g_position(3, 0.0);
std::vector<double> g_velocity(3, 0.0);
std::mutex g_dataMutex;

// 로봇의 실제 end-effector 좌표 데이터를 저장하는 구조체
std::mutex g_ee_mutex;
std::vector<double> ee_position_real = { 0.0, 0.0, 0.0 };

// 소켓 send() 호출 보호용 mutex
std::mutex g_socket_mutex;

// TCP 통신용 스레드 함수
// 이 함수는 클라이언트(Python 코드 등)로부터 JSON 데이터를 수신하여 전역 데이터를 갱신합니다.
void tcpCommunicationThread() {
    int server_fd, client_fd;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    // 서버 소켓 생성
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return;
    }

    // 주소 재사용 옵션 설정
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
        &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        return;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // 소켓 바인딩
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        return;
    }

    // 연결 대기
    if (listen(server_fd, 3) < 0) {
        perror("listen failed");
        return;
    }

    std::cout << "TCP server is waiting on Port " << PORT << "." << std::endl;

    // 클라이언트 연결 수락
    if ((client_fd = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
        perror("accept");
        return;
    }

    char buffer[BUFFER_SIZE] = { 0 };
    std::string dataBuffer;
    while (true) {
        ssize_t bytesRead = recv(client_fd, buffer, BUFFER_SIZE, 0);
        if (bytesRead <= 0) {
            std::cerr << "Connection ended or failed." << std::endl;
            break;
        }
        dataBuffer.append(buffer, bytesRead);

        // JSON 데이터가 완전한지 확인하고 파싱 시도
        try {
            auto j = json::parse(dataBuffer);
            std::cout << "Received JSON: " << j.dump() << std::endl;

            std::vector<double> pos(3), vel(3);  // 🔥 여기에 선언!

            if (j.contains("position") && j.contains("velocity")) {
                pos[0] = j["position"][0];
                pos[1] = j["position"][1];
                pos[2] = j["position"][2];
                vel[0] = j["velocity"][0];
                vel[1] = j["velocity"][1];
                vel[2] = j["velocity"][2];

                std::cout << "Position: [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]" << std::endl;
                std::cout << "Velocity: [" << vel[0] << ", " << vel[1] << ", " << vel[2] << "]" << std::endl;

                std::lock_guard<std::mutex> lock(g_dataMutex);
                g_position = pos;
                g_velocity = vel;
            }


            // 완전한 JSON을 처리했으므로 버퍼 초기화
            dataBuffer.clear();
        }
        catch (json::parse_error& e) {
            // 수신된 데이터가 불완전하면 파싱 오류가 발생하므로 추가 데이터를 기다림
            continue;
        }
    }
    close(client_fd);
    close(server_fd);
}

// 링크 정보를 담는 구조체
struct LinkDH {
    double a;       // 링크 길이 (mm)
    double alpha;   // twist 각도 (rad)
    double d;       // 링크 오프셋 (mm)
    double theta;   // 관절각 (rad) + 오프셋 포함 (변수 or 고정)
    bool isRevolute; // true면 관절 변수(회전), false면 고정
};

// degrees <-> radians 변환
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad / M_PI * 180.0; }

// 9개 링크를 포함하는 6축 로봇 예제
class Robot9Link {
public:
    // D-H 파라미터 (초기값: 오프셋 포함)
    // alpha, theta는 rad 단위로 관리
    vector<LinkDH> links;

    Robot9Link() {
        links.resize(9);

        // Link 1
        links[0].a = 0.0;
        links[0].alpha = deg2rad(-90.0);
        links[0].d = 169.2;    // mm
        links[0].theta = 0.0;      // 관절 변수(θ1), 초기 0
        links[0].isRevolute = true;

        // Link 2 (θ2 - 90° 오프셋 -> theta에 -90° 더해놓고 시작)
        links[1].a = 0.0;
        links[1].alpha = deg2rad(0.0);
        links[1].d = -148.4;    // mm
        links[1].theta = deg2rad(-90.0); // 초기 오프셋
        links[1].isRevolute = true;

        // Link 3 (고정 링크, a1=425, d3=148.4)
        links[2].a = 425.0;
        links[2].alpha = deg2rad(0.0);
        links[2].d = 0.0;    // mm
        links[2].theta = 0.0;      // 고정
        links[2].isRevolute = false;

        // Link 4 (θ3)
        links[3].a = 0.0;
        links[3].alpha = deg2rad(0.0);
        links[3].d = 148.4;    // mm
        links[3].theta = 0.0;      // 관절 변수
        links[3].isRevolute = true;

        // Link 5 (θ4, a2=392, d5=110.7)
        links[4].a = 392.0;
        links[4].alpha = deg2rad(0.0);
        links[4].d = 0.0;    // mm
        links[4].theta = 0.0;
        links[4].isRevolute = false;

        // Link 6 (θ5 + 90° 오프셋)
        links[5].a = 0.0;
        links[5].alpha = deg2rad(0.0);
        links[5].d = -110.7;     // mm
        links[5].theta = deg2rad(90.0);  // 초기 오프셋
        links[5].isRevolute = true;

        // Link 7 (고정 링크, 예: 손목 오프셋)
        links[6].a = 0.0;
        links[6].alpha = deg2rad(90.0);
        links[6].d = 0.0;      // mm
        links[6].theta = 0.0;
        links[6].isRevolute = false;

        // Link 8 (θ6)
        links[7].a = 0.0;
        links[7].alpha = deg2rad(-90.0);
        links[7].d = 110.7;      // mm
        links[7].theta = 0.0;
        links[7].isRevolute = true;

        // Link 9 (툴링크)
        links[8].a = 0.0;
        links[8].alpha = deg2rad(90.0);
        links[8].d = -96.7;
        links[8].theta = 0.0;
        links[8].isRevolute = true;
    }

    vector<double> joint_offset = { 0, -M_PI / 2, 0, M_PI / 2, 0, 0 };

    // 단일 D-H 변환행렬 (theta, alpha는 rad, d, a는 mm 단위)
    Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha) {
        double ct = cos(theta);
        double st = sin(theta);
        double ca = cos(alpha);
        double sa = sin(alpha);

        Eigen::Matrix4d T;
        T << ct, -st * ca, st* sa, a* ct,
            st, ct* ca, -ct * sa, a* st,
            0, sa, ca, d,
            0, 0, 0, 1;
        return T;
    }

    // 현재 links[] 상태(θ 포함)로부터 end-effector(마지막 Link 9)까지의 forward kinematics
    // 반환: 4x4 변환행렬 (mm, rad)
    Eigen::Matrix4d forwardKinematics(const vector<double>& jointVals) {
        // jointVals에는 실제 '회전'이 일어나는 6개의 관절각(θ1~θ6)이 들어있다고 가정
        // links[] 중 isRevolute=true 인 곳만 업데이트
        int jointIndex = 0;
        for (size_t i = 0; i < links.size(); i++) {
            if (links[i].isRevolute) {
                links[i].theta = jointVals[jointIndex] + joint_offset[jointIndex];
                jointIndex++;
            }
        }

        // base -> link9까지 변환행렬 계산
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 9; i++) {
            double th = links[i].theta;
            double dd = links[i].d;
            double aa = links[i].a;
            double al = links[i].alpha;
            T = T * dhTransform(th, dd, aa, al);
        }

        return T;
    }

    // 6x6 자코비언 (위치+자세)
    // 상단 3행: z_i x (p_end - p_i), 하단 3행: z_i
    Matrix<double, 6, 6> computeJacobian6x6(const vector<double>& jointVals) {
        // 우선 forwardKinematics 계산과 유사하게, 중간 프레임들을 모두 구해야 함
        // (주의) 자코비언을 구할 때도 links[].theta에 jointVals를 반영해야 하지만,
        //        여기서는 편의상 별도 계산 루틴을 만들겠습니다.

        // link0(base) ~ link9 까지 10개 프레임
        int jointIndex = 0;
        // links[].theta에 jointVals 반영
        for (size_t i = 0; i < links.size(); i++) {
            if (links[i].isRevolute) {
                links[i].theta = jointVals[jointIndex] + joint_offset[jointIndex];
                jointIndex++;
            }
        }

        // 2) 각 프레임별 누적 변환행렬 T_list
        vector<Matrix4d> T_list(10, Matrix4d::Identity());
        for (int i = 0; i < 9; i++) {
            T_list[i + 1] = T_list[i] * dhTransform(links[i].theta, links[i].d, links[i].a, links[i].alpha);
        }
        Vector3d p_end = T_list[9].block<3, 1>(0, 3);

        // 3) 자코비언 계산
        Matrix<double, 6, 6> J;
        J.setZero();

        jointIndex = 0;
        for (int i = 0; i < 9; i++) {
            if (!links[i].isRevolute) continue; // 고정 링크는 스킵

            // z_i, p_i
            Vector3d z_i = T_list[i].block<3, 1>(0, 2);
            Vector3d p_i = T_list[i].block<3, 1>(0, 3);

            // 위치 미분: z_i x (p_end - p_i)
            Vector3d Jv = z_i.cross(p_end - p_i);
            // 자세 미분: z_i
            Vector3d Jw = z_i;

            // 상단 3행은 위치, 하단 3행은 회전
            J.block<3, 1>(0, jointIndex) = Jv;
            J.block<3, 1>(3, jointIndex) = Jw;

            jointIndex++;
        }

        return J;
    }

    // 4x4 변환행렬로부터 (roll, pitch, yaw) [rad] 추출 (Z-Y-X 순 등 여러 정의가 있으니 주의)
    // 여기서는 roll=pitch=yaw 순서를 X->Y->Z 로 가정한 예시
    // 실제 로봇 소프트웨어와 일치하는 방식으로 구현해야 함
    Vector3d matrixToRPY(const Matrix4d& T) {
        // 회전행렬
        Matrix3d R = T.block<3, 3>(0, 0);

        // roll(y), pitch(x), yaw(z) 등 여러 convention이 있으므로 주의
        // 여기서는 R = Rz(yaw)*Ry(pitch)*Rx(roll) 형태라 가정한 예시
        // 일반적 ZYX 순서: yaw->pitch->roll
        // roll = atan2(R32, R33)
        // pitch = -asin(R31)
        // yaw = atan2(R21, R11)
        // (행렬에서 항목 추출)

        double roll, pitch, yaw;
        // Z-Y-X convention (yaw-pitch-roll) 예시:
        pitch = -asin(R(2, 0));
        double cPitch = cos(pitch);

        if (fabs(cPitch) > 1e-6) {
            roll = atan2(R(2, 1), R(2, 2));
            yaw = atan2(R(1, 0), R(0, 0));
        }
        else {
            // pitch ~ ±90도 부근 특이
            roll = 0.0;
            yaw = atan2(-R(0, 1), R(1, 1));
        }
        return Vector3d(roll, pitch, yaw);
    }

    // 수치적 IK (위치+자세)
    // target_position: (x, y, z) [mm]
    // target_orientation: (roll, pitch, yaw) [rad], ZYX 순서 가정
    // initialQ: 초기 관절값 (6개)
    // maxIter, eps: 반복 제한 / 오차 한계
    vector<double> inverseKinematics6D(const Vector3d& target_position,
        const Vector3d& target_orientation,
        const vector<double>& initialQ,
        int maxIter = 100,
        double eps = 1e-3)
    {
        vector<double> q = initialQ; // 현재 추정값 (6개)

        for (int iter = 0; iter < maxIter; iter++) {
            // 1) 현재 FK
            Matrix4d T_cur = forwardKinematics(q);
            Vector3d p_cur = T_cur.block<3, 1>(0, 3);
            Vector3d rpy_cur = matrixToRPY(T_cur);

            // 2) 위치 오차
            Vector3d pos_err = target_position - p_cur;
            // 3) 자세 오차 (단순 차)
            //   주의: rpy_des - rpy_cur 는 ±π 근처에서 불연속 가능
            Vector3d ori_err = target_orientation - rpy_cur;

            // 4) 종합 오차 (6x1)
            Vector<double, 6> e;
            e << pos_err(0), pos_err(1), pos_err(2),
                ori_err(0), ori_err(1), ori_err(2);

            if (e.norm() < eps) {
                cout << "[IK] Converged at iteration " << iter
                    << ", error norm = " << e.norm() << endl;
                return q;
            }

            // 5) 자코비언(6x6)
            Matrix<double, 6, 6> J = computeJacobian6x6(q);

            // 6) 의사역행렬
            //    (J * J^T)가 역가역이 안 되는 특이점에서 문제 발생 가능
            Matrix<double, 6, 6> JJt = J * J.transpose();
            if (fabs(JJt.determinant()) < 1e-12) {
                cout << "[IK] Near-singular or singular Jacobian. Stop.\n";
                return q;
            }
            Matrix<double, 6, 6> Jinv = J.transpose() * JJt.inverse();

            // 7) 관절 업데이트 Δq = J^+ * e
            Vector<double, 6> dq = Jinv * e;
            for (int i = 0; i < 6; i++) {
                q[i] += dq(i);
            }
        }

        cout << "[IK] Failed to converge after " << maxIter << " iterations.\n";
        return q;
    }
};


int main() {
    // TCP 통신 스레드를 실행함.
    std::thread tcpThread(tcpCommunicationThread);

    try {
        Robot9Link rm;

        // Make connection
        podo::Cobot robot("10.0.2.7");
        podo::ResponseCollector rc;

        //robot.set_operation_mode(rc, podo::OperationMode::Real);
        robot.set_operation_mode(rc, podo::OperationMode::Simulation);
        robot.set_speed_bar(rc, 0.5);

        robot.flush(rc);

        Vector3d target_position(286, -557, 300.0);
        Vector3d target_orientation(deg2rad(90.0), deg2rad(0.0), deg2rad(0.0));

        string choice;

        // whlie 문 시작하기

        while (true) {
            std::lock_guard<std::mutex> lock(g_dataMutex);
            // g_position, g_velocity - 3D data

            // 새로운 명령창 출력
            cout << "Do you want to continue? (y/n): ";
            cin >> choice;

            // 사용자가 "n"을 입력하면 루프 종료
            if (choice == "n" || choice == "N") {
                cout << "Exiting the program." << endl;
                break;
            }

            // 사용자가 "y"를 입력하면 xyz 좌표 입력
            else if (choice == "y" || choice == "Y") {

                cout << "Enter xyz target positions: ";
                for (double& val : target_position) {
                    cin >> val;
                }

                cout << "Target Position: ";
                for (double val : target_position) cout << val << " ";
                cout << endl;

                vector<double> joint_q = { 0,0,0,0,0,0 };
                robot.get_system_variable(rc, podo::SystemVariable::SD_J0_ANG, joint_q[0]);
                robot.get_system_variable(rc, podo::SystemVariable::SD_J1_ANG, joint_q[1]);
                robot.get_system_variable(rc, podo::SystemVariable::SD_J2_ANG, joint_q[2]);
                robot.get_system_variable(rc, podo::SystemVariable::SD_J3_ANG, joint_q[3]);
                robot.get_system_variable(rc, podo::SystemVariable::SD_J4_ANG, joint_q[4]);
                robot.get_system_variable(rc, podo::SystemVariable::SD_J5_ANG, joint_q[5]);

                vector<double> joint_q_rad;
                for (size_t i = 0; i < joint_q.size(); i++) {
                    joint_q_rad.push_back(deg2rad(joint_q[i]));
                }

                // IK 실행
                vector<double> solution = rm.inverseKinematics6D(target_position,
                    target_orientation, joint_q_rad, 2000, 1e-4);

                // 결과 출력
                cout << "\n--- IK result ---\n";

                Matrix4d T_final = rm.forwardKinematics(solution);
                Vector3d p_final = T_final.block<3, 1>(0, 3);
                Vector3d rpy_final = rm.matrixToRPY(T_final);

                cout << "POS: " << p_final.transpose() << " [mm]\n";
                cout << "RPY: roll = " << rpy_final(0) * 180.0 / M_PI
                    << " deg, pitch = " << rpy_final(1) * 180.0 / M_PI
                    << " deg, yaw = " << rpy_final(2) * 180.0 / M_PI << " deg\n\n";

                for (int i = 0; i < 6; i++) {
                    solution[i] = rad2deg(solution[i]);
                    cout << "q[" << i << "] = " << solution[i] << " deg\n";
                }

                //Eigen::Matrix4d T_current = rm.forwardKinematics(joint_q);
                //Eigen::Vector3d p_current = T_current.block<3, 1>(0, 3);  // x,y,z
                //std::cout << "Current End-Effector Position: "
                //    << p_current.transpose() << " mm" << std::endl;

                //robot.move_j(rc, { 100, -40, -110, 83, 320, 1.5 }, 200, 400);
                robot.move_j(rc, { solution[0], solution[1], solution[2], solution[3], solution[4], solution[5] }, 200, 400);
                // (x,y,z,rx,ry,rz) = (285.384, -555.692, 137.342, 55.354, -43.985, 36.691)

                if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success) {
                    robot.wait_for_move_finished(rc);
                }

            }

            // 잘못된 입력 처리
            else {
                cout << "Invalid input. Please enter 'y' to continue or 'n' to exit." << endl;
            }
        }

        // while 문 끝내기



        // If there is any error during above process, throw exception error
        rc.error().throw_if_not_empty();
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    // 스레드가 종료될 때까지 대기 (추후 사용 시 적절한 종료 조건을 추가 필요)
    tcpThread.join();

    return 0;
}