
//로우패스필터 적용 전
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
#include <chrono>
#include <atomic>
#include <array>

using namespace rb;
using namespace std::chrono_literals;
using namespace Eigen;
using namespace std;

using json = nlohmann::json;

std::vector<double> initial_tcp_info(3, 0.0);
std::vector<double> joint_deg(6);
std::vector<double> q_rad(6);
std::mutex q_mutex;
Eigen::Vector3d v_tcp_linear = Eigen::Vector3d::Zero();

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

#define PORT 12345
#define BUFFER_SIZE 1024

#define PORT 12345            // 제어 데이터용 포트
#define INIT_PORT 12346       // 초기 TCP 정보 전송용 포트

// 로봇의 좌표와 속도 데이터를 저장하는 구조체
std::vector<double> g_position(3, 0.0);
std::vector<double> g_velocity(3, 0.0);
std::mutex g_dataMutex;

// TCP 목표 업데이트 시간을 기록하는 변수와 뮤텍스
std::chrono::steady_clock::time_point g_last_target_update = std::chrono::steady_clock::now();
std::mutex g_target_mutex;

class LowPassFilterVec3 {
private:
    double alpha;
    Eigen::Vector3d prevValue;

public:
    LowPassFilterVec3(double dt, double cutoff_freq) {
        double rc = 1.0 / (2.0 * M_PI * cutoff_freq);
        alpha = dt / (dt + rc);  // 필터 계수 계산
        prevValue = Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d filter(const Eigen::Vector3d& input) {
        prevValue = alpha * input + (1.0 - alpha) * prevValue;
        return prevValue;
    }

    void reset(const Eigen::Vector3d& value = Eigen::Vector3d::Zero()) {
        prevValue = value;
    }
};


LowPassFilterVec3 lpf_position(0.05, 2);  // dt=0.05초, cutoff=2Hz
LowPassFilterVec3 lpf_velocity(0.05, 2);
void tcpInitialInfoThread() {
    int server_fd, client_fd;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    // 서버 소켓 생성
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return;
    }
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
        &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        return;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(INIT_PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        return;
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        return;
    }

    std::cout << "Initial TCP info server is waiting on Port " << INIT_PORT << "." << std::endl;

    while (true) {
        if ((client_fd = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            continue;
        }
        // 초기 TCP 정보 전송
        json initData;
        initData["initial_tcp"] = initial_tcp_info;  // 예: [x, y, z]
        std::string initStr = initData.dump() + "\n";
        send(client_fd, initStr.c_str(), initStr.size(), 0);
        close(client_fd);
    }
    close(server_fd);
}

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

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
        &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        return;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        return;
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        return;
    }

    std::cout << "TCP server is waiting on Port " << PORT << "." << std::endl;

    // 클라이언트 연결 수락 (제어 데이터용)
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
        std::vector<double> pos(3), vel(3);

        try {
            auto j = json::parse(dataBuffer);
            std::cout << "Received JSON: " << j.dump() << std::endl;

            pos[0] = j["position"][0];
            pos[1] = j["position"][1];
            pos[2] = j["position"][2];
            vel[0] = j["velocity"][0];
            vel[1] = j["velocity"][1];
            vel[2] = j["velocity"][2];

            if (j.contains("position") && j.contains("velocity")) {
                auto pos = j["position"];
                auto vel = j["velocity"];
                std::cout << "Position: [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]" << std::endl;
                std::cout << "Velocity: [" << vel[0] << ", " << vel[1] << ", " << vel[2] << "]" << std::endl;
            }

            {
                std::lock_guard<std::mutex> lock(g_dataMutex);
                Eigen::Vector3d posVec(pos[0], pos[1], pos[2]);
                Eigen::Vector3d velVec(vel[0], vel[1], vel[2]);

                Eigen::Vector3d filteredPos = lpf_position.filter(posVec);
                Eigen::Vector3d filteredVel = lpf_velocity.filter(velVec);

                g_position[0] = filteredPos[0];
                g_position[1] = filteredPos[1];
                g_position[2] = filteredPos[2];

                g_velocity[0] = filteredVel[0];
                g_velocity[1] = filteredVel[1];
                g_velocity[2] = filteredVel[2];
            }
            {
                std::lock_guard<std::mutex> lock(g_target_mutex);
                g_last_target_update = std::chrono::steady_clock::now();
            }
            dataBuffer.clear();
        }
        catch (json::parse_error& e) {
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

// === 제어 상수 ===
constexpr int LOOP_HZ = 250;
constexpr double LOOP_DT = 1.0 / LOOP_HZ;
constexpr double MAX_JOINT_SPEED = 50.0; // deg/s
constexpr double LAMBDA = 0.001;

//Vector3d p_target = Vector3d::Zero();
double v_desired = 20.0;
atomic<bool> has_new_target(false);
mutex mtx;

int main() {
    // TCP 통신 스레드를 실행함.
    std::array<double, 6> pnt{};
    // 로봇에서 TCP 정보 읽기
    // robot.get_tcp_info(rc, pnt);  // 기존 코드 사용
    // 초기 TCP 정보 저장 (예시)
    initial_tcp_info[0] = pnt[0];
    initial_tcp_info[1] = pnt[1];
    initial_tcp_info[2] = pnt[2];

    // 초기 정보 서버와 제어 데이터 서버 스레드 실행
    std::thread initThread(tcpInitialInfoThread);
    std::thread tcpThread(tcpCommunicationThread);

    try {
        Robot9Link rm;

        // Make connection
        podo::Cobot robot("10.0.2.7");
        podo::ResponseCollector rc;

        robot.set_operation_mode(rc, podo::OperationMode::Real);
        //robot.set_operation_mode(rc, podo::OperationMode::Simulation);
        robot.set_speed_bar(rc, 0.5);

        robot.flush(rc);

        Vector3d target_position(-326, -111, 453);
        Vector3d target_orientation(deg2rad(90.0), deg2rad(0.0), deg2rad(0.0));

        std::vector<double> initialQ(6);
        for (int i = 0; i < 6; ++i) {
            robot.get_system_variable(rc, static_cast<podo::SystemVariable>(
                static_cast<int>(podo::SystemVariable::SD_J0_ANG) + i), joint_deg[i]);
            initialQ[i] = joint_deg[i] * DEG2RAD;
        }

        // 2. IK 계산
        std::vector<double> ik_result = rm.inverseKinematics6D(
            target_position,            // 목표 위치
            target_orientation,         // 목표 자세 (roll, pitch, yaw)
            initialQ,                   // 초기 추정값
            100,                        // 최대 반복
            1e-3                        // 수렴 오차 기준
        );

        // 3. 각도(rad) → 각도(deg)로 변환
        std::array<double, 6> target_deg{};
        for (int i = 0; i < 6; ++i) {
            target_deg[i] = rad2deg(ik_result[i]);
        }

        // 4. move_j()로 이동 명령
        robot.move_j(rc, target_deg, 3.0, 3.0, 0.2, 0.2); // (rc, 각도, 가속, 속도, blend radius, blend time)
        cout << "[MOVE_J] IK 결과 각도로 이동합니다." << endl;
        //
        string choice;

        for (int i = 0; i < 6; ++i) {
            robot.get_system_variable(rc, static_cast<podo::SystemVariable>(
                static_cast<int>(podo::SystemVariable::SD_J0_ANG) + i), joint_deg[i]);
            q_rad[i] = joint_deg[i] * DEG2RAD;
        }

        int step = 0;
        bool has_target = false;

        // PID 제어 변수 설정
        Vector3d pid_integral = Vector3d::Zero();
        Vector3d pid_prev_error = Vector3d::Zero();
        double Kp = 0.9;   // 비례 게인 (필요에 따라 조정)
        double Ki = 0.0;   // 적분 게인
        double Kd = 0.07;   // 미분 게인

        // while 루프 시작 전에 실제 end-effector 위치를 g_position에 할당
        std::array<double, 6> pnt{};
        robot.get_tcp_info(rc, pnt);
        {
            std::lock_guard<std::mutex> lock(g_dataMutex);
            g_position[0] = pnt[0];
            g_position[1] = pnt[1];
            g_position[2] = pnt[2];
        }
        // TCP 목표 업데이트 시간을 갱신 (선택 사항)
        {
            std::lock_guard<std::mutex> lock(g_target_mutex);
            g_last_target_update = std::chrono::steady_clock::now();
        }
        
        robot.get_tcp_info(rc, pnt);
        {
            std::lock_guard<std::mutex> lock(g_dataMutex);
            g_position[0] = pnt[0];
            g_position[1] = pnt[1];
            g_position[2] = pnt[2];
        }
        // 초기 TCP 정보를 global 변수에 저장
        initial_tcp_info[0] = pnt[0];
        initial_tcp_info[1] = pnt[1];
        initial_tcp_info[2] = pnt[2];
        
        while (true) {
            auto t_loop = chrono::steady_clock::now();

            Vector3d p_cur;
            {
                lock_guard<mutex> lock(g_dataMutex);
                Matrix4d T_cur = rm.forwardKinematics(q_rad);
                p_cur = T_cur.block<3, 1>(0, 3);

                // 매 루프마다 현재 속도 계산 (g_velocity가 방향 포함하고 있다고 가정)
                v_tcp_linear = Vector3d(g_velocity[0], g_velocity[1], g_velocity[2]);
            }

            // 실제 end-effector의 위치 읽어오기 (x, y, z 만 사용)
            std::array<double, 6> pnt{};
            robot.get_tcp_info(rc, pnt);
            Vector3d p_meas(pnt[0], pnt[1], pnt[2]);

            // 목표 위치 결정: TCP로부터 최근 데이터가 수신되었으면 그 값을 사용,
            // 그렇지 않으면 현재 측정된 위치를 목표로 하여 정지 상태 유지
            Vector3d desired_pos;
            {
                lock_guard<mutex> lock(g_dataMutex);
                desired_pos = Vector3d(g_position[0], g_position[1], g_position[2]);
            }
            bool target_recent = false;
            {
                lock_guard<mutex> lock(g_target_mutex);
                auto now = chrono::steady_clock::now();
                auto diff_ms = chrono::duration_cast<chrono::milliseconds>(now - g_last_target_update).count();
                if (diff_ms < 1000) {  // 1초 이내에 업데이트된 경우
                    target_recent = true;
                }
            }
            if (!target_recent) {
                // TCP 업데이트가 없으면 현재 실제 위치를 유지하도록 목표를 설정
                desired_pos = p_meas;
            }

            // PID 제어 연산 (목표: desired_pos, 측정: p_meas)
            Vector3d error = desired_pos - p_meas;
            pid_integral += error * LOOP_DT;
            Vector3d pid_derivative = (error - pid_prev_error) / LOOP_DT;
            pid_prev_error = error;
            Vector3d control_command = Kp * error + Ki * pid_integral + Kd * pid_derivative;

            // v_tcp_linear = v_tcp_linear + control_command;
            v_tcp_linear = v_tcp_linear;
            if (error.norm() < 1.0) {
                v_tcp_linear.setZero();
            }

            // 자코비안 기반 속도 제어
            Matrix6d J;
            {
                lock_guard<mutex> lock(q_mutex);
                J = rm.computeJacobian6x6(q_rad);
            }
            Matrix6d J_pinv = J.transpose() * (J * J.transpose() + LAMBDA * Matrix6d::Identity()).inverse();

            Vector<double, 6> v_tcp;
            v_tcp.head<3>() = v_tcp_linear;
            v_tcp.tail<3>().setZero();

            Vector<double, 6> dq = J_pinv * v_tcp;

            array<double, 6> dq_deg{};
            for (int i = 0; i < 6; ++i)
                dq_deg[i] = clamp(RAD2DEG * dq[i], -MAX_JOINT_SPEED, MAX_JOINT_SPEED);

            robot.move_speed_j(rc, dq_deg, 0.01, 0.05, 0.2, 0.2);

            // 예측 업데이트
            {
                lock_guard<mutex> lock(q_mutex);
                for (int i = 0; i < 6; ++i)
                    q_rad[i] += dq[i] * LOOP_DT;
            }

            if (step++ % 10 == 0) {
                cout << "[STEP " << step << "] 현재: " << p_cur.transpose()
                    << ", 목표: " << Vector3d(g_position[0], g_position[1], g_position[2]).transpose()
                    << ", 속도: " << v_tcp_linear.transpose() << endl;
            }

            this_thread::sleep_until(t_loop + chrono::milliseconds(int(LOOP_DT * 1000)));
        }


        // while 문 끝내기



        // If there is any error during above process, throw exception error
        rc.error().throw_if_not_empty();
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    initThread.join();
    tcpThread.join();
    return 0;
}