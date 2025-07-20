#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <cmath>
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace Eigen;
using namespace rb;


// ---- 기존 로봇 모델 코드: Robot9Link 정의 ----
// 여기 너가 준 Robot9Link 클래스 코드 복붙 (너무 길어서 생략했지만 이미 잘 짜여 있음)
// forwardKinematics(), computeJacobian6x6() 다 포함됨
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
        vector<Eigen::Matrix4d> T_list = computeAllTransforms(jointVals);
        return T_list[9];  // 마지막 변환 행렬 반환
    }
    vector<Eigen::Matrix4d> computeAllTransforms(const vector<double>& jointVals) {
        vector<Eigen::Matrix4d> T_list(10, Eigen::Matrix4d::Identity());
        int jointIndex = 0;

        for (size_t i = 0; i < links.size(); i++) {
            if (links[i].isRevolute) {
                links[i].theta = jointVals[jointIndex] + joint_offset[jointIndex];
                jointIndex++;
            }
            // ✅ 한 번의 반복문에서 업데이트와 변환행렬 적용
            T_list[i + 1] = T_list[i] * dhTransform(links[i].theta, links[i].d, links[i].a, links[i].alpha);
        }
        return T_list;
    }

    // 6x6 자코비언 (위치+자세)
    // 상단 3행: z_i x (p_end - p_i), 하단 3행: z_i
    Matrix<double, 6, 6> computeJacobian6x6(const vector<double>& jointVals) {
        vector<Eigen::Matrix4d> T_list = computeAllTransforms(jointVals);
        Vector3d p_end = T_list[9].block<3, 1>(0, 3);
        Matrix<double, 6, 6> J;
        J.setZero();

        int jointIndex = 0;
        for (int i = 0; i < 9; i++) {
            if (!links[i].isRevolute) continue; // ✅ 6개 관절까지만 고려하도록 제한

            Vector3d z_i = T_list[i].block<3, 1>(0, 2);
            Vector3d p_i = T_list[i].block<3, 1>(0, 3);
            Vector3d Jv = z_i.cross(p_end - p_i);
            Vector3d Jw = z_i;

            J.block<3, 1>(0, jointIndex) = Jv;
            J.block<3, 1>(3, jointIndex) = Jw;
            jointIndex++;

            if (jointIndex == 6) break;  // ✅ 6개만 계산하고 탈출
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
// ---- 사다리꼴 속도 프로파일 클래스 ----
class TrapezoidalProfile {
public:
    double s_total;
    double v_max;
    double a_max;
    double t_total;
    double t_accel, t_const, t_decel;

    TrapezoidalProfile(double s_total_, double v_max_, double a_max_) {
        s_total = s_total_;
        v_max = v_max_;
        a_max = a_max_;

        t_accel = v_max / a_max;
        double s_accel = 0.5 * a_max * t_accel * t_accel;

        if (s_total < 2 * s_accel) {
            t_accel = sqrt(s_total / a_max);
            t_const = 0.0;
            t_decel = t_accel;
            t_total = 2 * t_accel;
        }
        else {
            double s_const = s_total - 2 * s_accel;
            t_const = s_const / v_max;
            t_decel = t_accel;
            t_total = t_accel + t_const + t_decel;
        }
    }

    double get_velocity(double t) {
        if (t < t_accel) return a_max * t;
        else if (t < t_accel + t_const) return v_max;
        else if (t < t_total) return v_max - a_max * (t - t_accel - t_const);
        else return 0.0;
    }

    double get_total_time() const { return t_total; }
};

// ---- 메인 ----
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <cmath>
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace Eigen;
using namespace rb;


int main() {
    podo::Cobot robot("10.0.2.7");
    podo::ResponseCollector rc;
    robot.set_operation_mode(rc, podo::OperationMode::Real);
    robot.set_speed_bar(rc, 0.5);
    robot.flush(rc);

    Robot9Link model;

    // 현재 관절각도 읽기
    vector<double> joint_deg(6);
    for (int i = 0; i < 6; ++i) {
        robot.get_system_variable(rc, static_cast<podo::SystemVariable>(static_cast<int>(podo::SystemVariable::SD_J0_ANG) + i), joint_deg[i]);
    }

    // 초기 관절 값 출력
    cout << "[DEBUG] 초기 관절각도 (deg): ";
    for (double deg : joint_deg) cout << deg << " ";
    cout << endl;

    // 라디안 변환
    vector<double> q_rad(6);
    for (int i = 0; i < 6; ++i)
        q_rad[i] = joint_deg[i] * M_PI / 180.0;

    // 현재 위치 계산
    Vector3d p_start = model.forwardKinematics(q_rad).block<3, 1>(0, 3);
    Vector3d p_target = p_start;

    // 목표 좌표 입력
    cout << "Target (x y z): ";
    for (int i = 0; i < 3; ++i) cin >> p_target[i];

    // 입력한 목표 좌표 출력
    cout << "[DEBUG] 목표 위치: " << p_target.transpose() << endl;

    Vector3d dir = p_target - p_start;
    double dist = dir.norm();
    if (dist < 1e-6) {
        cout << "[ERROR] Target too close or same.\n";
        return 0;
    }
    dir.normalize();

    // 이동 거리가 작으면 속도 제한
    double max_vel = (dist < 20.0) ? 10.0 : 20.0;
    double max_acc = (dist < 20.0) ? 20.0 : 50.0;

    TrapezoidalProfile profile(dist, max_vel, max_acc);
    double t_total = profile.get_total_time();

    cout << "[DEBUG] 이동 거리: " << dist << " mm, 예상 이동 시간: " << t_total << " 초" << endl;

    // 제어 루프 설정
    constexpr int LOOP_HZ = 50;
    constexpr double LOOP_DT = 1.0 / LOOP_HZ;
    constexpr double MAX_JOINT_SPEED = 30.0;
    double t = 0.0;

    for (int step = 0; t < t_total; ++step) {
        auto start_time = chrono::steady_clock::now();

        
        // 현재 위치 계산
        Matrix4d T_cur = model.forwardKinematics(q_rad);
        Vector3d p_cur = T_cur.block<3, 1>(0, 3);
        Vector3d rpy_cur = model.matrixToRPY(T_cur);

        // 목표 방향 속도
        double v_mag = profile.get_velocity(t);
        Vector3d v_tcp_linear = dir * v_mag;

        // 자코비안 계산
        Matrix<double, 6, 6> J = model.computeJacobian6x6(q_rad);

        // 자코비안 의사역행렬 (Damped Least Squares)
        double lambda = 0.001;
        Matrix<double, 6, 6> I = Matrix<double, 6, 6>::Identity();
        Matrix<double, 6, 6> J_pinv = J.transpose() * (J * J.transpose() + lambda * I).inverse();

        // TCP 속도 벡터 구성
        Vector<double, 6> v_tcp;
        v_tcp.head<3>() = v_tcp_linear;
        v_tcp.tail<3>().setZero(); // 회전 없음

        // 관절 속도 계산
        Vector<double, 6> dq = J_pinv * v_tcp;

        // 속도 제한 후 deg/s 변환
        array<double, 6> dq_deg{};
        for (int i = 0; i < 6; ++i) {
            dq_deg[i] = dq[i] * 180.0 / M_PI;
            dq_deg[i] = std::clamp(dq_deg[i], -MAX_JOINT_SPEED, MAX_JOINT_SPEED);
        }

        // 관절 적분
        for (int i = 0; i < 6; ++i) {
            q_rad[i] += dq[i] * LOOP_DT;
        }

        // 10 스텝마다 디버깅 출력
        if (step % 10 == 0) {
            cout << fixed;
            cout.precision(2);

            cout << "\n[STEP " << step << "] t = " << t << " sec\n";
            cout << "  ▸ TCP 위치 [mm]:   " << p_cur.transpose() << endl;
            cout << "  ▸ TCP 자세 [deg]:  " << (rpy_cur * 180.0 / M_PI).transpose() << endl;
            cout << "  ▸ TCP 속도 [mm/s]: " << v_tcp_linear.transpose() << endl;
            cout << "  ▸ 자코비안(앞부분):\n" << J.block<3, 3>(0, 0) << endl;
            cout << "  ▸ dq (rad/s):      " << dq.transpose() << endl;

            cout << "  ▸ dq (deg/s):      ";
            for (double val : dq_deg) cout << val << " ";
            cout << endl;

            cout << "  ▸ q (deg):         ";
            for (double q : q_rad) cout << rad2deg(q) << " ";
            cout << endl;
        }

        // 명령 전송
        robot.move_speed_j(rc, dq_deg, 0.02, 0.04, 0.1, 0.1);

        // 시간 갱신 및 주기 유지
        t += LOOP_DT;
        this_thread::sleep_until(start_time + chrono::microseconds(int(LOOP_DT * 1e6)));
    }

    cout << "[INFO] 이동 완료!" << endl;
    return 0;
}