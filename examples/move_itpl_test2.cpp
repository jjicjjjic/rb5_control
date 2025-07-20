#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace rb;
using namespace Eigen;

// --- deg <-> rad 변환 ---
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad / M_PI * 180.0; }

// --- 링크 정보 구조체 ---
struct LinkDH {
    double a, alpha, d, theta;
    bool isRevolute;
};

// --- vector -> std::array 변환 ---
std::array<double, 6> convert_to_array(const vector<double>& vec) {
    if (vec.size() < 6) {
        throw std::runtime_error("❌ 오류: 벡터 크기가 6보다 작습니다!");
    }
    std::array<double, 6> arr{};
    std::copy_n(vec.begin(), 6, arr.begin());
    return arr;
}
double normalize_angle(double angle) {
    while (angle <= -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
    return angle;
}


// --- 9링크 로봇 정의 클래스 ---
class Robot9Link {
public:
    vector<LinkDH> links;
    vector<double> joint_offset = { 0, -M_PI / 2, 0, M_PI / 2, 0, 0 };

    Robot9Link() {
        links.resize(9);
        links[0] = { 0.0, deg2rad(-90), 169.2, 0.0, true };
        links[1] = { 0.0, deg2rad(0), -148.4, deg2rad(-90), true };
        links[2] = { 425.0, deg2rad(0), 0.0, 0.0, false };
        links[3] = { 0.0, deg2rad(0), 148.4, 0.0, true };
        links[4] = { 392.0, deg2rad(0), 0.0, 0.0, false };
        links[5] = { 0.0, deg2rad(0), -110.7, deg2rad(90), true };
        links[6] = { 0.0, deg2rad(90), 0.0, 0.0, false };
        links[7] = { 0.0, deg2rad(-90), 110.7, 0.0, true };
        links[8] = { 0.0, deg2rad(90), -96.7, 0.0, true };
    }

    Matrix4d dhTransform(double theta, double d, double a, double alpha) {
        double ct = cos(theta), st = sin(theta);
        double ca = cos(alpha), sa = sin(alpha);
        Matrix4d T;
        T << ct, -st * ca, st* sa, a* ct,
            st, ct* ca, -ct * sa, a* st,
            0, sa, ca, d,
            0, 0, 0, 1;
        return T;
    }

    Matrix4d forwardKinematics(const vector<double>& jointVals) {
        int j = 0;
        for (int i = 0; i < links.size(); i++)
            if (links[i].isRevolute)
                links[i].theta = jointVals[j++] + joint_offset[j - 1];
        Matrix4d T = Matrix4d::Identity();
        for (int i = 0; i < 9; i++)
            T *= dhTransform(links[i].theta, links[i].d, links[i].a, links[i].alpha);
        return T;
    }

    Vector3d matrixToRPY(const Matrix4d& T) {
        Matrix3d R = T.block<3, 3>(0, 0);
        double pitch = -asin(R(2, 0));
        double roll, yaw;
        if (abs(cos(pitch)) > 1e-6) {
            roll = atan2(R(2, 1), R(2, 2));
            yaw = atan2(R(1, 0), R(0, 0));
        }
        else {
            roll = 0.0;
            yaw = atan2(-R(0, 1), R(1, 1));
        }
        return Vector3d(roll, pitch, yaw);
    }

    Matrix<double, 6, 6> computeJacobian6x6(const vector<double>& jointVals) {
        int j = 0;
        for (int i = 0; i < links.size(); i++)
            if (links[i].isRevolute)
                links[i].theta = jointVals[j++] + joint_offset[j - 1];

        vector<Matrix4d> T_list(10, Matrix4d::Identity());
        for (int i = 0; i < 9; i++)
            T_list[i + 1] = T_list[i] * dhTransform(links[i].theta, links[i].d, links[i].a, links[i].alpha);
        Vector3d p_end = T_list[9].block<3, 1>(0, 3);

        Matrix<double, 6, 6> J = Matrix<double, 6, 6>::Zero();
        j = 0;
        for (int i = 0; i < 9; i++) {
            if (!links[i].isRevolute) continue;
            Vector3d z = T_list[i].block<3, 1>(0, 2);
            Vector3d p = T_list[i].block<3, 1>(0, 3);
            J.block<3, 1>(0, j) = z.cross(p_end - p);
            J.block<3, 1>(3, j) = z;
            j++;
        }
        return J;
    }

    vector<double> inverseKinematics6D(const Vector3d& pos, const Vector3d& rpy,
        const vector<double>& initQ, int maxIter = 100, double eps = 1e-5) { // 더 정밀한 오차 기준
        vector<double> q = initQ;
        double lambda = 0.2; // 감쇠 계수를 증가하여 안정화

        for (int iter = 0; iter < maxIter; iter++) {
            Matrix4d T_current = forwardKinematics(q);
            Vector3d pos_current = T_current.block<3, 1>(0, 3);
            Vector3d rpy_current = matrixToRPY(T_current);

            Vector<double, 6> e;
            e.head<3>() = pos - pos_current;  // 위치 오차
            e.tail<3>() = rpy - rpy_current;  // 자세 오차

            if (e.norm() < eps) break; // 오차가 작으면 종료

            // Jacobian 계산
            Matrix<double, 6, 6> J = computeJacobian6x6(q);

            // 🔥 마지막 관절이 더 많이 변하도록 가중치 추가
            J(5, 5) += 0.2; // 마지막 관절 변화 강조

            // SVD 기반 Jacobian 역변환
            JacobiSVD<Matrix<double, 6, 6>> svd(J, ComputeThinU | ComputeThinV);
            VectorXd singularValues = svd.singularValues();

            // 작은 특이값 감쇠
            for (int i = 0; i < 6; i++) {
                if (singularValues[i] < 1e-4) singularValues[i] = 1e-4;
            }

            Matrix<double, 6, 6> Jinv = svd.matrixV() * singularValues.asDiagonal().inverse() * svd.matrixU().transpose();

            // 관절 변화량 계산
            Vector<double, 6> dq = Jinv * e;
            for (int i = 0; i < 6; i++) q[i] += dq[i];

            // 🔥 라디안 단위에서 정규화 적용 및 제한 추가
            for (int i = 0; i < 6; i++) {
                q[i] = normalize_angle(q[i]);

                // ✅ 관절 범위 제한: -180도 ~ 180도 (라디안)
                if (q[i] > M_PI) q[i] = M_PI;
                if (q[i] < -M_PI) q[i] = -M_PI;
            }
        }
        return q;
    }
};

// --- Waypoint 생성 (x, y, z 입력 후 IK 계산) ---
vector<vector<double>> create_waypoints(Robot9Link& rm, podo::Cobot<podo::StandardVector>& robot, podo::ResponseCollector& rc) {
    int n;
    cout << "📍 이동할 waypoint 개수 입력: ";
    cin >> n;

    vector<double> joint_deg(6);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J0_ANG, joint_deg[0]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J1_ANG, joint_deg[1]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J2_ANG, joint_deg[2]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J3_ANG, joint_deg[3]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J4_ANG, joint_deg[4]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J5_ANG, joint_deg[5]);

    vector<double> q_guess;
    for (double d : joint_deg) q_guess.push_back(deg2rad(d));

    vector<vector<double>> result;

    for (int i = 0; i < n; ++i) {
        Vector3d pos;
        cout << "➡️ Waypoint " << i + 1 << "의 x y z(mm) 입력: ";
        for (int j = 0; j < 3; ++j) cin >> pos[j];

        Vector3d rpy(0, 0, 0);  // 기본 RPY 설정
        vector<double> q = rm.inverseKinematics6D(pos, rpy, q_guess);

        cout << "   ✅ 계산된 관절각도 (deg): ";
        for (double rad : q) cout << rad2deg(rad) << " ";
        cout << endl;

        vector<double> q_deg;
        for (double rad : q) q_deg.push_back(rad2deg(rad));
        result.push_back(q_deg);
        q_guess = q;
    }

    return result;
}
// --- 메인 함수 ---
int main() {
    try {
        Robot9Link rm;
        podo::Cobot<podo::StandardVector> robot("10.0.2.7");
        podo::ResponseCollector rc;

        robot.set_operation_mode(rc, podo::OperationMode::Simulation); // 또는 Simulation
        robot.set_speed_bar(rc, 0.2);
        robot.flush(rc);

        auto waypoints = create_waypoints(rm, robot, rc);

        robot.move_itpl_clear(rc);

        for (size_t i = 0; i < waypoints.size(); ++i) {
            double blend = (i == waypoints.size() - 1) ? 0.0 : 0.1;
            robot.move_itpl_add(rc, convert_to_array(waypoints[i]), 200, 400, blend);
        }

        robot.move_itpl_run(rc, 0.2, podo::MoveITPLOption::Smooth); // Smooth 모드 실행

        if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success)
            robot.wait_for_move_finished(rc);

        rc.error().throw_if_not_empty();
        cout << "✅ 모든 waypoint 이동 완료!" << endl;
    }
    catch (const std::exception& e) {
        cerr << "❌ 오류 발생: " << e.what() << endl;
        return 1;
    }

    return 0;
}
