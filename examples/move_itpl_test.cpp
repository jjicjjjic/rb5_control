#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <array>
#include <Eigen/Dense>
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace rb;
using namespace std::chrono_literals;
using namespace Eigen;

// --- deg <-> rad 변환 ---
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad / M_PI * 180.0; }

// --- 링크 정보 구조체 ---
struct LinkDH {
    double a, alpha, d, theta;
    bool isRevolute;
};

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
        const vector<double>& initQ, int maxIter = 1000, double eps = 1e-3) {
        vector<double> q = initQ;
        for (int iter = 0; iter < maxIter; iter++) {
            Matrix4d T = forwardKinematics(q);
            Vector3d p_cur = T.block<3, 1>(0, 3);
            Vector3d rpy_cur = matrixToRPY(T);

            Vector<double, 6> e;
            e.head<3>() = pos - p_cur;
            e.tail<3>() = rpy - rpy_cur;
            if (e.norm() < eps) return q;

            Matrix<double, 6, 6> J = computeJacobian6x6(q);
            Matrix<double, 6, 6> JJt = J * J.transpose();
            if (fabs(JJt.determinant()) < 1e-12) return q;
            Matrix<double, 6, 6> Jinv = J.transpose() * JJt.inverse();
            Vector<double, 6> dq = Jinv * e;
            for (int i = 0; i < 6; i++) q[i] += dq[i];
        }
        return q;
    }
};

// --- 벡터를 배열로 변환 ---
std::array<double, 6> convert_to_array(const std::vector<double>& vec) {
    std::array<double, 6> arr{};
    std::copy_n(vec.begin(), std::min(vec.size(), size_t(6)), arr.begin());
    return arr;
}

// --- 사용자 입력 기반 waypoint 생성 함수 ---
vector<vector<double>> create_waypoints(Robot9Link& rm, podo::Cobot<podo::StandardVector>& robot, podo::ResponseCollector& rc) {
    int n;
    cout << "📍 이동할 waypoint 개수 입력: ";
    cin >> n;

    vector<vector<double>> result;
    vector<double> q_guess(6, 0.0);

    for (int i = 0; i < n; ++i) {
        Vector3d pos;
        cout << "➡️ Waypoint " << i + 1 << "의 x y z(mm) 입력: ";
        for (int j = 0; j < 3; ++j) cin >> pos[j];

        Vector3d rpy(deg2rad(90), 0, 0);
        Vector3d q_pos = rm.forwardKinematics(q_guess).block<3, 1>(0, 3); // Vector3d로 변환
        vector<double> q = { q_pos[0], q_pos[1], q_pos[2] }; // Vector3d -> vector<double>

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

        robot.set_operation_mode(rc, podo::OperationMode::Real); // or Real
        robot.set_speed_bar(rc, 0.2);
        robot.flush(rc);

        auto waypoints = create_waypoints(rm, robot, rc);

        // 1. 이전 이터폴레이션 경로 초기화
        robot.move_itpl_clear(rc);

        // 2. 각 waypoint를 추가
        for (size_t i = 0; i < waypoints.size(); ++i) {
            double blend = (i == waypoints.size() - 1) ? 0.0 : 0.1;
            robot.move_itpl_add(rc, convert_to_array(waypoints[i]), 200, 400, blend);
        }

        // 3. 전체 경로 실행
        robot.move_itpl_run(rc, 0.2, podo::MoveITPLOption::Smooth);


        // 이동 시작 및 완료 대기
        if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success)
            robot.wait_for_move_finished(rc);

        rc.error().throw_if_not_empty();
        cout << "✅ 모든 waypoint 이동 완료!" << endl;
    }
    catch (const std::exception& e) {
        cerr << "❌ 오류: " << e.what() << endl;
        return 1;
    }
    return 0;
}
