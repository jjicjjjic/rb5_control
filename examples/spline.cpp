// --- 필수 헤더 ---
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

// --- DEG/RAD 변환 ---
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double MAX_JOINT_SPEED = 50.0; // ✅ (단위: deg/s, 필요에 따라 조정)

// degrees <-> radians 변환
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad / M_PI * 180.0; }


struct LinkDH {
    double a;       // 링크 길이 (mm)
    double alpha;   // twist 각도 (rad)
    double d;       // 링크 오프셋 (mm)
    double theta;   // 관절각 (rad) + 오프셋 포함 (변수 or 고정)
    bool isRevolute; // true면 관절 변수(회전), false면 고정
};

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
// --- Cubic Spline 보간 클래스 (1D 기준, 3D는 따로 구성) ---
class CubicSpline {
public:
    vector<double> t, a, b, c, d;

    void set_points(const vector<double>& x, const vector<double>& y) {
        int n = x.size();
        t = x; a = y;
        b.resize(n); c.resize(n); d.resize(n);
        vector<double> h(n - 1), alpha(n - 1), l(n), mu(n), z(n);

        for (int i = 0; i < n - 1; i++) h[i] = x[i + 1] - x[i];
        for (int i = 1; i < n - 1; i++)
            alpha[i] = (3.0 / h[i]) * (a[i + 1] - a[i]) - (3.0 / h[i - 1]) * (a[i] - a[i - 1]);

        l[0] = 1; mu[0] = z[0] = 0;

        for (int i = 1; i < n - 1; i++) {
            l[i] = 2 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n - 1] = 1;
        for (int j = n - 2; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }
    }

    double evaluate(double x_val) {
        int i = lower_bound(t.begin(), t.end(), x_val) - t.begin() - 1;
        if (i < 0) i = 0;
        double dx = x_val - t[i];
        return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    }

    double derivative(double x_val) {
        int i = lower_bound(t.begin(), t.end(), x_val) - t.begin() - 1;
        if (i < 0) i = 0;
        double dx = x_val - t[i];
        return b[i] + 2 * c[i] * dx + 3 * d[i] * dx * dx;
    }
};

// 3D Cubic Spline
class CubicSpline3D {
public:
    CubicSpline sx, sy, sz;

    void set_points(const vector<double>& t, const vector<Vector3d>& points) {
        vector<double> x, y, z;
        for (auto& p : points) {
            x.push_back(p(0));
            y.push_back(p(1));
            z.push_back(p(2));
        }
        sx.set_points(t, x);
        sy.set_points(t, y);
        sz.set_points(t, z);
    }

    Vector3d evaluate(double t_val) {
        return Vector3d(sx.evaluate(t_val), sy.evaluate(t_val), sz.evaluate(t_val));
    }

    Vector3d derivative(double t_val) {
        return Vector3d(sx.derivative(t_val), sy.derivative(t_val), sz.derivative(t_val));
    }
};

// --- 사다리꼴 속도 프로파일 (0 → vmax → 0) ---
class TrapezoidalProfile {
public:
    double s_total, v_max, a_max;
    double t_total, t_accel, t_const, t_decel;

    TrapezoidalProfile(double s_total_, double v_max_, double a_max_)
        : s_total(s_total_), v_max(v_max_), a_max(a_max_) {
        t_accel = v_max / a_max;
        double s_accel = 0.5 * a_max * t_accel * t_accel;

        if (2 * s_accel > s_total) {
            t_accel = sqrt(s_total / a_max);
            t_const = 0;
            t_decel = t_accel;
        }
        else {
            t_const = (s_total - 2 * s_accel) / v_max;
            t_decel = t_accel;
        }
        t_total = t_accel + t_const + t_decel;
    }

    double get_velocity(double t) {
        if (t < t_accel) return a_max * t;
        else if (t < t_accel + t_const) return v_max;
        else if (t < t_total) return v_max - a_max * (t - t_accel - t_const);
        else return 0;
    }

    double get_progress(double t) {
        if (t < t_accel) return 0.5 * a_max * t * t;
        else if (t < t_accel + t_const)
            return 0.5 * a_max * t_accel * t_accel + v_max * (t - t_accel);
        else if (t < t_total) {
            double t1 = t - t_accel - t_const;
            return s_total - 0.5 * a_max * t1 * t1;
        }
        return s_total;
    }

    double get_total_time() const { return t_total; }
};

// Robot9Link 클래스는 이미 위에서 너가 짜둔 걸 그대로 사용하면 돼

// --- Main 함수 ---
int main() {
    podo::Cobot robot("10.0.2.7");
    podo::ResponseCollector rc;
    robot.set_operation_mode(rc, podo::OperationMode::Real);
    robot.set_speed_bar(rc, 0.5);
    robot.flush(rc);

    Robot9Link model;

    // 초기 관절값
    vector<double> q_deg(6), q_rad(6);
    for (int i = 0; i < 6; ++i) {
        robot.get_system_variable(rc, static_cast<podo::SystemVariable>(static_cast<int>(podo::SystemVariable::SD_J0_ANG) + i), q_deg[i]);
        q_rad[i] = q_deg[i] * DEG2RAD;
    }

    // 현재 위치
    Vector3d p_start = model.forwardKinematics(q_rad).block<3, 1>(0, 3);

    // 사용자 입력 Waypoint 받기
    vector<Vector3d> waypoints;
    int N;
    cout << "Enter number of waypoints: ";
    cin >> N;

    waypoints.push_back(p_start); // 시작점 자동 추가
    for (int i = 0; i < N; ++i) {
        Vector3d pt;
        cout << "Waypoint " << (i + 1) << " (x y z): ";
        for (int j = 0; j < 3; ++j) cin >> pt[j];
        waypoints.push_back(pt);
    }

    // 길이 기반으로 시간 스케일링
    vector<double> t_list = { 0.0 };
    double total_length = 0.0;
    for (int i = 1; i < waypoints.size(); ++i) {
        double d = (waypoints[i] - waypoints[i - 1]).norm();
        total_length += d;
        t_list.push_back(t_list.back() + d);
    }

    // 전체 프로파일 시간 계산
    double max_vel = 50.0, max_acc = 100.0;
    TrapezoidalProfile profile(total_length, max_vel, max_acc);
    double total_time = profile.get_total_time();

    // Spline 생성
    CubicSpline3D spline;
    spline.set_points(t_list, waypoints);

    // 루프 시작
    constexpr int LOOP_HZ = 25;
    constexpr double LOOP_DT = 1.0 / LOOP_HZ;
    double t = 0.0;

    while (t < total_time) {
        auto t_start = chrono::steady_clock::now();

        double progress = profile.get_progress(t);
        Vector3d pos_des = spline.evaluate(progress);
        Vector3d vel_des = spline.derivative(progress) * profile.get_velocity(t);

        // 현재 상태
        Matrix4d T_cur = model.forwardKinematics(q_rad);
        Vector3d p_cur = T_cur.block<3, 1>(0, 3);
        Vector3d rpy_cur = model.matrixToRPY(T_cur);

        // 자코비안
        Matrix<double, 6, 6> J = model.computeJacobian6x6(q_rad);
        double lambda = 0.001;
        Matrix<double, 6, 6> I = Matrix<double, 6, 6>::Identity();
        Matrix<double, 6, 6> Jinv = J.transpose() * (J * J.transpose() + lambda * I).inverse();

        // TCP 속도 벡터 (회전 없음)
        Vector<double, 6> v_tcp;
        v_tcp.head<3>() = vel_des;
        v_tcp.tail<3>().setZero();

        // 관절 속도 계산
        Vector<double, 6> dq = Jinv * v_tcp;

        // 관절 속도 제한 및 적분
        array<double, 6> dq_deg;
        for (int i = 0; i < 6; ++i) {
            dq_deg[i] = rad2deg(dq[i]);
            dq_deg[i] = std::clamp(dq_deg[i], -MAX_JOINT_SPEED, MAX_JOINT_SPEED);
            q_rad[i] += dq[i] * LOOP_DT;
        }

        // 디버깅 출력
        if ((int)(t * 1000) % 200 == 0) {
            cout << "\n[Time: " << t << " sec]" << endl;
            cout << "  TCP 위치: " << p_cur.transpose() << " mm" << endl;
            cout << "  TCP 속도: " << vel_des.transpose() << " mm/s" << endl;
            cout << "  관절 속도 (deg/s): ";
            for (double v : dq_deg) cout << v << " ";
            cout << endl;
        }

        // 속도 명령 전송
        robot.move_speed_j(rc, dq_deg, 0.02, 0.04, 1.0, 1.0);

        // 시간 업데이트 및 sleep
        t += LOOP_DT;
        std::this_thread::sleep_until(t_start + chrono::microseconds(int(LOOP_DT * 1e6)));
    }

    cout << "? 모든 waypoint 정확 통과 완료!" << endl;
    return 0;
}