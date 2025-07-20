
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "rbpodo/rbpodo.hpp"
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <chrono>

using namespace rb;
using namespace std;
using namespace Eigen;
using json = nlohmann::json;
// Mainjh1.cpp 맨 위 전역 위치에 추가
podo::Cobot<podo::StandardVector>* g_robot_ptr = nullptr;




constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr int PORT = 12345;

inline double deg2rad(double deg) { return deg * DEG2RAD; }
inline double rad2deg(double rad) { return rad * RAD2DEG; }


// 수신값 저장
std::atomic<bool> running{true};
podo::ResponseCollector rc;
podo::Cobot robot("10.0.2.7");
std::array<double, 6> pnt{};



// 안전한 RPY 변환
Vector3d quaternionToRPY(const Quaterniond& q) {
    double sinr = 2*(q.w()*q.x() + q.y()*q.z());
    double cosr = 1 - 2*(q.x()*q.x() + q.y()*q.y());
    double roll  = atan2(sinr, cosr);

    double sinp = 2*(q.w()*q.y() - q.z()*q.x());
    double pitch = abs(sinp) >= 1 ? copysign(M_PI/2, sinp) : asin(sinp);

    double siny = 2*(q.w()*q.z() + q.x()*q.y());
    double cosy = 1 - 2*(q.y()*q.y() + q.z()*q.z());
    double yaw   = atan2(siny, cosy);

    return Vector3d(roll, pitch, yaw);
}

// 링크 정보를 담는 구조체
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
        Matrix4d T_end = T_list[9];  // 기존 TCP 위치

        // 🎯 오프셋 적용
        // Matrix4d T_offset = Matrix4d::Identity();
        // T_offset(1, 3) = -11.0;
        // T_end = T_end * T_offset;

        // 🎯 오프셋 반영된 spoon 끝단 위치로 p_end 설정
        Vector3d p_end = T_end.block<3, 1>(0, 3);

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
            
            Vector<double, 6> ae;
            ae << q[0]-initialQ[0],
                  q[1]-initialQ[1],
                  q[2]-initialQ[2],
                  q[3]-initialQ[3],
                  q[4]-initialQ[4],
                  q[5]-initialQ[5];
            
                  
            if (e.norm()< eps) {
                cout << "[IK] Converged at iteration " << iter
                    << ", error norm = " << e.norm() << endl;
                return q;
            }
            
            // 5) 자코비언(6x6)
            Matrix<double, 6, 6> J = computeJacobian6x6(q);
            // for(int i=0;i<6;i++){
            //     J(3,i) = 0;
            // }

            // 6) 의사역행렬
            //    (J * J^T)가 역가역이 안 되는 특이점에서 문제 발생 가능
            Matrix<double, 6, 6> JtJ = J.transpose()*J;
            if (fabs(JtJ.determinant()) < 1e-12) {
                cout << "[IK] Near-singular or singular Jacobian. Stop.\n";
                return q;
            }
            Matrix<double, 6, 6> Jinv = JtJ.inverse() * J.transpose() ;

            // for(int i=0;i<6;i++){
            //     Jinv(1,i) = 0;
            // }

            // 7) 관절 업데이트 Δq = J^+ * e
            Vector<double, 6> dq = Jinv * e;
            // std::cout<<dq<<std::endl;
            for (int i = 0; i < 6; i++) {
                q[i] += dq(i);
            }
            // std::cout<<q[1]<<std::endl;
        }

        cout << "[IK] Failed to converge after " << maxIter << " iterations.\n";
        return q;
    }

    vector<double> inverseKinematics3D_pos(const Vector3d& target_position,
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

            // 3-constraint
            double err1, err2, err3, err4, err5, err6;
            if(floor(q[0]*RAD2DEG)>=-180 && floor(q[0]*RAD2DEG)<=0){err1 = 0;}else{err1 = 100;}
            if(floor(q[1]*RAD2DEG)>=-45 && floor(q[1]*RAD2DEG)<=45){err2 = 0;}else{err2 = 100;}
            if(floor(q[2]*RAD2DEG)>=-150 && floor(q[2]*RAD2DEG)<=0){err3 = 0;}else{err3 = 100;}
            // if(floor(q[3]*RAD2DEG)>=90 && floor(q[3]*RAD2DEG)<=180){err4 = 0;}else{err4 = 1;}
            // if(floor(q[4]*RAD2DEG)>=90 && floor(q[4]*RAD2DEG)<=180){err5 = 0;}else{err5 = 1;}
            // if(floor(q[5]*RAD2DEG)>=90 && floor(q[5]*RAD2DEG)<=180){err6 = 0;}else{err6 = 1;}

            // 4) 종합 오차 (6x1)
            Vector<double, 6> e;
            e << pos_err(0), pos_err(1), pos_err(2),
                ori_err(0), ori_err(1), ori_err(2);
            
            Vector<double, 6> ae;
            ae << q[0]-initialQ[0],
                  q[1]-initialQ[1],
                  q[2]-initialQ[2],
                  q[3]-initialQ[3],
                  q[4]-initialQ[4],
                  q[5]-initialQ[5];

            if (e.norm() + 1000*ae.norm()< eps) {
                cout << "[IK] Converged at iteration " << iter
                    << ", error norm = " << e.norm() << endl;
                return q;
            }

            // 5) 자코비언(6x6)
            Matrix<double, 6, 6> J = computeJacobian6x6(q);
            // for(int i=0;i<6;i++){
            //     J(3,i) = 0;
            // }

            // 6) 의사역행렬
            //    (J * J^T)가 역가역이 안 되는 특이점에서 문제 발생 가능
            Matrix<double, 6, 6> JtJ = J.transpose()*J;
            if (fabs(JtJ.determinant()) < 1e-12) {
                cout << "[IK] Near-singular or singular Jacobian. Stop.\n";
                return q;
            }
            Matrix<double, 6, 6> Jinv = JtJ.inverse() * J.transpose() ;

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

// (필요한 헤더와 LowPassFilterVec3, Robot9Link 클래스 등은 기존 코드와 동일합니다.)

// === 제어 상수 ===
constexpr int LOOP_HZ = 100;           // 제어 루프 주파수 (예: 100Hz)
constexpr double LOOP_DT = 1.0 / LOOP_HZ; // 제어 주기
constexpr double MAX_JOINT_SPEED = 50.0; // deg/s
constexpr double LAMBDA = 0.001;


void executeStaticMove(
    Robot9Link& rm,
    podo::Cobot<podo::StandardVector>& robot,
    podo::ResponseCollector& rc,
    std::vector<double>& q_current,
    const Vector3d& target_pos,
    const Quaterniond& target_q
) {
    std::cout << "[DEBUG] executeStaticMove 시작, q_current = [";
    for (double q: q_current) std::cout << q << " ";
    std::cout << "], target_pos = " << target_pos.transpose()
              << ", target_q (as RPY) = " 
              << rad2deg( target_q.toRotationMatrix().eulerAngles(2,1,0)[2] ) << ","
              << rad2deg( target_q.toRotationMatrix().eulerAngles(2,1,0)[1] ) << ","
              << rad2deg( target_q.toRotationMatrix().eulerAngles(2,1,0)[0] )
              << "\n";
    // 1) 시작 위치·자세
    Matrix4d T0 = rm.forwardKinematics(q_current);
    Vector3d start_pos = T0.block<3, 1>(0, 3);
    Quaterniond start_q(T0.block<3, 3>(0, 0));

    // 2) 전체 경로 길이
    double total_len = (target_pos - start_pos).norm();
    if (total_len < 1e-3) return;

    // 3) Waypoint 개수 (1 mm 간격)
    int N = std::max(1, int(total_len));

    // 4) Waypoint 생성 (Position: 선형, Orientation: SLERP)
    std::vector<Vector3d> wp_pos(N);
    std::vector<Quaterniond> wp_q(N);
    for (int i = 1; i <= N; ++i) {
        double s = double(i) / N;
        wp_pos[i - 1] = start_pos + s * (target_pos - start_pos);
    
        Quaterniond q2 = target_q;
        if (start_q.dot(q2) < 0) q2.coeffs() = -q2.coeffs();
        wp_q[i - 1] = start_q.slerp(s, q2);  // ← 바뀐 부분
    }
    
    
    // 5) Waypoint 순회
    for (int i = 0; i < N; ++i) {
        // 5.1) IK
        Vector3d rpy = quaternionToRPY(wp_q[i]);
        std::cout << "[DEBUG] WP[" << i << "] pos=" << wp_pos[i].transpose()
            << " rpy(deg)="
            << rad2deg(rpy(0)) << "," << rad2deg(rpy(1)) << "," << rad2deg(rpy(2))
            << " q_current(deg)=[";
        for (double q : q_current) std::cout << q*RAD2DEG << " ";
        std::cout << "]\n";
        auto q_next = rm.inverseKinematics6D(
            wp_pos[i], rpy, q_current, 50, 1e-3
        );

        // 5.2) Progress 계산 (0→1)
        Vector3d cur_tcp = rm.forwardKinematics(q_next)
            .block<3, 1>(0, 3);
        double done = (start_pos - cur_tcp).norm();
        double prog = std::clamp(done / total_len, 0.0, 1.0);

        // 5.3) Alpha 선형 보간
        constexpr double αmin = 0.05, αmax = 0.95;
        double alpha = αmin + (αmax - αmin) * prog;

        // 5.4) deg 변환
        std::array<double, 6> q_deg;
        for (int j = 0; j < 6; ++j)
            q_deg[j] = q_next[j] * RAD2DEG;

        // 5.5) Servo_J 호출
        robot.move_servo_j(rc,
            { q_deg[0],q_deg[1],q_deg[2],
             q_deg[3],q_deg[4],q_deg[5] },
            0.01, 0.05, 100.0, alpha
        );

        // 5.6) 100 Hz 유지
        std::this_thread::sleep_for(10ms);

        // 5.7) 상태 갱신
        q_current = q_next;
    }
}

int main() {
    try {
        Robot9Link rm;
        podo::Cobot robot("10.0.2.7");
        podo::ResponseCollector rc;
        std::array<double, 6> pnt{};

        //robot.set_operation_mode(rc, podo::OperationMode::Real);
        robot.set_operation_mode(rc, podo::OperationMode::Simulation);
        robot.set_speed_bar(rc, 1);
        rc.error().throw_if_not_empty();
        
        //현재 관절값 가져오기
        vector<double> Initial_Angle(6);
        double out;
        for (int i = 0; i < 6; ++i) {
            robot.get_system_variable(rc,
                static_cast<podo::SystemVariable>(static_cast<int>(podo::SystemVariable::SD_J0_REF) + i), out);
            Initial_Angle[i] = out * DEG2RAD;
        }

        cout << "[Notify] Current Angle: ";
        for (double a : Initial_Angle) cout << a * RAD2DEG << ", ";
        cout << endl;
        

        //초기 위치로 이동
        Vector3d Base_Pose(-100, 400, 400);
        Vector3d Base_RPY(deg2rad(90), deg2rad(0), deg2rad(0));
        vector<double> BA = rm.inverseKinematics6D(Base_Pose, Base_RPY, Initial_Angle, 1000, 1e-3);

        for (double& a : BA) a = round(a * RAD2DEG);
        cout << "[Notify] Target Angle: ";
        for (double a : BA) cout << a << ", ";
        cout << endl;

        robot.move_j(rc, {BA[0], BA[1], BA[2], BA[3], BA[4], BA[5]}, 100, 100);
        if (robot.wait_for_move_started(rc, 1.0).is_success())
            robot.wait_for_move_finished(rc);

        //이동완료 후 관절값 다시 읽기
        vector<double> q_current(6);
        for (int i = 0; i < 6; ++i) {
            robot.get_system_variable(rc,
                static_cast<podo::SystemVariable>(static_cast<int>(podo::SystemVariable::SD_J0_REF) + i), out);
            q_current[i] = out * DEG2RAD;
        }

        // ✅ 정해진 trajectory 설정 (원하는 만큼 추가 가능)
        std::vector<Vector3d> traj_positions = {
            {0, 400, 400},
            {50, 450, 400},
            {0, 400, 450}
        };

        std::vector<Vector3d> traj_rpys = {
            {deg2rad(90), deg2rad(0), deg2rad(0)},
            {deg2rad(90), deg2rad(10), deg2rad(0)},
            {deg2rad(90), deg2rad(0), deg2rad(10)}
        };

        // ✅ Trajectory 실행
        for (size_t i = 0; i < traj_positions.size(); ++i) {
            Vector3d pos = traj_positions[i];
            Vector3d rpy = traj_rpys[i];

            Quaterniond qx(AngleAxisd(rpy[0], Vector3d::UnitX()));
            Quaterniond qy(AngleAxisd(rpy[1], Vector3d::UnitY()));
            Quaterniond qz(AngleAxisd(rpy[2], Vector3d::UnitZ()));
            Quaterniond target_q = qz * qy * qx;

            executeStaticMove(rm, robot, rc, q_current, pos, target_q);
        }

        cout << "Trajectory 수행 완료. 프로그램 종료.\n";
        return 0;
    }
    catch (const exception& e) {
        cerr << "[Error] " << e.what() << endl;
        return 1;
    }
}
