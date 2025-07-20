#include <iostream>

#include "rbpodo/rbpodo.hpp"

using namespace rb;
using namespace std::chrono_literals;

int main() {
  try {
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

    robot.set_operation_mode(rc, podo::OperationMode::Real);
    robot.set_speed_bar(rc, 0.5);
    rc = rc.error().throw_if_not_empty();

    std::array<double, 6> tcp_pose{};
    robot.get_tcp_info(rc, tcp_pose);
    std::cout << "Current TCP Pose: ";
    for (const auto& value : tcp_pose) {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    std::this_thread::sleep_for(2000ms);

    // Go home pose
    robot.move_j(rc, {100, -10, -130, 0, -20, 0}, 50, 100);
    //robot.move_jl(rc, { 100, 200, 300, 0, 0, 0 }, 20, 5);
    //tcp_pose[0] -= 100.0;
    //robot.move_jl(rc, tcp_pose, 20, 5);
    if (robot.wait_for_move_started(rc, 0.1).is_success())
      robot.wait_for_move_finished(rc);

    std::this_thread::sleep_for(2000ms);

    //// move_servo_j
    robot.disable_waiting_ack(rc);
    for (int i = 0; i < 1000; i++) {
      robot.move_servo_j(rc, {100, -10 + (double)i * 30. / 1000., -130 + (double)i * 30. / 1000., (double)i * 20. / 1000., -20, 0 }, 0.01, 0.1, 1.0, 1.0);
      std::this_thread::sleep_for(2ms);
    }

    std::this_thread::sleep_for(2000ms);

    robot.move_speed_j(rc, {0, 0, 0, 0, 0, 0}, 0.01, 0.1, 1.0, 1.0);
    robot.enable_waiting_ack(rc);
    robot.wait_for_move_finished(rc);
    rc.clear();


    //std::array<double, 6> tcp_pose{};
    robot.get_tcp_info(rc, tcp_pose);
    std::cout << "Current TCP Pose: ";
    for (const auto& value : tcp_pose) {
        std::cout << value << " ";
    }
    std::cout << std::endl;


    ////robot.move_j(rc, {100, 0, 0, 0, 0, 0}, 50, 100);
    //robot.move_j(rc, { 100, -10, -130, 0, -20, 0 }, 50, 100);
    //if (robot.wait_for_move_started(rc, 0.1).is_success())
    //  robot.wait_for_move_finished(rc);

    std::array<double, 6> joint_refs{};  // 6개 조인트 값을 저장할 배열

    // 각 조인트의 Reference 값 가져오기
    robot.get_system_variable(rc, podo::SystemVariable::SD_J0_REF, joint_refs[0]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J1_REF, joint_refs[1]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J2_REF, joint_refs[2]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J3_REF, joint_refs[3]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J4_REF, joint_refs[4]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J5_REF, joint_refs[5]);

    rc = rc.error().throw_if_not_empty();

    // 출력
    std::cout << "Joint Reference Values: ";
    for (const auto& value : joint_refs) {
        std::cout << value << " ";
    }
    std::cout << std::endl;


    std::this_thread::sleep_for(500ms);

    robot.move_j(rc, joint_refs, 50, 100);

    std::this_thread::sleep_for(2000ms);

    if (robot.wait_for_move_started(rc, 0.1).is_success())
        robot.wait_for_move_finished(rc);

    //// move_servo_j
    robot.disable_waiting_ack(rc);
    for (int i = 0; i < 1000; i++) {
        robot.move_servo_j(rc, { 100, 20 - (double)i * 30. / 1000., -100 - (double)i * 30. / 1000., 20 - (double)i * 20. / 1000., -20, 0 }, 0.01, 0.1, 1.0, 1.0);
        std::this_thread::sleep_for(2ms);
    }

    robot.move_speed_j(rc, { 0, 0, 0, 0, 0, 0 }, 0.01, 0.1, 1.0, 1.0);
    robot.enable_waiting_ack(rc);
    robot.wait_for_move_finished(rc);
    rc.clear();


    //std::array<double, 6> tcp_pose{};
    robot.get_tcp_info(rc, tcp_pose);
    std::cout << "Current TCP Pose: ";
    for (const auto& value : tcp_pose) {
        std::cout << value << " ";
    }
    std::cout << std::endl;



    rc = rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}