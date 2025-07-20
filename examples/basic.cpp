#include <iostream>
#include <cmath>
#include "rbpodo/rbpodo.hpp"

using namespace rb;
using namespace std::chrono_literals;

int main() {
  try {
    // Make connection
    podo::Cobot robot("10.0.2.7");
    podo::ResponseCollector rc;

    robot.set_operation_mode(rc, podo::OperationMode::Real);
    //robot.set_operation_mode(rc, podo::OperationMode::Simulation);
    robot.set_speed_bar(rc, 1.0);

    robot.flush(rc);

    // Move robot in joint space
    //robot.move_j(rc, {100, -10, -130, 0, -20, 0}, 200, 400);

    double target_position[3] = { 300, 200, 150 };
    double target_orientation[3] = { 0.0, 1.57, 0.0 };

    std::this_thread::sleep_for(2000ms);

    //robot.move_l(rc, {300, 200, 150, 0.0, 1.57, 0.0}, 200, 400);
    //robot.move_l(rc, { 80, -750, 300, 0.0, 1.57, 0.0 }, 200, 400);
    //robot.move_l(rc, { 240, -550, 150, 0.0, 1.57, 0.0 }, 200, 400);
    robot.move_j(rc, { 100, -40, -110, 83, 320, 1.5 }, 200, 400);

    if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success) {
        robot.wait_for_move_finished(rc);
    }

    //std::this_thread::sleep_for(2000ms);
    //robot.move_j(rc, { 100, 20, -100, 20, -20, 0 }, 200, 400);
    //robot.move_l(rc, { 160, -560, 50, 0.0, 1.57, 0.0 }, 200, 400);
    robot.move_j(rc, { 90, -43, -110, 69, 270, 1.5 }, 200, 400);

    if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success) {
      robot.wait_for_move_finished(rc);
    }

    //robot.move_l(rc, { 240, -550, 150, 0.0, 1.57, 0.0 }, 200, 400);
    robot.move_j(rc, { 100, -40, -110, 83, 320, 1.5 }, 200, 400);

    if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success) {
        robot.wait_for_move_finished(rc);
    }

    //robot.move_j(rc, { 100, -10, -130, 0, -20, 0 }, 200, 400);
    //robot.move_l(rc, { 320, -500, 530, 0.0, 1.57, 0.0 }, 200, 400);
    robot.move_j(rc, { 105, -14, -90, 23, 350, 1.5 }, 200, 400);


    if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success) {
        robot.wait_for_move_finished(rc);
    }

    //robot.move_l(rc, { 240, -550, 150, 0.0, 1.57, 0.0 }, 200, 400);
    robot.move_j(rc, { 100, -40, -110, 83, 320, 1.5 }, 200, 400);

    if (robot.wait_for_move_started(rc, 0.1).type() == podo::ReturnType::Success) {
        robot.wait_for_move_finished(rc);
    }

    // If there is any error during above process, throw exception error
    rc.error().throw_if_not_empty();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}