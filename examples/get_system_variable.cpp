#include <iostream>
#include <vector>
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace rb;

int main() {
  try {
    // Make connection
    auto robot = podo::Cobot("10.0.2.7");
    auto rc = podo::ResponseCollector();

    vector<double> joint_q = {0,0,0,0,0,0};
    robot.get_system_variable(rc, podo::SystemVariable::SD_J0_ANG, joint_q[0]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J1_ANG, joint_q[1]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J2_ANG, joint_q[2]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J3_ANG, joint_q[3]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J4_ANG, joint_q[4]);
    robot.get_system_variable(rc, podo::SystemVariable::SD_J5_ANG, joint_q[5]);
    rc = rc.error().throw_if_not_empty();

    cout << joint_q[0] << " " << joint_q[1] << " " << joint_q[2] << " " 
        << joint_q[3] << " " << joint_q[4] << " " << joint_q[5] << endl;
  } catch (const std::exception& e) {
    cerr << e.what() << endl;
    return 1;
  }
  return 0;
}