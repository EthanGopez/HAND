// No sockets or robot are used. Inject failure beneath the real SDK decoder.
#include <algorithm>
#include <cassert>
#include <iostream>
#include "xarm/core/instruction/uxbus_cmd.h"

class TimeoutTransport : public UxbusCmd {
  int _send_modbus_request(unsigned char, unsigned char*, unsigned short, int) override {
    return 1;
  }
  int _recv_modbus_response(unsigned char, unsigned short, unsigned char*,
                            unsigned short, int, int) override {
    return UXBUS_STATE::ERR_TOUT;
  }
};

int main() {
  TimeoutTransport transport;
  float position[7], velocity[7], effort[7];
  std::fill_n(position, 7, 0.25f);
  std::fill_n(velocity, 7, 0.5f);
  std::fill_n(effort, 7, 0.75f);
  const int result = transport.get_joint_states(position, velocity, effort, 3);
  assert(result == 3);
  for (int i = 0; i < 7; ++i) {
    assert(position[i] == 0.0f);
    assert(velocity[i] == 0.0f);
    assert(effort[i] == 0.0f);
  }
  std::cout << "Reproduced: failed read returns 3 and overwrites all 21 values with zeros.\n";
}
