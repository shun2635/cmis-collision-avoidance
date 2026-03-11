#include <cassert>
#include <cmath>

#include "cmis/orca/simulator.hpp"

namespace {

bool nearlyEqual(double lhs, double rhs) {
  return std::abs(lhs - rhs) < 1e-9;
}

}  // namespace

int main() {
  cmis::orca::Simulator simulator(0.5);

  const auto agent_id = simulator.addAgent({
      .radius = 0.4,
      .max_speed = 2.0,
      .initial_position = {0.0, 0.0},
      .initial_velocity = {0.0, 0.0},
  });

  simulator.setPreferredVelocity(agent_id, {1.0, 0.0});
  simulator.step();

  const auto &state = simulator.agentState(agent_id);
  assert(nearlyEqual(state.position.x, 0.5));
  assert(nearlyEqual(state.position.y, 0.0));
  assert(nearlyEqual(state.velocity.x, 1.0));
  assert(nearlyEqual(state.velocity.y, 0.0));
  return 0;
}
