#include <iostream>

#include "cmis/orca/simulator.hpp"

int main() {
  cmis::orca::Simulator simulator(0.5);

  const auto agent_id = simulator.addAgent({
      .radius = 0.4,
      .max_speed = 1.0,
      .initial_position = {0.0, 0.0},
      .initial_velocity = {0.0, 0.0},
  });

  simulator.setPreferredVelocity(agent_id, {1.0, 0.0});
  simulator.step();

  const auto &state = simulator.agentState(agent_id);
  std::cout << "agent=" << agent_id << " position=(" << state.position.x << ", " << state.position.y
            << ") velocity=(" << state.velocity.x << ", " << state.velocity.y << ")\n";
  return 0;
}
