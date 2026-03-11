#include "cmis/orca/simulator.hpp"

#include <stdexcept>
#include <vector>

namespace cmis::orca {

namespace {

Vector2 clampToMaxSpeed(const Vector2 &velocity, double max_speed) {
  const double speed = velocity.norm();
  if (speed <= max_speed || speed == 0.0) {
    return velocity;
  }

  return velocity * (max_speed / speed);
}

}  // namespace

Simulator::Simulator(double time_step) : time_step_(time_step) {
  if (time_step_ <= 0.0) {
    throw std::invalid_argument("time_step must be positive");
  }
}

double Simulator::timeStep() const noexcept { return time_step_; }

std::size_t Simulator::numAgents() const noexcept { return agents_.size(); }

AgentId Simulator::addAgent(const AgentConfig &config) {
  AgentState state;
  state.position = config.initial_position;
  state.velocity = clampToMaxSpeed(config.initial_velocity, config.max_speed);
  state.preferred_velocity = state.velocity;

  agents_.push_back(AgentRecord{config, state});
  return agents_.size() - 1;
}

void Simulator::setPreferredVelocity(AgentId id, const Vector2 &velocity) {
  agentRecord(id).state.preferred_velocity = velocity;
}

const AgentState &Simulator::agentState(AgentId id) const {
  return agentRecord(id).state;
}

void Simulator::step() {
  std::vector<Vector2> next_velocities;
  next_velocities.reserve(agents_.size());

  for (const auto &agent : agents_) {
    next_velocities.push_back(computeCommandVelocity(agent));
  }

  for (std::size_t i = 0; i < agents_.size(); ++i) {
    agents_[i].state.velocity = next_velocities[i];
    agents_[i].state.position += next_velocities[i] * time_step_;
  }
}

Vector2 Simulator::computeCommandVelocity(const AgentRecord &agent) const {
  // Initial scaffold only. ORCA constraints and LP solver will be added later.
  return clampToMaxSpeed(agent.state.preferred_velocity, agent.config.max_speed);
}

Simulator::AgentRecord &Simulator::agentRecord(AgentId id) {
  if (id >= agents_.size()) {
    throw std::out_of_range("agent id is out of range");
  }

  return agents_[id];
}

const Simulator::AgentRecord &Simulator::agentRecord(AgentId id) const {
  if (id >= agents_.size()) {
    throw std::out_of_range("agent id is out of range");
  }

  return agents_[id];
}

}  // namespace cmis::orca
