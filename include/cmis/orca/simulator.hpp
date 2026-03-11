#pragma once

#include <cstddef>
#include <vector>

#include "cmis/orca/vector2.hpp"

namespace cmis::orca {

using AgentId = std::size_t;

struct AgentConfig {
  double radius{0.5};
  double max_speed{1.0};
  Vector2 initial_position{};
  Vector2 initial_velocity{};
};

struct AgentState {
  Vector2 position{};
  Vector2 velocity{};
  Vector2 preferred_velocity{};
};

class Simulator {
 public:
  explicit Simulator(double time_step = 0.1);

  [[nodiscard]] double timeStep() const noexcept;
  [[nodiscard]] std::size_t numAgents() const noexcept;

  AgentId addAgent(const AgentConfig &config);
  void setPreferredVelocity(AgentId id, const Vector2 &velocity);
  [[nodiscard]] const AgentState &agentState(AgentId id) const;

  void step();

 private:
  struct AgentRecord {
    AgentConfig config;
    AgentState state;
  };

  [[nodiscard]] Vector2 computeCommandVelocity(const AgentRecord &agent) const;
  [[nodiscard]] AgentRecord &agentRecord(AgentId id);
  [[nodiscard]] const AgentRecord &agentRecord(AgentId id) const;

  double time_step_;
  std::vector<AgentRecord> agents_;
};

}  // namespace cmis::orca
