"""Unit tests for shared state types."""

from __future__ import annotations

import unittest

from cmis_ca.core.geometry import Vector2
from cmis_ca.core.state import AgentCommand, AgentState, SimulationResult


class StateTypesTest(unittest.TestCase):
    def test_agent_state_with_preferred_velocity(self) -> None:
        state = AgentState(
            position=Vector2(1.0, 2.0),
            velocity=Vector2(0.5, 0.0),
            preferred_velocity=Vector2(1.0, 0.0),
        )

        updated = state.with_preferred_velocity(Vector2(0.0, 1.0))

        self.assertEqual(updated.position, state.position)
        self.assertEqual(updated.velocity, state.velocity)
        self.assertEqual(updated.preferred_velocity, Vector2(0.0, 1.0))

    def test_simulation_result_num_steps(self) -> None:
        state = AgentState(position=Vector2(0.0, 0.0), velocity=Vector2(), preferred_velocity=Vector2())
        result = SimulationResult(
            algorithm="orca",
            final_states=(state,),
            history=((state,), (state,), (state,)),
        )

        self.assertEqual(result.num_steps, 2)

    def test_agent_command_holds_velocity(self) -> None:
        command = AgentCommand(velocity=Vector2(1.0, -1.0))
        self.assertEqual(command.velocity, Vector2(1.0, -1.0))


if __name__ == "__main__":
    unittest.main()
