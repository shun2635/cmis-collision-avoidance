"""Tests for PyQtGraph viewer helpers."""

from __future__ import annotations

import pytest

from cmis_ca.core.geometry import Vector2
from cmis_ca.visualization.models import VisualizationFrame, VisualizationTrace
from cmis_ca.visualization.pyqtgraph_viewer import (
    _build_agent_spots,
    _build_ffmpeg_export_command,
    _fit_plot,
    save_trace_animation,
)


class _FakePlotWidget:
    def __init__(self) -> None:
        self.x_range: tuple[float, float, float] | None = None
        self.y_range: tuple[float, float, float] | None = None

    def setXRange(self, minimum: float, maximum: float, *, padding: float) -> None:
        self.x_range = (minimum, maximum, padding)

    def setYRange(self, minimum: float, maximum: float, *, padding: float) -> None:
        self.y_range = (minimum, maximum, padding)


def test_build_agent_spots_uses_diameter_in_world_units() -> None:
    spots = _build_agent_spots(
        (Vector2(1.0, 2.0), Vector2(-3.0, 4.0)),
        (0.5, 2.0),
    )

    assert spots == [
        {"pos": (1.0, 2.0), "size": 1.0},
        {"pos": (-3.0, 4.0), "size": 4.0},
    ]


def test_fit_plot_includes_agent_radius_in_bounds() -> None:
    trace = VisualizationTrace(
        scenario_name="viewer-radius",
        algorithm="cnav",
        time_step=0.1,
        agent_names=("agent_0",),
        agent_radii=(2.0,),
        initial_positions=(Vector2(0.0, 0.0),),
        obstacles=(),
        frames=(
            VisualizationFrame(
                step_index=0,
                global_time=0.0,
                positions=(Vector2(5.0, 0.0),),
            ),
        ),
    )
    plot_widget = _FakePlotWidget()

    _fit_plot(plot_widget, trace)

    assert plot_widget.x_range is not None
    assert plot_widget.y_range is not None
    assert plot_widget.x_range[0] <= -2.0
    assert plot_widget.x_range[1] >= 7.0
    assert plot_widget.y_range[0] <= -2.0
    assert plot_widget.y_range[1] >= 2.0


def test_build_ffmpeg_export_command_for_gif() -> None:
    command = _build_ffmpeg_export_command(
        frame_pattern="/tmp/frame_%05d.png",
        output_path="artifacts/demo.gif",
        playback_fps=12.5,
    )

    assert command[:6] == ["ffmpeg", "-y", "-framerate", "12.5", "-i", "/tmp/frame_%05d.png"]
    assert command[-2:] == ["0", "artifacts/demo.gif"]


def test_save_trace_animation_rejects_unsupported_extension(tmp_path) -> None:
    trace = VisualizationTrace(
        scenario_name="viewer-radius",
        algorithm="cnav",
        time_step=0.1,
        agent_names=("agent_0",),
        agent_radii=(2.0,),
        initial_positions=(Vector2(0.0, 0.0),),
        obstacles=(),
        frames=(
            VisualizationFrame(
                step_index=0,
                global_time=0.0,
                positions=(Vector2(5.0, 0.0),),
            ),
        ),
    )

    with pytest.raises(ValueError, match="supports only \\.gif and \\.mp4"):
        save_trace_animation(trace, str(tmp_path / "demo.webm"))
