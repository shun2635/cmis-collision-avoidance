"""PyQtGraph-based playback viewer for simulation traces."""

from __future__ import annotations

from typing import Any

from cmis_ca.core.geometry import Vector2
from cmis_ca.visualization.models import VisualizationTrace


def launch_pyqtgraph_viewer(trace: VisualizationTrace, playback_fps: float = 30.0) -> None:
    """Launch an interactive PyQtGraph viewer for one trace."""

    try:
        import pyqtgraph as pg
        from PySide6 import QtCore, QtWidgets
    except ImportError as exc:
        raise RuntimeError(
            "PyQtGraph visualization requires `pyqtgraph` and `PySide6`. "
            "Run `poetry install` again after updating dependencies."
        ) from exc

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = QtWidgets.QWidget()
    window.setWindowTitle(f"CMIS Visualization: {trace.scenario_name} ({trace.algorithm})")

    root_layout = QtWidgets.QVBoxLayout(window)
    plot_widget = pg.PlotWidget()
    plot_widget.setAspectLocked(True)
    plot_widget.showGrid(x=True, y=True, alpha=0.2)
    plot_widget.setLabel("left", "y")
    plot_widget.setLabel("bottom", "x")
    root_layout.addWidget(plot_widget)

    controls_layout = QtWidgets.QHBoxLayout()
    play_button = QtWidgets.QPushButton("Play")
    controls_layout.addWidget(play_button)
    step_label = QtWidgets.QLabel("Step")
    controls_layout.addWidget(step_label)
    slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    slider.setMinimum(0)
    slider.setMaximum(max(0, trace.num_frames - 1))
    controls_layout.addWidget(slider, stretch=1)
    speed_label = QtWidgets.QLabel("Speed")
    controls_layout.addWidget(speed_label)
    speed_box = QtWidgets.QDoubleSpinBox()
    speed_box.setRange(1.0, 120.0)
    speed_box.setValue(playback_fps)
    speed_box.setSuffix(" fps")
    controls_layout.addWidget(speed_box)
    root_layout.addLayout(controls_layout)

    initial_item = pg.ScatterPlotItem(
        pen=pg.mkPen(None),
        brush=pg.mkBrush(160, 160, 160, 90),
        pxMode=False,
    )
    trail_item = pg.PlotDataItem(
        pen=pg.mkPen(70, 110, 180, 90, width=1.5),
        connect="finite",
    )
    current_item = pg.ScatterPlotItem(
        pen=pg.mkPen(20, 30, 60),
        brush=pg.mkBrush(70, 110, 180, 220),
        pxMode=False,
    )
    plot_widget.addItem(initial_item)
    plot_widget.addItem(trail_item)
    plot_widget.addItem(current_item)

    for obstacle in trace.obstacles:
        path_points = obstacle.points + ((obstacle.points[0],) if obstacle.closed else ())
        plot_widget.plot(
            [point.x for point in path_points],
            [point.y for point in path_points],
            pen=pg.mkPen(30, 30, 30, width=2),
        )

    initial_item.setData(
        spots=_build_agent_spots(trace.initial_positions, trace.agent_radii),
    )

    _fit_plot(plot_widget, trace)

    timer = QtCore.QTimer()
    timer.setInterval(max(1, int(1000.0 / playback_fps)))
    state: dict[str, Any] = {"playing": False}

    def set_frame(frame_index: int) -> None:
        frame = trace.frames[frame_index]
        current_item.setData(
            spots=_build_agent_spots(frame.positions, trace.agent_radii),
        )
        trail_x, trail_y = _build_trail_arrays(trace, frame_index)
        trail_item.setData(trail_x, trail_y)
        step_label.setText(
            f"Step {frame.step_index}/{trace.num_frames - 1}  t={frame.global_time:.2f}s"
        )

    def toggle_playback() -> None:
        state["playing"] = not state["playing"]
        if state["playing"]:
            timer.start()
            play_button.setText("Pause")
        else:
            timer.stop()
            play_button.setText("Play")

    def advance_frame() -> None:
        if slider.value() >= trace.num_frames - 1:
            state["playing"] = False
            timer.stop()
            play_button.setText("Play")
            return
        slider.setValue(slider.value() + 1)

    def update_timer_interval(value: float) -> None:
        timer.setInterval(max(1, int(1000.0 / value)))

    slider.valueChanged.connect(set_frame)
    play_button.clicked.connect(toggle_playback)
    speed_box.valueChanged.connect(update_timer_interval)
    timer.timeout.connect(advance_frame)

    set_frame(0)
    window.resize(960, 720)
    window.show()
    app.exec()


def _build_trail_arrays(trace: VisualizationTrace, frame_index: int) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for agent_index in range(trace.num_agents):
        for frame in trace.frames[: frame_index + 1]:
            position = frame.positions[agent_index]
            xs.append(position.x)
            ys.append(position.y)
        xs.append(float("nan"))
        ys.append(float("nan"))
    return xs, ys


def _build_agent_spots(
    positions: tuple[Vector2, ...],
    radii: tuple[float, ...],
) -> list[dict[str, object]]:
    if len(positions) != len(radii):
        raise ValueError("positions and radii must have the same length")
    return [
        {
            "pos": (position.x, position.y),
            "size": radius * 2.0,
        }
        for position, radius in zip(positions, radii, strict=True)
    ]


def _fit_plot(plot_widget: Any, trace: VisualizationTrace) -> None:
    all_points: list[Vector2] = list(trace.initial_positions)
    for obstacle in trace.obstacles:
        all_points.extend(obstacle.points)
    for frame in trace.frames:
        all_points.extend(frame.positions)

    if not all_points:
        return

    min_x = min(point.x - radius for point, radius in zip(trace.initial_positions, trace.agent_radii, strict=True))
    max_x = max(point.x + radius for point, radius in zip(trace.initial_positions, trace.agent_radii, strict=True))
    min_y = min(point.y - radius for point, radius in zip(trace.initial_positions, trace.agent_radii, strict=True))
    max_y = max(point.y + radius for point, radius in zip(trace.initial_positions, trace.agent_radii, strict=True))
    for frame in trace.frames:
        min_x = min(
            min_x,
            min(
                point.x - radius
                for point, radius in zip(frame.positions, trace.agent_radii, strict=True)
            ),
        )
        max_x = max(
            max_x,
            max(
                point.x + radius
                for point, radius in zip(frame.positions, trace.agent_radii, strict=True)
            ),
        )
        min_y = min(
            min_y,
            min(
                point.y - radius
                for point, radius in zip(frame.positions, trace.agent_radii, strict=True)
            ),
        )
        max_y = max(
            max_y,
            max(
                point.y + radius
                for point, radius in zip(frame.positions, trace.agent_radii, strict=True)
            ),
        )
    for point in all_points:
        min_x = min(min_x, point.x)
        max_x = max(max_x, point.x)
        min_y = min(min_y, point.y)
        max_y = max(max_y, point.y)
    margin_x = max(1.0, (max_x - min_x) * 0.1)
    margin_y = max(1.0, (max_y - min_y) * 0.1)
    plot_widget.setXRange(min_x - margin_x, max_x + margin_x, padding=0.0)
    plot_widget.setYRange(min_y - margin_y, max_y + margin_y, padding=0.0)
