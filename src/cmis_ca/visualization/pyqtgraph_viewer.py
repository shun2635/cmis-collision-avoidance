"""PyQtGraph-based playback viewer for simulation traces."""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path
from typing import Any

from cmis_ca.core.geometry import Vector2
from cmis_ca.visualization.models import VisualizationTrace


DEFAULT_ANIMATION_SIZE = (960, 720)


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


def save_trace_animation(
    trace: VisualizationTrace,
    output_path: str,
    playback_fps: float = 30.0,
    *,
    image_size: tuple[int, int] = DEFAULT_ANIMATION_SIZE,
) -> None:
    """Render one trace to an animation file using Qt and ffmpeg."""

    destination = Path(output_path)
    extension = destination.suffix.lower()
    if extension not in {".gif", ".mp4"}:
        raise ValueError("--save-animation supports only .gif and .mp4 output paths")
    if trace.num_frames == 0:
        raise ValueError("cannot export an animation for a trace with no frames")

    destination.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="cmis_ca_animation_") as temp_dir:
        frame_pattern = str(Path(temp_dir) / "frame_%05d.png")
        for frame_index in range(trace.num_frames):
            frame_path = Path(temp_dir) / f"frame_{frame_index:05d}.png"
            image = _render_trace_frame(trace, frame_index, image_size=image_size)
            if not image.save(str(frame_path), "PNG"):
                raise RuntimeError(f"failed to save intermediate frame: {frame_path}")
        _run_ffmpeg_export(
            _build_ffmpeg_export_command(
                frame_pattern=frame_pattern,
                output_path=str(destination),
                playback_fps=playback_fps,
            )
        )


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
    bounds = _compute_plot_bounds(trace)
    if bounds is None:
        return
    min_x, max_x, min_y, max_y = bounds
    plot_widget.setXRange(min_x, max_x, padding=0.0)
    plot_widget.setYRange(min_y, max_y, padding=0.0)


def _compute_plot_bounds(trace: VisualizationTrace) -> tuple[float, float, float, float] | None:
    all_points: list[Vector2] = list(trace.initial_positions)
    for obstacle in trace.obstacles:
        all_points.extend(obstacle.points)
    for frame in trace.frames:
        all_points.extend(frame.positions)

    if not all_points:
        return None

    min_x = min(point.x for point in all_points)
    max_x = max(point.x for point in all_points)
    min_y = min(point.y for point in all_points)
    max_y = max(point.y for point in all_points)
    for position, radius in zip(trace.initial_positions, trace.agent_radii, strict=True):
        min_x = min(min_x, position.x - radius)
        max_x = max(max_x, position.x + radius)
        min_y = min(min_y, position.y - radius)
        max_y = max(max_y, position.y + radius)
    for frame in trace.frames:
        for position, radius in zip(frame.positions, trace.agent_radii, strict=True):
            min_x = min(min_x, position.x - radius)
            max_x = max(max_x, position.x + radius)
            min_y = min(min_y, position.y - radius)
            max_y = max(max_y, position.y + radius)
    margin_x = max(1.0, (max_x - min_x) * 0.1)
    margin_y = max(1.0, (max_y - min_y) * 0.1)
    return (min_x - margin_x, max_x + margin_x, min_y - margin_y, max_y + margin_y)


def _render_trace_frame(
    trace: VisualizationTrace,
    frame_index: int,
    *,
    image_size: tuple[int, int],
):
    from PySide6 import QtCore, QtGui

    width, height = image_size
    image = QtGui.QImage(width, height, QtGui.QImage.Format.Format_ARGB32)
    image.fill(QtGui.QColor(248, 249, 251))

    bounds = _compute_plot_bounds(trace)
    if bounds is None:
        return image

    min_x, max_x, min_y, max_y = bounds
    padding = 48.0
    plot_width = max(1.0, width - padding * 2.0)
    plot_height = max(1.0, height - padding * 2.0)
    world_width = max(max_x - min_x, 1e-6)
    world_height = max(max_y - min_y, 1e-6)
    scale = min(plot_width / world_width, plot_height / world_height)
    x_offset = padding + (plot_width - world_width * scale) * 0.5
    y_offset = padding + (plot_height - world_height * scale) * 0.5

    def to_canvas(point: Vector2) -> tuple[float, float]:
        return (
            x_offset + (point.x - min_x) * scale,
            y_offset + (max_y - point.y) * scale,
        )

    painter = QtGui.QPainter(image)
    try:
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        painter.setPen(QtGui.QPen(QtGui.QColor(216, 220, 227), 1))
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.drawRect(
            QtCore.QRectF(
                x_offset,
                y_offset,
                world_width * scale,
                world_height * scale,
            )
        )

        obstacle_pen = QtGui.QPen(QtGui.QColor(30, 30, 30), 2)
        painter.setPen(obstacle_pen)
        for obstacle in trace.obstacles:
            polygon = QtGui.QPolygonF(
                [QtCore.QPointF(*to_canvas(point)) for point in obstacle.points]
            )
            if obstacle.closed:
                painter.drawPolygon(polygon)
            else:
                painter.drawPolyline(polygon)

        trail_pen = QtGui.QPen(QtGui.QColor(70, 110, 180, 90), 2)
        painter.setPen(trail_pen)
        for agent_index in range(trace.num_agents):
            trail_points = [
                QtCore.QPointF(*to_canvas(frame.positions[agent_index]))
                for frame in trace.frames[: frame_index + 1]
            ]
            if len(trail_points) >= 2:
                painter.drawPolyline(QtGui.QPolygonF(trail_points))

        painter.setPen(QtGui.QPen(QtCore.Qt.PenStyle.NoPen))
        painter.setBrush(QtGui.QColor(160, 160, 160, 90))
        for position, radius in zip(trace.initial_positions, trace.agent_radii, strict=True):
            x, y = to_canvas(position)
            painter.drawEllipse(QtCore.QPointF(x, y), radius * scale, radius * scale)

        current_frame = trace.frames[frame_index]
        painter.setPen(QtGui.QPen(QtGui.QColor(20, 30, 60), 2))
        painter.setBrush(QtGui.QColor(70, 110, 180, 220))
        for position, radius in zip(
            current_frame.positions,
            trace.agent_radii,
            strict=True,
        ):
            x, y = to_canvas(position)
            painter.drawEllipse(QtCore.QPointF(x, y), radius * scale, radius * scale)

    finally:
        painter.end()

    return image


def _build_ffmpeg_export_command(
    *,
    frame_pattern: str,
    output_path: str,
    playback_fps: float,
) -> list[str]:
    extension = Path(output_path).suffix.lower()
    base_command = [
        "ffmpeg",
        "-y",
        "-framerate",
        f"{playback_fps:g}",
        "-i",
        frame_pattern,
    ]
    if extension == ".gif":
        return [
            *base_command,
            "-vf",
            "split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse",
            "-loop",
            "0",
            output_path,
        ]
    if extension == ".mp4":
        return [
            *base_command,
            "-vf",
            "scale=trunc(iw/2)*2:trunc(ih/2)*2",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            output_path,
        ]
    raise ValueError(f"unsupported animation format: {output_path}")


def _run_ffmpeg_export(command: list[str]) -> None:
    try:
        subprocess.run(command, check=True, capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            "Saving animations requires `ffmpeg` to be installed and available on PATH."
        ) from exc
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip()
        raise RuntimeError(
            f"ffmpeg failed while exporting animation: {stderr or exc}"
        ) from exc
