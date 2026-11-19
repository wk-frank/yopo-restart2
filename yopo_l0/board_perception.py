"""Exposure-timestamped D435 replay for a single wall-clock board."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict, dataclass
import math
from pathlib import Path

import numpy as np

from .d435_live import InterpolatedState, PoseRingBuffer
from .depth_accel import prepare_d435_dual_depth
from .depth_local import DepthLocalField
from .sensor import DepthFrame


@dataclass(frozen=True)
class ClockMapping:
    scale: float = 1.0
    offset_s: float = 0.0
    error_s: float = 0.0

    def __post_init__(self) -> None:
        if (not all(map(math.isfinite, (self.scale, self.offset_s, self.error_s)))
                or self.scale <= 0 or self.error_s < 0):
            raise ValueError("invalid camera clock mapping")

    def to_monotonic(self, sensor_stamp_s: float) -> float:
        return self.scale * sensor_stamp_s + self.offset_s


@dataclass(frozen=True)
class DepthExposure:
    camera_serial: str
    frame_id: int
    sensor_stamp_s: float
    depth_m: np.ndarray
    intrinsics: tuple[float, float, float, float]
    verified_far_free: np.ndarray | None = None
    far_free_source: str | None = None
    valid_col_span: tuple[int, int] | None = None

    @classmethod
    def from_archive(cls, path: Path, evidence: dict) -> DepthExposure:
        """Read a frame using its NPZ and matching depth-observation log entry."""
        with np.load(path, allow_pickle=False) as raw:
            depth = raw["depth_m"]
            mask = raw["verified_far_free"]
        far_source = evidence["far_free_source"]
        span = evidence["valid_col_span"]
        return cls(
            evidence["source_serial"], evidence["frame_id"], evidence["sensor_stamp_s"],
            depth, tuple(evidence["intrinsics"]), mask if far_source is not None else None,
            far_source, tuple(span) if span is not None else None,
        )


@dataclass(frozen=True)
class PerceptionFrame:
    frame_id: int
    exposure_s: float
    pose: InterpolatedState
    network_depth: np.ndarray
    safety_depth_m: np.ndarray
    generation: int


@dataclass(frozen=True)
class GeometryDecision:
    allowed: bool
    reason: str
    frame_id: int | None
    failed_center_enu: tuple[float, float, float] | None = None
    coverage_exposures_s: tuple[float, ...] = ()


def _proper_rotation(rotation: np.ndarray) -> bool:
    return (rotation.shape == (3, 3) and np.isfinite(rotation).all()
            and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
            and np.linalg.det(rotation) > 0)


def _pose_evidence(pose: InterpolatedState) -> dict:
    return dict(stamp_s=pose.stamp_s, position_enu=pose.position_enu.tolist(),
                velocity_enu=pose.velocity_enu.tolist(), acceleration_enu=pose.acceleration_enu.tolist(),
                rotation_enu_flu=pose.rotation_enu_flu.tolist())


class BoardPerception:
    def __init__(
        self,
        camera_serial: str,
        *,
        mount_xyz_body=(0., 0., 0.),
        camera_clock: ClockMapping = ClockMapping(),
        camera_rotation=None,
        body_radius_m: float = 0.35,
        min_range_m: float = 0.20,
        max_range_m: float = 20.0,
        max_clock_error_s: float = 0.025,
        max_frame_age_s: float = 0.20,
        max_odometry_age_s: float = 0.25,
        history_age_s: float = 0.75,
        sample_step_m: float = 0.10,
        geometry_margin_m: float = 0.02,
        log: Callable[..., None] | None = None,
        archive_dir: Path | None = None,
    ) -> None:
        self.camera_serial = camera_serial
        self.camera_clock = camera_clock
        self._clock_synchronized = True
        self.mount_xyz_body = np.asarray(mount_xyz_body, dtype=float).copy()
        self.camera_rotation = np.eye(3) if camera_rotation is None else np.asarray(camera_rotation, dtype=float).copy()
        limits = (body_radius_m, min_range_m, max_range_m, max_clock_error_s,
                  max_frame_age_s, max_odometry_age_s, history_age_s, sample_step_m)
        if (not camera_serial or self.mount_xyz_body.shape != (3,) or not np.isfinite(self.mount_xyz_body).all()
                or not _proper_rotation(self.camera_rotation)
                or not all(math.isfinite(x) and x > 0 for x in limits) or max_range_m <= min_range_m + 1.0
                or not math.isfinite(geometry_margin_m) or geometry_margin_m < 0):
            raise ValueError("invalid camera binding or geometry limits")
        self.body_radius_m = body_radius_m
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.max_clock_error_s = max_clock_error_s
        self.max_frame_age_s = max_frame_age_s
        self.max_odometry_age_s = max_odometry_age_s
        self.history_age_s = history_age_s
        self.sample_step_m = sample_step_m
        self.geometry_margin_m = geometry_margin_m
        self.log = log
        self.archive_dir = Path(archive_dir) if archive_dir is not None else None
        if self.archive_dir is not None:
            self.archive_dir.mkdir(exist_ok=True, parents=True)
        self._poses = PoseRingBuffer(max_time_gap_s=max_odometry_age_s)
        self._last_sensor_stamp_s: float | None = None
        self._last_frame_id: int | None = None
        self._coordinate_version: int | None = None
        self.latest_frame: PerceptionFrame | None = None
        self.generation = 0
        self.last_reason = "NO_DEPTH"
        self._reset_field()

    def _reset_field(self) -> None:
        self._field = DepthLocalField(
            self.mount_xyz_body, self.max_range_m, strict_unknown=True,
            swept_radius_m=self.body_radius_m, cam_rotation=self.camera_rotation,
            history_frames=math.ceil(15 * min(2.0, self.history_age_s)) + 2,
        )

    def _event(self, event: str, **fields) -> None:
        if self.log is not None:
            self.log(event, camera_serial=self.camera_serial, generation=self.generation, **fields)

    def _invalidate(self, reason: str, *, clear_history: bool = True) -> None:
        self.generation += 1
        self.last_reason = reason
        self.latest_frame = None
        if clear_history:
            self._reset_field()

    def synchronize_camera_clock(self, mapping: ClockMapping) -> None:
        """Install a freshly calibrated map after a camera-clock discontinuity."""
        if mapping.error_s > self.max_clock_error_s:
            raise ValueError("camera clock mapping exceeds the configured error budget")
        self.camera_clock = mapping
        self._clock_synchronized = True
        self._last_sensor_stamp_s = None
        self._last_frame_id = None
        self._invalidate("CAMERA_CLOCK_REMAPPED")
        self._event("camera_clock_remapped", camera_clock=asdict(mapping))

    def push_odometry(
        self, stamp_s: float, position_enu, velocity_enu, rotation_enu_flu, acceleration_enu=None,
        *, coordinate_version: int = 1,
    ) -> bool:
        rotation = np.asarray(rotation_enu_flu, dtype=float)
        if self._coordinate_version is None:
            self._coordinate_version = coordinate_version
        if coordinate_version != self._coordinate_version:
            self._coordinate_version = coordinate_version
            reason = "COORDINATE_VERSION_CHANGED"
        elif self._poses.latest is not None and stamp_s < self._poses.latest.stamp_s:
            reason = "ODOMETRY_CLOCK_REGRESSION"
        elif not _proper_rotation(rotation):
            reason = "INVALID_ODOMETRY_ROTATION"
        elif self._poses.push(stamp_s, position_enu, velocity_enu, rotation, acceleration_enu):
            return True
        else:
            reason = self._poses.invalid_reason
        self._poses.reset()
        self._invalidate(reason)
        self._event("odometry_invalidated", reason=reason, coordinate_version=coordinate_version,
                    stamp_s=stamp_s if math.isfinite(stamp_s) else None)
        return False

    def tracking_state(self, now_s: float) -> InterpolatedState | None:
        latest = self._poses.latest
        if latest is None or not 0.0 <= now_s - latest.stamp_s <= self.max_odometry_age_s:
            return None
        return latest

    def _reject(self, frame: DepthExposure, reason: str, *, clear_history: bool = True, **details) -> None:
        self._invalidate(reason, clear_history=clear_history)
        stamp = frame.sensor_stamp_s if math.isfinite(frame.sensor_stamp_s) else None
        frame_id = frame.frame_id if type(frame.frame_id) is int else None
        self._event("depth_invalidated", frame_id=frame_id, source_serial=frame.camera_serial,
                    sensor_stamp_s=stamp, camera_clock=asdict(self.camera_clock), reason=reason, **details)

    def observe(self, frame: DepthExposure, *, now_s: float) -> PerceptionFrame | None:
        """Consume a single exposure for both inference and strict observed-space geometry."""
        if type(frame.frame_id) is not int or frame.frame_id < 0:
            self._reject(frame, "INVALID_FRAME_ID")
            return None
        if frame.camera_serial != self.camera_serial:
            self._reject(frame, "CAMERA_SERIAL_MISMATCH")
            return None
        if not self._clock_synchronized:
            self._reject(frame, "CAMERA_CLOCK_UNSYNCHRONIZED")
            return None
        if (not math.isfinite(frame.sensor_stamp_s) or not math.isfinite(now_s)
                or self.camera_clock.error_s > self.max_clock_error_s):
            self._reject(frame, "INVALID_CAMERA_CLOCK")
            return None
        if self._last_sensor_stamp_s is not None and frame.sensor_stamp_s < self._last_sensor_stamp_s:
            self._clock_synchronized = False
            self._reject(frame, "CAMERA_CLOCK_REGRESSION")
            return None
        if (self._last_frame_id is not None and frame.frame_id <= self._last_frame_id
                or frame.sensor_stamp_s == self._last_sensor_stamp_s):
            self._reject(frame, "FRAME_OUT_OF_ORDER")
            return None

        self._last_frame_id = frame.frame_id
        self._last_sensor_stamp_s = frame.sensor_stamp_s
        exposure_s = self.camera_clock.to_monotonic(frame.sensor_stamp_s)
        error_s = self.camera_clock.error_s
        if not math.isfinite(exposure_s) or not error_s <= now_s - exposure_s <= self.max_frame_age_s - error_s:
            self._reject(frame, "STALE_OR_FUTURE_EXPOSURE")
            return None
        bracket = self._poses.bracketed_exposure(exposure_s, error_s=error_s)
        if bracket is None or bracket[2].stamp_s > now_s:
            self._reject(frame, "EXPOSURE_POSE_BRACKET_MISSING", exposure_s=exposure_s)
            return None
        pose, before, after = bracket

        raw = np.asarray(frame.depth_m, dtype=np.float32)
        K = tuple(frame.intrinsics)
        if (raw.shape != (96, 160) or len(K) != 4 or not all(map(math.isfinite, K))
                or min(K[:2]) <= 0):
            self._reject(frame, "INVALID_DEPTH_IMAGE")
            return None
        measured = np.isfinite(raw) & (raw > self.min_range_m) & (raw < self.max_range_m)
        mask = np.zeros(raw.shape, dtype=bool)
        if frame.verified_far_free is not None:
            mask = np.asarray(frame.verified_far_free)
            if mask.dtype != np.dtype(bool) or mask.shape != raw.shape or not frame.far_free_source:
                self._reject(frame, "INVALID_FAR_FREE_PROVENANCE")
                return None
        verified = mask & np.isposinf(raw)
        sanitized = np.full(raw.shape, np.nan, dtype=np.float32)
        sanitized[measured] = raw[measured]
        sanitized[verified] = np.inf
        try:
            network, safety = prepare_d435_dual_depth(sanitized, valid_col_span=frame.valid_col_span)
        except ValueError:
            self._reject(frame, "INVALID_DEPTH_FIELD_OF_VIEW")
            return None
        metadata = dict(
            frame_id=frame.frame_id, source_serial=frame.camera_serial, sensor_stamp_s=frame.sensor_stamp_s,
            exposure_s=exposure_s, received_s=now_s, camera_clock=asdict(self.camera_clock),
            pose_before=_pose_evidence(before), pose_after=_pose_evidence(after),
            exposure_pose=_pose_evidence(pose), intrinsics=K, valid_col_span=frame.valid_col_span,
            camera_origin_enu=(pose.position_enu + pose.rotation_enu_flu @ self.mount_xyz_body).tolist(),
            mount_xyz_body=self.mount_xyz_body.tolist(), camera_rotation=self.camera_rotation.tolist(),
            body_radius_m=self.body_radius_m, far_free_source=frame.far_free_source,
            coordinate_version=self._coordinate_version, measured_surface_count=int(np.isfinite(safety).sum()),
            far_free_count=int(np.isposinf(safety).sum()), archive=self._archive(frame, raw, mask, now_s),
        )
        if not (np.isfinite(safety).any() or np.isposinf(safety).any()):
            self._invalidate("ALL_BLIND", clear_history=False)
            self._event("depth_invalidated", reason=self.last_reason, **metadata)
            return None
        safety.setflags(write=False)
        network.setflags(write=False)
        self._field.observe(DepthFrame(exposure_s, safety, K), pose.position_enu, pose.rotation_enu_flu)
        processed = PerceptionFrame(frame.frame_id, exposure_s, pose, network, safety, self.generation)
        self.latest_frame = processed
        self.last_reason = "DEPTH_READY"
        self._event("depth_observed", **metadata)
        return processed

    def check_first_segment(self, points_enu, *, now_s: float) -> GeometryDecision:
        """Check a body-centred continuous polyline sweep from current measured odometry."""
        latest = self.latest_frame
        frame_id = latest.frame_id if latest is not None else self._last_frame_id
        track = self.tracking_state(now_s)
        points = np.asarray(points_enu, dtype=float)
        check_details = {}
        if latest is None:
            decision = GeometryDecision(False, self.last_reason, frame_id)
        elif not 0 <= now_s - latest.exposure_s <= self.max_frame_age_s - self.camera_clock.error_s:
            self._invalidate("STALE_DEPTH")
            decision = GeometryDecision(False, self.last_reason, frame_id)
        elif track is None:
            self._invalidate("STALE_ODOMETRY")
            decision = GeometryDecision(False, self.last_reason, frame_id)
        elif (points.ndim != 2 or points.shape[1:] != (3,) or len(points) == 0 or not np.isfinite(points).all()
              or np.linalg.norm(points[0] - track.position_enu) > 1e-6):
            decision = GeometryDecision(False, "INVALID_SWEEP_ORIGIN", frame_id)
        else:
            check_details["path_enu"] = points.tolist()
            check_details["tracking_pose"] = _pose_evidence(track)
            decision = self._verify_sweep(latest, points, now_s, check_details)

        self._event("depth_geometry_decision", checked_s=now_s if math.isfinite(now_s) else None,
                    body_radius_m=self.body_radius_m, geometry_margin_m=self.geometry_margin_m,
                    sample_step_m=self.sample_step_m, history_age_s=self.history_age_s,
                    **asdict(decision), **check_details)
        return decision

    def _verify_sweep(
        self, latest: PerceptionFrame, points: np.ndarray, now_s: float, details: dict,
    ) -> GeometryDecision:
        vectors = np.diff(points, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        counts = np.ceil(lengths / self.sample_step_m)
        if not np.isfinite(counts).all() or counts.sum() > 1024:
            return GeometryDecision(False, "SWEEP_SAMPLE_BUDGET", latest.frame_id)
        if not counts.any():
            return GeometryDecision(True, "BODY_OCCUPIED", latest.frame_id)

        centers, radii = [], []
        for start, delta, distance, count in zip(points[:-1], vectors, lengths, counts.astype(int)):
            if count:
                fractions = (np.arange(count) + 0.5) / count
                centers.extend(start + fractions[:, None] * delta)
                radii.extend(np.full(count, self.body_radius_m + self.geometry_margin_m + distance / (2 * count)))
        centers, radii = np.asarray(centers), np.asarray(radii)
        coverage = self._field.measured_sphere_coverage(
            centers, radii, min_range_m=self.min_range_m,
            since_s=now_s - self.history_age_s + self.camera_clock.error_s,
        )
        collisions = self._field.intersects_observed_surface(centers, radii)
        details["sweep_centers_enu"] = centers.tolist()
        details["sweep_radii_m"] = radii.tolist()
        details["coverage_by_center_s"] = [float(stamp) if math.isfinite(stamp) else None for stamp in coverage]
        blocked = np.flatnonzero(np.isnan(coverage) | collisions)
        exposures = tuple(sorted(set(coverage[np.isfinite(coverage)])))
        if blocked.size:
            i = int(blocked[0])
            pose = latest.pose
            camera_origin = pose.position_enu + pose.rotation_enu_flu @ self.mount_xyz_body
            optical_axis = (pose.rotation_enu_flu @ self.camera_rotation)[:, 0]
            forward = float((centers[i] - camera_origin) @ optical_axis)
            reason = "UNKNOWN_SWEPT_VOLUME"
            if collisions[i]:
                reason = "OBSERVED_SURFACE_IN_SWEEP"
            elif forward - radii[i] <= self.min_range_m:
                reason = "NEAR_BLIND_BAND"
            return GeometryDecision(False, reason, latest.frame_id, tuple(centers[i]), exposures)
        return GeometryDecision(True, "OBSERVED_SWEEP_CLEAR", latest.frame_id, coverage_exposures_s=exposures)

    def _archive(self, frame: DepthExposure, raw: np.ndarray, verified: np.ndarray, now_s: float) -> str | None:
        if self.archive_dir is None:
            return None
        path = self.archive_dir / f"depth-{round(now_s * 1e9)}-{frame.frame_id}.npz"
        with path.open("xb") as output:
            np.savez_compressed(output, depth_m=raw, verified_far_free=verified)
        return path.name
