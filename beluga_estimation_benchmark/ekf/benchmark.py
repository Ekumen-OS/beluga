# Copyright 2026 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""EKF odometry benchmark.

Compares four odometry estimates over the same recorded run:

- ``raw_odom``: wheel odometry as recorded in the bag (baseline).
- ``rl_ekf``: robot_localization's ``ekf_node`` fusing odometry twist and IMU yaw rate.
- ``rl_ukf``: robot_localization's ``ukf_node`` fusing the same inputs.
- ``beluga_estimation_ekf``: Beluga's ``RecursiveFilter`` and ``ExtendedKalmanFilter``
  fusing the same inputs.
"""

import shutil

import matplotlib.pyplot as plt
import numpy as np

import lambkin

ESTIMATE_FILE = "estimate.tum"
REFERENCE_FILE = "reference.tum"

# Max. time difference (s) when pairing reference and estimate poses.
MAX_TIME_DIFF = "0.02"

# Max. gap (s) allowed in an estimate within the reference time span.
MAX_ESTIMATE_GAP = 0.1

# robot_localization launch file per variant.
RL_LAUNCH_FILES = {
    "rl_ekf": "robot_localization_ekf.launch.py",
    "rl_ukf": "robot_localization_ukf.launch.py",
}

# Metrics per estimate: (result file, evo command, extra arguments).
METRICS = {
    # Translational drift per 1 m travelled.
    "rpe_trans": ("output.rpe.zip", "evo_rpe", ["--delta", "1", "--delta_unit", "m"]),
    # Heading drift per 10 m travelled, over overlapping windows.
    "rpe_rot": (
        "output.rpe_rot.zip",
        "evo_rpe",
        [
            "--pose_relation",
            "angle_deg",
            "--delta",
            "10",
            "--delta_unit",
            "m",
            "--all_pairs",
        ],
    ),
}


def record_estimator(ctx, launch_file, topic):
    """Play the dataset through a launched estimator while recording its output topic."""
    # Messages are recorded with simulation time stamps.
    with lambkin.process.background(
        ctx.shell.ros2.bag.record,
        "--use-sim-time",
        "--output",
        "output",
        "--topics",
        topic,
    ):
        with lambkin.process.background(
            ctx.shell.ros2.launch, "beluga_estimation_ekf_benchmark", launch_file
        ):
            # The dataset publishes its own /clock.
            ctx.shell.ros2.bag.play(ctx.inputs.dataset, "-r", ctx.options.clock_rate)


def rotation_matrix(q):
    """Rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def align_origin(estimate, reference):
    """Return estimate positions moved so its pose at the reference start matches it."""
    start = estimate[np.argmin(np.abs(estimate[:, 0] - reference[0, 0]))]
    rotation = rotation_matrix(reference[0, 4:8]) @ rotation_matrix(start[4:8]).T
    return (estimate[:, 1:4] - start[1:4]) @ rotation.T + reference[0, 1:4]


def resample(estimate, stamps):
    """Return the estimate interpolated at the given stamps."""
    times = estimate[:, 0]
    xyz = [np.interp(stamps, times, estimate[:, k]) for k in (1, 2, 3)]
    # Keep consecutive quaternions in the same hemisphere before interpolating.
    quats = estimate[:, 4:8].copy()
    flips = np.sign(np.sum(quats[1:] * quats[:-1], axis=1))
    quats[1:] *= np.cumprod(flips)[:, None]
    # Linear interpolation is enough between poses milliseconds apart.
    q = np.column_stack([np.interp(stamps, times, quats[:, k]) for k in range(4)])
    q /= np.linalg.norm(q, axis=1)[:, None]
    return np.column_stack([stamps, *xyz, q])


def check_coverage(estimate, reference):
    """Fail if the estimate does not cover the whole reference time span without gaps."""
    stamps = estimate[:, 0]
    span = (stamps >= reference[0, 0] - MAX_ESTIMATE_GAP) & (
        stamps <= reference[-1, 0] + MAX_ESTIMATE_GAP
    )
    gaps = np.diff(stamps[span])
    if (
        stamps[0] > reference[0, 0]
        or stamps[-1] < reference[-1, 0]
        or (gaps.size and gaps.max() > MAX_ESTIMATE_GAP)
    ):
        raise RuntimeError(
            f"estimate [{stamps[0]:.3f}, {stamps[-1]:.3f}] does not cover reference "
            f"[{reference[0, 0]:.3f}, {reference[-1, 0]:.3f}] without gaps"
        )


@lambkin.benchmark(
    variants=lambkin.common.named_product(
        algorithm=["raw_odom", "rl_ekf", "rl_ukf", "beluga_estimation_ekf"]
    ),
    num_iterations=2,
)
@lambkin.option("--clock-rate", default=1.0)
@lambkin.option("--dataset", default="/data/datasets/input")
@lambkin.option("--ground-truth", default="/data/ground_truth/groundtruth.tum")
@lambkin.option("--warmup", default=1.0, help="Ground truth seconds to skip at start")
def nominal(ctx):
    """Run the EKF odometry benchmark."""
    lambkin.logger.info(
        "Running benchmark with variant: %s, iteration: %d", ctx.variant, ctx.iteration
    )

    # Working directory of ctx.shell commands.
    iter_dir = ctx.paths.iteration_dir
    estimate_path = iter_dir / ESTIMATE_FILE

    if ctx.variant.algorithm == "raw_odom":
        ctx.shell.evo_traj.bag2(ctx.inputs.dataset, "/odom", "--save_as_tum")
        shutil.move(iter_dir / "odom.tum", estimate_path)
    elif ctx.variant.algorithm in RL_LAUNCH_FILES:
        record_estimator(ctx, RL_LAUNCH_FILES[ctx.variant.algorithm], "/odom_filtered")
        ctx.shell.evo_traj.bag2("output", "/odom_filtered", "--save_as_tum")
        shutil.move(iter_dir / "odom_filtered.tum", estimate_path)
    elif ctx.variant.algorithm == "beluga_estimation_ekf":
        # Beluga publishes its estimate as the beluga_odom -> beluga_base_footprint transform.
        record_estimator(ctx, "beluga_estimation_ekf.launch.py", "/tf")
        # Sampling rate (Hz) of TF trajectories.
        ctx.shell.evo_config.set("tf_cache_lookup_frequency", "100")
        ctx.shell.evo_traj.bag2(
            "output", "/tf:beluga_odom.beluga_base_footprint", "--save_as_tum"
        )
        shutil.move(
            iter_dir / "tf_beluga_odom.beluga_base_footprint.tum", estimate_path
        )
    else:
        raise ValueError(f"Unknown algorithm variant: {ctx.variant.algorithm}")

    estimate = np.loadtxt(estimate_path)
    reference = np.loadtxt(ctx.inputs.reference)
    check_coverage(estimate, reference)
    # Same stamps as the reference, so output rate does not bias the comparison.
    np.savetxt(estimate_path, resample(estimate, reference[:, 0]), fmt="%.9f")

    for filename, command, args in METRICS.values():
        getattr(ctx.shell, command).tum(
            ctx.inputs.reference,
            ESTIMATE_FILE,
            "--t_max_diff",
            MAX_TIME_DIFF,
            *args,
            "--save_results",
            filename,
        )


@nominal.input
def dataset(ctx):
    """Return dataset path."""
    return ctx.options.dataset


@nominal.input
def reference(ctx):
    """Write the ground truth without its warm-up period, shared by all variants."""
    ground_truth = np.loadtxt(ctx.options.ground_truth)
    ground_truth = ground_truth[
        ground_truth[:, 0] >= ground_truth[0, 0] + ctx.options.warmup
    ]
    path = ctx.base_dir / REFERENCE_FILE
    np.savetxt(path, ground_truth, fmt="%.9f")
    return str(path)


def representative_iterations(ctx):
    """Return, per variant, the iteration with the median rotational RPE."""
    runs = {}
    for entry in lambkin.data.evo.stats(ctx, METRICS["rpe_rot"][0]):
        runs.setdefault(entry.params.algorithm, []).append(
            (entry.rmse, entry.iteration)
        )
    return {name: sorted(r)[(len(r) - 1) // 2][1] for name, r in runs.items()}


@nominal.output
def plots(ctx):
    """Plot origin-aligned trajectories and error timeseries, one iteration per variant."""
    reference = np.loadtxt(ctx.inputs.reference)
    fig, axes = plt.subplot_mosaic(
        [["traj", "rpe_trans"], ["traj", "rpe_rot"]], figsize=(14, 12)
    )
    ax_traj = axes["traj"]
    shown = representative_iterations(ctx)

    ax_traj.plot(
        reference[:, 1], reference[:, 2], "k--", label="ground truth", linewidth=1.5
    )
    for entry in lambkin.data.access.iterations(ctx):
        if entry.iteration != shown[entry.params.algorithm]:
            continue
        xyz = align_origin(np.loadtxt(entry.iter_dir / ESTIMATE_FILE), reference)
        ax_traj.plot(xyz[:, 0], xyz[:, 1], label=entry.params.algorithm)
    ax_traj.set(
        xlabel="x (m)", ylabel="y (m)", title="Trajectories (aligned at origin)"
    )
    ax_traj.axis("equal")

    panels = (
        ("rpe_trans", "Trans RPE (m)", "Translational RPE, delta = 1 m"),
        ("rpe_rot", "Rot RPE (deg)", "Rotational RPE, delta = 10 m"),
    )
    for metric, ylabel, title in panels:
        ax = axes[metric]
        for entry in lambkin.data.evo.series(ctx, METRICS[metric][0]):
            if entry.iteration != shown[entry.params.algorithm]:
                continue
            ax.plot(entry.time, entry.error, label=entry.params.algorithm)
        ax.set(xlabel="time (s)", ylabel=ylabel, title=title)

    for ax in axes.values():
        ax.legend()
        ax.grid(True)
    fig.tight_layout()
    fig.savefig(ctx.base_dir / "plots.png")


@nominal.output
def stats(ctx):
    """Log RMSE, mean, std, median, and max of every metric for every variant."""
    for metric, (filename, _, _) in METRICS.items():
        for entry in lambkin.data.evo.stats(ctx, filename):
            lambkin.logger.info(
                "%s %s iter %d: rmse=%.4f mean=%.4f std=%.4f median=%.4f max=%.4f",
                metric,
                entry.params.algorithm,
                entry.iteration,
                entry.rmse,
                entry.mean,
                entry.std,
                entry.median,
                entry.max,
            )


if __name__ == "__main__":
    nominal()
