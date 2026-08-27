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
"""Nominal benchmark for the Beluga AMCL localization system."""

import matplotlib.pyplot as plt

import lambkin


@lambkin.benchmark(
    variants=lambkin.common.named_product(
        sensor_model=["beam", "likelihood_field"], num_particles=[10, 20, 100]
    ),
    num_iterations=2,
)
@lambkin.option("--clock-rate", default=1.0)
@lambkin.option("--sensor-topic", default="/scan")
def nominal(ctx):
    """Run a nominal Beluga AMCL benchmark across sensor models and particle counts.

    Launches the Beluga localization stack, plays back a ROS 2 bag,
    and evaluates trajectory accuracy with evo_ape.

    Args:
        ctx: Lambkin context with variant, options, source, and shell access.
    """
    lambkin.logger.info(
        "Running benchmark with variant: %s, iteration: %d", ctx.variant, ctx.iteration
    )

    with lambkin.process.background(
        ctx.shell.ros2.bag.record,
        "--output",
        "output",
        "-a",
    ):
        with lambkin.process.background(
            ctx.shell.ros2.launch,
            "beluga_ros2",
            "beluga.launch.py",
            f"sensor_model:={ctx.variant.sensor_model}",
            f"num_particles:={ctx.variant.num_particles}",
            f"map_path:={ctx.inputs.map}",
        ):
            ctx.shell.ros2.bag.play(
                ctx.inputs.dataset, "--clock", "-r", ctx.options.clock_rate
            )
    ctx.shell.evo_ape.bag2(
        "output",
        "/ground_truth",
        "/pose",
        "--t_max_diff",
        "0.5",
        "--save_results",
        "output.ape.zip",
    )


@nominal.input
def dataset(ctx):
    """Return the path to the MCAP dataset used as input for the benchmark."""
    return "/data/datasets/input"


@nominal.input
def map(ctx):
    """Return the path to the map file used for localization."""
    return "/data/maps/map.yaml"


@nominal.output
def plots(ctx):
    """Save APE timeseries plot for all iterations."""
    for entry in lambkin.data.evo.series(ctx, "output.ape.zip"):
        plt.plot(
            entry.time, entry.error, label=f"{entry.variant} / iter {entry.iteration}"
        )
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.legend()
    plt.savefig(ctx.base_dir / "plots.png")


@nominal.output
def stats(ctx):
    """Log RMSE, mean, and max APE for all iterations."""
    for entry in lambkin.data.evo.stats(ctx, "output.ape.zip"):
        lambkin.logger.info(
            "%s iter %d: rmse=%.4f mean=%.4f max=%.4f",
            entry.variant,
            entry.iteration,
            entry.rmse,
            entry.mean,
            entry.max,
        )


if __name__ == "__main__":
    nominal()
