.. repl-quiet::

    import lambkin.shepherd.data as lks
    import pandas as pd
    import numpy as np
    import os

    os.makedirs('_generated', exist_ok=True)


Nominal Beluga AMCL vs Nav2 AMCL benchmark
==========================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Objective
---------

Compare `Beluga AMCL <https://github.com/Ekumen-OS/beluga>`_ and `Nav2 AMCL <https://github.com/ros-planning/navigation2/tree/main/nav2_amcl>`_ localization systems in terms of localization and computational performance for the nominal case.

Methodology
-----------

Dataset
^^^^^^^

For this report, `Magazino datasets <https://google-cartographer-ros.readthedocs.io/en/latest/data.html#magazino>`_, published with the Cartographer Public Data set under Apache License v2.0, were chosen. As these datasets are distributed in rosbag format, equivalent datasets in rosbag2 format were recreated. As both localization systems need a map to work with, and a groundtruth is necessary for performance evaluation, offline mapping was conducted using Cartographer ROS.

Configuration
^^^^^^^^^^^^^

For this report, the following baseline configuration:

.. datatemplate:import-module:: ament_index_python

    {% set package_path = data.get_package_share_directory('beluga_vs_nav2') %}

    .. literalinclude:: {{config.sysroot}}/{{ package_path }}/params/amcl.yaml
        :language: yaml

was modified for each benchmark case in terms of:

* the laser sensor model, to assess both beam and likelihood models (see sections 6.3 and 6.4 of Probabilistic Robotics, by Thrun et al);
* the execution policy, to compare single-threaded and multi-threaded performance. Note this feature is only provided by Beluga AMCL.

so as to have a reasonably complete picture of how both localization systems perform.

Platform
^^^^^^^^

.. datatemplate:import-module:: lambkin.clerk

    * Hardware
      {% set cpu_info = data.hardware.cpu_info() %}
        * CPU: {{ cpu_info.description }}
            {% for cache in cpu_info.caches %}
            * {{ cache }}
            {% endfor %}
        {% set memory_info = data.hardware.memory_info() %}
        * Memory: {{ '{:~P}'.format(memory_info.ram_size.to('MB')) }}
    * Software
      {% set os_distribution_info = data.os.distribution_info() %}
        * OS: {{ os_distribution_info.description  }}
        * ROS:
            {% set ros_distribution_info = data.ros2.distribution_info() %}
            * Distribution: {{ ros_distribution_info.name }}
            * Packages:
              {% for name in ('beluga_amcl', 'nav2_amcl') %}
              {% set pkg_info = data.ros2.package_info(name) %}
                * ``{{ pkg_info.name }}`` {{ pkg_info.version }}
              {% endfor %}

Metrics
^^^^^^^

To characterize the localization performance of both systems, this report uses:

* **APE**. The Absolute Pose Error is the difference between estimated and reference trajectories after alignment when taken as a whole. It is a measure of global accuracy and consistency.

Metrics are aggregated across multiple runs of each parameter variation to ensure statistical significance.

Results
-------

.. repl-quiet::

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'ape', normalization='long'),
        lks.evo.series('/nav2_amcl/pose', 'ape', normalization='long')
    ])

    data = data[[
        'variation.parameters.dataset',
        'variation.parameters.laser_model',
        'trajectory.name', 'metric.series.value'
    ]]

    def rms(x):
        return np.sqrt(np.mean(np.power(x, 2.)))

    data = data.groupby([
        'variation.parameters.dataset',
        'trajectory.name',
        'variation.parameters.laser_model',
    ])['metric.series.value']

    ape = data.agg([rms, 'mean', 'std', 'max']).stack().round(3)
    ape.to_pickle('_generated/ape.pkl')


.. datatemplate:import-module:: pandas

    {% set data = data.read_pickle('_generated/ape.pkl') %}
    {% set ape = data.loc['hallway_localization'] %}

    .. flat-table:: APE metrics for ``hallway_localization`` trajectories
       :header-rows: 2
       :name: hallway-localization-ape-comparison

       *
         - :rspan:`1`  :cspan:`1`  Implementation
         - :cspan:`3` Likelihood field sensor model
         - :cspan:`3` Beam sensor model

       *
         - rms
         - mean
         - stddev
         - max
         - rms
         - mean
         - stddev
         - max

       *
         - :cspan:`1` Beluga AMCL
         - {{ ape.loc['/beluga_amcl/pose', 'likelihood_field', 'rms'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'likelihood_field', 'mean'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'likelihood_field', 'std'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'likelihood_field', 'max'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'beam', 'rms'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'beam', 'mean'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'beam', 'std'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'beam', 'max'] }} m

       *
         - :cspan:`1` Nav2 AMCL
         - {{ ape.loc['/nav2_amcl/pose', 'likelihood_field', 'rms'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'likelihood_field', 'mean'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'likelihood_field', 'std'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'likelihood_field', 'max'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'beam', 'rms'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'beam', 'mean'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'beam', 'std'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'beam', 'max'] }} m


    {% set ape = data.loc['hallway_return'] %}

    .. flat-table:: APE metrics for ``hallway_return`` trajectories
       :header-rows: 2
       :name: hallway-return-ape-comparison

       *
         - :rspan:`1`  :cspan:`1`  Implementation
         - :cspan:`3` Likelihood field sensor model
         - :cspan:`3` Beam sensor model

       *
         - rms
         - mean
         - stddev
         - max
         - rms
         - mean
         - stddev
         - max

       *
         - :cspan:`1` Beluga AMCL
         - {{ ape.loc['/beluga_amcl/pose', 'likelihood_field', 'rms'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'likelihood_field', 'mean'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'likelihood_field', 'std'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'likelihood_field', 'max'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'beam', 'rms'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'beam', 'mean'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'beam', 'std'] }} m
         - {{ ape.loc['/beluga_amcl/pose', 'beam', 'max'] }} m

       *
         - :cspan:`1` Nav2 AMCL
         - {{ ape.loc['/nav2_amcl/pose', 'likelihood_field', 'rms'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'likelihood_field', 'mean'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'likelihood_field', 'std'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'likelihood_field', 'max'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'beam', 'rms'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'beam', 'mean'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'beam', 'std'] }} m
         - {{ ape.loc['/nav2_amcl/pose', 'beam', 'max'] }} m
