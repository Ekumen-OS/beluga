.. repl-quiet::

    import lambkin.shepherd.data as lks
    import pandas as pd
    import seaborn as sns
    import numpy as np
    from matplotlib import pyplot as plt
    sns.set_theme(style='darkgrid')


Nav2 NDT sensor model benchmarks
===================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Objective
---------

Compare `Beluga AMCL <https://github.com/Ekumen-OS/beluga>`_  `NDT` sensor model against a groundtruth signal for magazino datasets.

Methodology
-----------

Dataset
^^^^^^^

For this report, `Magazino datasets <https://google-cartographer-ros.readthedocs.io/en/latest/data.html#magazino>`_, published with the Cartographer Public Data set under Apache License v2.0, were chosen. As these datasets are distributed in rosbag format, equivalent datasets in rosbag2 format were recreated. As both localization systems need a map to work with, and a groundtruth is necessary for performance evaluation, offline mapping was conducted using Cartographer ROS.

NDT map
^^^^^^^

Maps where built by extracting aggregated raw pointcloud data, leveraging `Cartographer's assets writer <https://google-cartographer-ros.readthedocs.io/en/stable/assets_writer.html#assets-writer>`_ and then converted using `beluga_ros's` `PLY to NDT conversion <https://github.com/Ekumen-OS/beluga/commit/c062cce51ee3188b13a2ab1cfc2da9c16bb2c39c#diff-17c3b7d22c479af6e486f0cfbe57e2991ce174d6bf54bd07bcbc8ba891f9909c>`_ with a 30cm cell size.

Configuration
^^^^^^^^^^^^^

For this report, the following baseline configuration:

.. datatemplate:import-module:: ament_index_python

    {% set package_path = data.get_package_share_directory('beluga_vs_nav2') %}

    .. literalinclude:: {{config.sysroot}}/{{ package_path }}/params/ndt_mcl.yaml
        :language: yaml

was modified for each benchmark case in terms of:

.. datatemplate:import-module:: lambkin.shepherd.data

    {% set suite = data.access.suite() %}

    * the number of particles, chosen from the {{ "{" }}{{ ', '.join(suite.metadata['variables']['NUM_PARTICLES']) }}{{ "}" }} set;
    * the execution policy, to compare single-threaded and multi-threaded performance. Note this feature is only provided by Beluga AMCL.

so as to have a reasonably complete picture of how the localization systems performs.

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

To characterize the localization performance of the system, this report uses:

* **APE**. The Absolute Pose Error is the difference between estimated and reference trajectories after alignment when taken as a whole. It is a measure of global accuracy and consistency.
* **RPE**. The Relative Pose Error is the difference between estimated and reference trajectories when compared in fixed time intervals. It is a measure of local accuracy and consistency.

To characterize the computational performance of both systems, this report uses:

* **CPU usage**. Usage is defined as the ratio between the time spent by an OS process, in both user and kernel space, and the total time elapsed.
* **RSS**. The Resident Set Size is the amount of physical memory allocated and held by an OS process.

These metrics are aggregated across multiple runs of each parameter variation to ensure statistical significance.

Results
-------

Nominal setup
^^^^^^^^^^^^^

In the following, for a nominal setup this report assumes :math:`N = 2000` particles (fixed), and a sequential (i.e. single-threaded) execution policy.

.. repl-quiet::

    nominal_case = list(lks.access.iterations([
        location for location in lks.access.variations() if
        location.metadata['variation']['parameters']['num_particles'] == '2000' and
        location.metadata['variation']['parameters']['execution_policy'] == 'seq'
    ]))


.. repl-quiet::

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'ape', target_iterations=nominal_case, normalization='long'),
        lks.evo.series('/beluga_amcl/pose', 'rpe', target_iterations=nominal_case, normalization='long')
    ])

    data = data.replace({'metric.name': {'ape': 'APE', 'rpe': 'RPE'}})

    grid = sns.displot(
        data=data, x='metric.series.value', hue='trajectory.name',
        row='metric.name', col='variation.parameters.dataset',
        kind='kde', facet_kws=dict(sharex='row')
    )
    grid.figure.suptitle('Localization error distribution')
    grid.set_titles('{row_name} @ {col_name}')
    grid.set_axis_labels('Error (m)')
    grid._legend.set_title('Trajectory')
    grid.tight_layout()

    plt.show()


.. repl-quiet::

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'ape', target_iterations=nominal_case, normalization='long')
    ])

    grid = sns.relplot(
        data=data,
        x='metric.series.time',
        y='metric.series.value',
        hue='trajectory.name',
        col='variation.parameters.dataset',
        kind='line', n_boot=20,
        facet_kws=dict(sharex='col')
    )
    grid.figure.suptitle('Absolute Pose Error (APE) over time')
    grid.set_axis_labels('Time (s)', 'APE (m)')
    grid.set_titles('{col_name} dataset')
    grid._legend.set_title('Trajectory')
    grid.set(ylim=(0, None))
    grid.tight_layout()

    plt.show()


.. repl-quiet::

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'rpe', target_iterations=nominal_case, normalization='long')
    ])

    grid = sns.relplot(
        data=data,
        x='metric.series.time',
        y='metric.series.value',
        hue='trajectory.name',
        col='variation.parameters.dataset',
        kind='line', n_boot=20,
        facet_kws=dict(sharex='col')
    )
    grid.figure.suptitle('Relative Pose Error (RPE) over time')
    grid.set_axis_labels('Time (s)', 'RPE (m)')
    grid.set_titles('{col_name} dataset')
    grid._legend.set_title('Trajectory')
    grid.set(ylim=(0, None))
    grid.tight_layout()

    plt.show()


.. repl-quiet::

    data = pd.concat([
        lks.evo.trajectory(
            'groundtruth', target_iterations=nominal_case,
            trajectory_name_format='path', trajectory_file_format='tum'),
        lks.evo.trajectory('/beluga_amcl/pose', target_iterations=nominal_case)
    ])

    grid = sns.relplot(
      data=data,
      x='trajectory.x', y='trajectory.y',
      col='variation.parameters.dataset',
      hue='trajectory.name',
      kind='line', n_boot=20,
    )
    grid.figure.suptitle('Localization performance w.r.t. map')
    grid.set_axis_labels('x (m)', 'y (m)')
    grid.set_titles('{col_name} dataset')
    grid._legend.set_title('Trajectory')
    grid.tight_layout()

    plt.show()


.. repl-quiet::

    data = pd.concat([
        lks.evo.trajectory(
            'groundtruth', target_iterations=nominal_case,
            trajectory_name_format='path', trajectory_file_format='tum'),
        lks.evo.trajectory('/beluga_amcl/pose', target_iterations=nominal_case)
    ])

    data['trajectory.yaw'] = np.degrees(data['trajectory.yaw'])

    data = pd.melt(
        data,
        value_vars=['trajectory.x', 'trajectory.y', 'trajectory.yaw'],
        id_vars=['trajectory.time', 'variation.parameters.dataset', 'trajectory.name'],
        var_name='trajectory.coordinate.name', value_name='trajectory.coordinate.value'
    )

    data = data.replace({'trajectory.coordinate.name': {
        'trajectory.x': 'x (m)', 'trajectory.y': 'y (m)', 'trajectory.yaw': 'yaw (deg)'
    }})

    grid = sns.relplot(
        data=data, x='trajectory.time', y='trajectory.coordinate.value',
        row='trajectory.coordinate.name', col='variation.parameters.dataset',
        hue='trajectory.name', facet_kws=dict(sharex='col', sharey='row'),
        kind='line', n_boot=20
    )
    grid.figure.suptitle('Localization performance w.r.t. time')
    grid.set_axis_labels('Time (s)', '')
    grid.set_titles('{row_name}|{col_name} dataset')
    grid._legend.set_title('Trajectory')

    for i, row in enumerate(grid.axes):
        for axis in row:
            title = axis.title.get_text()
            row_name, col_name = title.split('|')
            axis.set_title(col_name if i == 0 else '')
        row[0].set_ylabel(row_name)

    grid.tight_layout()

    plt.show()


Setup sweep
^^^^^^^^^^^

In the following, this report sweeps over parameterizations to compare the resulting performance. Error bars represent bootstrapped 95% confidence intervals.

.. repl-quiet::

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'ape', normalization='long')
    ])

    data['trajectory.qualified_name'] = data.apply(
        lambda row: '{} ({})'.format(
            row['trajectory.name'],
            row['variation.parameters.execution_policy']
            if 'beluga' in row['trajectory.name'] else 'seq'
        ), axis=1
    )

    data['variation.parameters.num_particles'] = pd.to_numeric(
        data['variation.parameters.num_particles']
    )

    data = data.sort_values([
        'variation.parameters.dataset',
        'trajectory.qualified_name',
        'variation.parameters.num_particles'
    ])

    grid = sns.relplot(
        data=data, y='metric.series.value',
        x='variation.parameters.num_particles',
        hue='trajectory.qualified_name',
        row='variation.parameters.dataset',
        marker='o', kind='line', n_boot=20,
        height=4, aspect=2.5, err_style='bars',
        facet_kws=dict(sharey=False)
    )
    xticks = np.sort(np.unique(data['variation.parameters.num_particles']))
    grid.set(xticks=xticks, xticklabels=list(map(str, xticks)))
    grid.figure.suptitle('Absolute Pose Error (APE) with particle filter size')
    grid.set_axis_labels('Particle count', 'Error (m)')
    grid.set_titles('{row_name} dataset')
    grid._legend.set_title('Trajectory')
    grid.set(ylim=(0, None))
    grid.tight_layout()

    plt.show()


.. repl-quiet::

    data = pd.concat([
        lks.evo.series('/beluga_amcl/pose', 'rpe', normalization='long')
    ])

    data['trajectory.qualified_name'] = data.apply(
        lambda row: '{} ({}, {})'.format(
            row['trajectory.name'],
            row['variation.parameters.execution_policy']
            if 'beluga' in row['trajectory.name'] else 'seq'
        ), axis=1
    )

    data['variation.parameters.num_particles'] = pd.to_numeric(
        data['variation.parameters.num_particles']
    )

    data = data.sort_values([
        'variation.parameters.dataset'
        'trajectory.qualified_name',
        'variation.parameters.num_particles'
    ])

    grid = sns.relplot(
        data=data, y='metric.series.value',
        x='variation.parameters.num_particles',
        hue='trajectory.qualified_name',
        row='variation.parameters.dataset',
        marker='o', kind='line', n_boot=20,
        height=4, aspect=2.5, err_style='bars',
        facet_kws=dict(sharey=False)
    )
    xticks = np.sort(np.unique(data['variation.parameters.num_particles']))
    grid.set(xticks=xticks, xticklabels=list(map(str, xticks)))
    grid.figure.suptitle('Relative Pose Error (RPE) with particle filter size')
    grid.set_axis_labels('Particle count', 'Error (m)')
    grid.set_titles('{row_name} dataset')
    grid._legend.set_title('Trajectory')
    grid.set(ylim=(0, None))
    grid.tight_layout()

    plt.show()


.. repl-quiet::

    data = pd.concat([
        lks.timem.summary('beluga_amcl', 'cpu_util', normalization='long')
    ])

    data = lks.pandas.rescale(data, {'process.summary.cpu_util': 100})
    data['process.qualified_name'] = data.apply(
        lambda row: '{} ({})'.format(
            row['process.name'],
            row['variation.parameters.execution_policy']
            if 'beluga' in row['process.name'] else 'seq'
        ), axis=1
    )

    data['variation.parameters.num_particles'] = pd.to_numeric(
        data['variation.parameters.num_particles']
    )

    data = data.sort_values([
        'variation.parameters.dataset',
        'process.qualified_name',
        'variation.parameters.num_particles'
    ])

    grid = sns.relplot(
        data=data, y='process.summary.cpu_util',
        x='variation.parameters.num_particles',
        hue='process.qualified_name',
        row='variation.parameters.dataset',
        marker='o', kind='line', n_boot=20,
        height=4, aspect=2.5, err_style='bars',
        facet_kws=dict(sharey=False)
    )
    xticks = np.sort(np.unique(data['variation.parameters.num_particles']))
    grid.set(xticks=xticks, xticklabels=list(map(str, xticks)))
    grid.figure.suptitle('CPU usage with particle filter size')
    grid.set_axis_labels('Particle count', 'Usage (%)')
    grid.set_titles('{row_name} dataset')
    grid._legend.set_title('Process')
    grid.set(ylim=(0, None))
    grid.tight_layout()

    plt.show()


.. repl-quiet::

    data = pd.concat([
        lks.timem.summary('beluga_amcl', 'peak_rss', normalization='long')
    ])

    data = lks.pandas.rescale(data, {'process.summary.peak_rss': 1 / 8e6})
    data['process.qualified_name'] = data.apply(
        lambda row: '{} ({})'.format(
            row['process.name'],
            row['variation.parameters.execution_policy']
            if 'beluga' in row['process.name'] else 'seq'
        ), axis=1
    )

    data['variation.parameters.num_particles'] = pd.to_numeric(
        data['variation.parameters.num_particles']
    )

    data = data.sort_values([
        'variation.parameters.dataset',
        'process.qualified_name',
        'variation.parameters.num_particles'
    ])

    grid = sns.relplot(
        data=data, y='process.summary.peak_rss',
        x='variation.parameters.num_particles',
        hue='process.qualified_name',
        row='variation.parameters.dataset',
        kind='line', marker='o', n_boot=20,
        height=4, aspect=2.5, err_style='bars',
        facet_kws=dict(sharey=False)
    )
    xticks = np.sort(np.unique(data['variation.parameters.num_particles']))
    grid.set_xticks(xticks, labels=list(map(str, xticks)))
    grid.figure.suptitle('Peak RSS with particle filter size')
    grid.set_axis_labels('Particle count', 'Memory (MB)')
    grid.set_titles('{row_name} dataset')
    grid._legend.set_title('Process')
    grid.set(ylim=(0, None))
    grid.tight_layout()

    plt.show()
