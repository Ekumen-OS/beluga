#!/usr/bin/env -S shepherd robot -f

# Copyright 2022 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


*** Settings ***
Documentation       Beluga AMCL vs Nav2 AMCL benchmark over configuration space.

Resource            lambkin/shepherd/robot/resources/all.resource

Suite Setup         Setup Beluga vs Nav2 benchmark suite
Suite Teardown      Teardown Beluga vs Nav2 benchmark suite
Test Template       Run Beluga vs Nav2 benchmark case for each ${dataset} ${laser_model} ${num_particles} ${execution_policy}


*** Variables ***
@{NUM_PARTICLES}       250  300  400  500  750  1000  2000  3000  4000  5000
@{LASER_MODELS}        beam  likelihood_field
@{EXECUTION_POLICIES}  seq  par


*** Test Cases ***        DATASET                 LASER_MODEL         NUM_PARTICLES        EXECUTION_POLICY
Hallway Localization      hallway_localization    ${{LASER_MODELS}}   ${{NUM_PARTICLES}}   ${{EXECUTION_POLICIES}}
Hallway Return            hallway_return          ${{LASER_MODELS}}   ${{NUM_PARTICLES}}   ${{EXECUTION_POLICIES}}


*** Keywords ***
Beluga vs Nav2 benchmark suite
    Extends ROS 2 system benchmark suite
    Extends ROS 2 2D SLAM system benchmark suite
    Extends generic resource usage benchmark suite
    Generates latexpdf report from swept_report template in beluga_vs_nav2 ROS 2 package

Beluga vs Nav2 benchmark case
    Extends generic resource usage benchmark case
    Extends ROS 2 system benchmark case
    Extends ROS 2 2D SLAM system benchmark case
    # Setup benchmark inputs
    ${dataset_path} =  Set Variable  ${EXECDIR}/datasets/${dataset}
    Uses ${dataset_path}/ROS2/${dataset}_bag as input to ROS 2 system
    ${package_share_path} =  Find ROS 2 Package  beluga_vs_nav2  share=yes
    ${qos_override_path} =  Join Path  ${package_share_path}  config  qos_override.yml
    Configures QoS overrides from ${qos_override_path} for input to ROS 2 system
    # Setup benchmark rig
    Uses beluga_vs_nav2.launch in beluga_vs_nav2 ROS package as rig
    Sets map_filename launch argument to ${dataset_path}/map.yaml
    Sets laser_model_type launch argument to ${laser_model}
    Sets max_particles launch argument to ${num_particles}
    Sets min_particles launch argument to ${num_particles}
    Sets execution_policy launch argument to ${execution_policy}
    Sets use_sim_time launch argument to true
    # Setup benchmark profiling
    Uses timemory-timem to sample beluga_amcl performance
    Uses timemory-timem to sample nav2_amcl performance
    # Setup benchmark analysis
    Tracks /nav2_amcl/pose /beluga_amcl/pose trajectories
    Uses ${dataset_path}/groundtruth.tum as trajectory groundtruth
    Performs trajectory corrections  align=yes  t_max_diff=${0.1}

    Uses 1 iterations
