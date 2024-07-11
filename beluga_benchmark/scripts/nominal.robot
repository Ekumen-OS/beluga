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
Documentation       Nominal Beluga AMCL vs Nav2 AMCL benchmark using 2D datasets.

Resource            lambkin/shepherd/robot/resources/all.resource

Suite Setup         Setup Beluga vs Nav2 benchmark suite
Suite Teardown      Teardown Beluga vs Nav2 benchmark suite
Test Template       Run Beluga vs Nav2 benchmark case for each ${dataset} ${laser_model}


*** Variables ***
@{LASER_MODELS}        beam  likelihood_field


*** Test Cases ***        DATASET                 LASER_MODEL
Hallway Localization      hallway_localization    ${{LASER_MODELS}}
Hallway Return            hallway_return          ${{LASER_MODELS}}


*** Keywords ***
Beluga vs Nav2 benchmark suite
    Extends ROS 2 system benchmark suite
    Extends ROS 2 2D SLAM system benchmark suite
    Generates latexpdf report from nominal_report template in beluga_vs_nav2 ROS 2 package

Beluga vs Nav2 benchmark case
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
    Sets use_sim_time launch argument to true
    # Setup benchmark analysis
    Tracks /nav2_amcl/pose /beluga_amcl/pose trajectories
    Uses ${dataset_path}/groundtruth.tum as trajectory groundtruth
    Performs trajectory corrections  align=yes  t_max_diff=${0.1}

    Uses 10 iterations
