// Copyright 2022-2024 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// https://github.com/Ekumen-OS/beluga/issues/279#issuecomment-1903914387

#include <fstream>
#include <iostream>
#include <string>
#include <random>
#include <cmath>
#include <vector>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/action.hpp>

#include <beluga/beluga.hpp>

// Tutorial parameters
static constexpr int mapSize = 100;
static constexpr int numDoors = mapSize / 3; 
static constexpr int kNumParticles = 200;

// TODO: use template to use only one random function.
bool generateRandomMeasurement(double mean, double sd) {
    static auto generator = std::mt19937{std::random_device()()};
    std::normal_distribution<double> distribution(mean, sd);

    // Generate a random number and check if it's greater than 0.5
    return distribution(generator) > 0.5;
}

double generateRandomOdom(double mean, double sd) {
    static auto generator = std::mt19937{std::random_device()()};
    std::normal_distribution<double> distribution(mean, sd);

    return (distribution(generator));
}

int main()
{
    std::cout << "beluga tutorial - main.cpp start!!" << std::endl;

    // Generate a random map
    std::uniform_int_distribution landmark_distribution{0, mapSize};
    auto landmark_map = beluga::views::sample(landmark_distribution) |
                        ranges::views::take_exactly(numDoors)        |
                        ranges::to<std::vector<int>>;
    // TODO(alon): the map has to be sort and unique or not necessarily? for now it helps for testing purpose.
    landmark_map |= ranges::actions::sort | ranges::actions::unique;
    std::cout << "landamrk_map" << std::endl;
    for(const auto& lm : landmark_map)
    {
        std::cout << lm << ",";
    }
    std::cout << std::endl;
    std::cout << std::endl;

    // Save the map in a csv file
    {
        std::string filename{"map.csv"};
        std::fstream fout {filename, fout.trunc | fout.out};
        fout << "mark_pose" << std::endl;
        for(const auto& l: landmark_map)
        {
            fout << l << std::endl;
        }
    }

    // Generate particles
    std::uniform_int_distribution initial_distribution{0, mapSize};
    using Particle = std::tuple<int, beluga::Weight>;
    auto particles = beluga::views::sample(initial_distribution)                 |
                     ranges::views::transform(beluga::make_from_state<Particle>) |
                     ranges::views::take_exactly(kNumParticles)                  |
                     ranges::to<beluga::TupleVector>;
    
    // TODO(alon): code doesn't compile. landmark_map structure was changed.
    // Execute the particle filter using beluga
    double prev_pose = landmark_map[0].pose;
    for (const auto& l : landmark_map) {
        
        // Generate noisy odom and measurement
        double odom = generateRandomOdom(l.pose, 0.1);
        bool measurement = generateRandomMeasurement(l.mark, 0.35);

        auto motion_model = [&](double state) {
            double distance = odom - prev_pose;
            prev_pose = odom;
            double translaton_param = generateRandomOdom(0, 0.3);
            return state + distance - translaton_param;
        };

        auto sensor_model = [&](const double& state) {
            
            double no_landmark_probability = 1;
            std::vector<double> landmark_probability;
            double factor = 0;

            for(const auto& l : landmark_map)
            {
                if(l.mark == 1)
                {
                    double probability = exp( -1 * pow(l.pose - state, 2));
                    factor += probability;
                    landmark_probability.push_back(probability);
                    no_landmark_probability *= (1 - probability);
                }
            }

            factor += no_landmark_probability;

            // Normalize and sum
            no_landmark_probability /= factor;
            double probability_landmark_normalize_sum = 0;
            for(const auto& pl : landmark_probability)
            {
                probability_landmark_normalize_sum += (pl/factor);
            }

            if(measurement == 1)
            {
                return probability_landmark_normalize_sum;
            }
            return no_landmark_probability;
        };

        // TODO: To showcase the example, it may be useful to apply each range adaptor separately and save the particles
        // in a file for display them using some plot lib.
        particles |= beluga::actions::propagate(std::execution::seq, motion_model) |
                     beluga::actions::reweight(std::execution::seq, sensor_model)  | 
                     beluga::actions::normalize;

        // Resample
        particles |= beluga::views::sample | ranges::views::take_exactly(kNumParticles)|
                    beluga::actions::assign;

        // TODO: It is throwing me a compiling error when I'm trying to use the estimate function.
        // const auto [mean, covariance] = beluga::estimate(particles);
        // std::cout << "mena: " << mean << ", covariance: " << covariance << std::endl;
    }

    return 0;
}