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

#include <beluga/beluga.hpp>

#define NUM_OF_PARTICLES 200

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

    // Type of map: 1D map composed by walls or doors
    struct Landmark {
        double pose;
        bool feature; // 0 = wall; 1 = Door
    };

    // Generate a random map
    const size_t map_size = 100;
    std::vector<Landmark> landmark_map;
    for(size_t i = 0; i < map_size; i++)
    {
        Landmark l;
        l.pose = static_cast<double>(i);
        l.feature = generateRandomMeasurement(0, 0.5);
        landmark_map.push_back(l);
    }

    // Save the map in a csv file
    std::string filename{"map.csv"};
    std::fstream fout {filename, fout.trunc | fout.out};
    fout << "pose,feature" << std::endl;
    for(const auto& l: landmark_map)
    {
        fout << l.pose << ","
             << l.feature    << std::endl;
    }

    // Generate particles
    std::uniform_int_distribution initial_distribution{0, static_cast<int>(landmark_map.size())};
    using Particle = std::tuple<double, beluga::Weight>;
    auto particles = beluga::views::sample(initial_distribution) |
                 ranges::views::transform(beluga::make_from_state<Particle>) |
                 ranges::views::take_exactly(NUM_OF_PARTICLES) |
                 ranges::to<beluga::TupleVector>;

    // Execute the particle filter using beluga
    double prev_pose = landmark_map[0].pose;
    for (const auto& l : landmark_map) {
        
        // Generate noisy odom and measurement
        double odom = generateRandomOdom(l.pose, 0.1);
        bool measurement = generateRandomMeasurement(l.feature, 0.35);

        auto motion_model = [&](double state) {
            double distance = odom - prev_pose;
            prev_pose = odom;
            double translaton_param = generateRandomOdom(0, 0.3);
            return state + distance - translaton_param;
        };

        auto sensor_model = [&](const double& state) {
            
            double probability_no_landmark = 1;
            std::vector<double> probability_landmark;
            double factor = 0;

            for(const auto& l : landmark_map)
            {
                if(l.feature == 1)
                {
                    double temp = exp( -1 * pow(l.pose - state, 2));
                    factor += temp;
                    probability_landmark.push_back(temp);
                    probability_no_landmark *= (1 - temp);
                }
            }

            factor += probability_no_landmark;

            // Normalize and sum
            probability_no_landmark /= factor;
            double probability_landmark_normalize_sum = 0;
            for(const auto& pl : probability_landmark)
            {
                probability_landmark_normalize_sum += (pl/factor);
            }

            if(measurement == 1)
            {
                return probability_landmark_normalize_sum;
            }
            return probability_no_landmark;
        };

        // TODO: To showcase the example, it may be useful to apply each range adaptor separately and save the particles
        // in a file for display them using some plot lib.
        particles |= beluga::actions::propagate(std::execution::seq, motion_model) |
                     beluga::actions::reweight(std::execution::seq, sensor_model)  | 
                     beluga::actions::normalize;

        // Resample
        particles |= beluga::views::sample | ranges::views::take_exactly(NUM_OF_PARTICLES)|
                    beluga::actions::assign;

        // TODO: It is throwing me a compiling error when I'm trying to use the estimate function.
        // const auto [mean, covariance] = beluga::estimate(particles);
        // std::cout << "mena: " << mean << ", covariance: " << covariance << std::endl;
    }

    return 0;
}