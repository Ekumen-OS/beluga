// https://github.com/Ekumen-OS/beluga/issues/279#issuecomment-1903914387

#include <iostream>
#include <string>
#include <random>
#include <cmath>
#include <vector>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <beluga/beluga.hpp>

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

    struct Landmark {
        double x_position;
        bool feature; // 0 = wall; 1 = Door
    };

    std::vector<Landmark> landmark_map = {
        {0.0, 0},
        {1.0, 0},
        {2.0, 0},
        {3.0, 1},
        {4.0, 0},
        {5.0, 1},
        {6.0, 0},
        {7.0, 0},
        {8.0, 0},
        {9.0, 1},
        {10.0, 0}
    };

    std::uniform_int_distribution initial_distribution{0, static_cast<int>(landmark_map.size())};

    std::cout << "landmark_map size: " << static_cast<int>(landmark_map.size()) << std::endl;

    using Particle = std::tuple<double, beluga::Weight>;

    auto particles = beluga::views::sample(initial_distribution) |
                 ranges::views::transform(beluga::make_from_state<Particle>) |
                 ranges::views::take_exactly(50) |
                 ranges::to<beluga::TupleVector>;

    struct Dataset {
        double odom;
        bool measurement;  
    };

    // Add noise to odom and measurement
    std::vector<Dataset> data_set;
    for (const auto& l : landmark_map) {
        double odom = generateRandomOdom(l.x_position, 0.1);
        bool measurement = generateRandomMeasurement(l.feature, 0.35);
        data_set.push_back({odom, measurement});
    }

    // Display the resulted dataset
    // std::cout << "Dataset:" << std::endl;
    // for (const auto& d : data_set) {
    //     std::cout << "Odom: " << d.odom << ", Measurement: " << d.measurement << std::endl;
    // }

    for (const auto& d : data_set) {
        
        //TODO: save the last position, calculate the traslation and return it
        auto motion_model = [&](double odom) {
            odom = d.odom + 1;
            return odom;
        };

        // TODO: ask here about the sensor model. It is correct to use the formula w = w - 1?
        // w = exp(x), x = -(d^2)/(2*sigma), d = distance between coincidence reading
        auto sensor_model = [&](const double& state) {

            // Calculate w
            double w = 0;
            for(const auto& l : landmark_map)
            {
                if(l.feature == d.measurement)
                {
                    w += exp( -1 * ((pow(l.x_position - state, 2)) / (2 * 0.2)) ) ;
                }
            }
            if(d.measurement == 0)
            {
                w = 1 - w;
            }
            return w;
        };

        // TODO: To showcase the example, it may be useful to apply each range adaptor separately and save the particles
        // in a file for display them using some plot lib.
        particles |= beluga::actions::propagate(std::execution::seq, motion_model) |
                     beluga::actions::reweight(std::execution::seq, sensor_model)  | 
                     beluga::actions::normalize;

        // Resample
        // TODO: It is throwing me a compiling error when I'm trying to resample the particles.
        // TODO: which distribution use for the resample?
        std::uniform_int_distribution resample_distribution{0, static_cast<int>(landmark_map.size())};
        particles |= beluga::views::sample | ranges::views::take_exactly(50)|
                    beluga::actions::assign;

        // TODO: It is throwing me a compiling error when I'm trying to use the estimate function.
        // const auto [mean, covariance] = beluga::estimate(particles);
        // std::cout << "mena: " << mean << ", covariance: " << covariance << std::endl;
    }

    return 0;
}