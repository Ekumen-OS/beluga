# Architecture

Beluga is a collection of reusable components, implementing specific concepts. While diverse, these concepts are modeled after a few well-known patterns and idioms in modern C++ programming.

## Distribution as a range

Monte Carlo approximations to probability distributions operate on collections of samples. The key word here is _collection_. C++ [ranges](https://github.com/ericniebler/range-v3) are a powerful abstraction to work with collections of any sort, and Beluga leverages them throughout, for implementation and in user-facing APIs. An example of this are the `beluga::sample` and `beluga::propagate` functors, modeling two forms of distribution sampling as range view and action respectively.

## STL-like abstractions

The C++ standard library contains plenty useful and widely adopted abstractions and concepts, many of which Beluga itself uses. To simplify that integration and encourage it, Beluga reuses and extends wherever possible. An example of this are the `beluga::TupleVector` and `beluga::MultivariateNormalDistribution` classes. These are interchangeable with the `std::vector<std::tuple<T>>` class, and a generalization of the `std::normal_distribution` class, respectively.

## Functional forms

Probability distributions, likelihood functions, policy predicates, can naturally be defined as functions. Beluga preserves those functional forms using regular functions, functors, and higher-order functions (typically, lambdas) when appropriate. An example of this are the `MotionModel` and `SensorModel` concepts, implemented by the `beluga::DifferentialDriveModel` and `beluga::LikelihoodFieldModel` classes.

## CRTP-based data adapters

Data comes and goes in many shapes and forms. Beluga does not try to enforce any, but like any other software it does need to put expectations on data. To bridge that gap efficiently, Beluga relies on adaptation patterns based on the [_curiously recurring template pattern_](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern) idiom. An example of this are the `OccupancyGrid2d` and `LaserScan` concepts, implemented by the `beluga::BaseOccupancyGrid2` and `beluga::BaseLaserScan` classes, and leveraged by the `beluga_ros::OccupancyGrid` and `beluga_ros::LaserScan` classes.
