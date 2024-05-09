# Rationale

Beluga's _raison d'être_ rests on a relatively mundane observation: there is a large body of research in Monte Carlo methods for Bayesian inference with applications in robotics that has not yet seen wide industry adoption simply because few quality implementations exist.

MCL is a good example of this. It first became popular in the 1990s, and there many MCL variations and many (though fewer) MCL implementations to be found in the wild. But most of those implementations never intended to be anything but a proof of concept. There is little reuse and so features are scattered. Licensing terms and code quality vary substantially, and support is in many cases close to non-existent.

Even though Beluga started out as a ground-up re-implementation of AMCL for ROS, and currently stands as a toolkit for Monte Carlo Localization (MCL), the long-term vision is for it to become a general-purpose framework for Bayesian inference that academic researchers and industry practitioners can build on and reuse.
