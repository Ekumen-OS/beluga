^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package beluga_system_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2025-11-25)
------------------
* Drop Noetic support (`#520 <https://github.com/Ekumen-OS/beluga/issues/520>`_)
* Add ROS Kilted Kaiju support (`#485 <https://github.com/Ekumen-OS/beluga/issues/485>`_)
* Contributors: Andrés Brumovsky, Michel Hidalgo

2.0.2 (2024-06-18)
------------------

2.0.1 (2024-05-24)
------------------

2.0.0 (2024-05-21)
------------------
* Please clang-tidy on Ubuntu Noble (`#379 <https://github.com/Ekumen-OS/beluga/issues/379>`_)
* Update missed ROS version detection logic (`#368 <https://github.com/Ekumen-OS/beluga/issues/368>`_)
* Add AMCL implementation for ROS (`#327 <https://github.com/Ekumen-OS/beluga/issues/327>`_)
* Migrate sensor models to functional form (`#325 <https://github.com/Ekumen-OS/beluga/issues/325>`_)
* Relocate `make_random_state()` method (`#324 <https://github.com/Ekumen-OS/beluga/issues/324>`_)
* Add uniform distribution with bounding regions (`#322 <https://github.com/Ekumen-OS/beluga/issues/322>`_)
* Migrate motion models to functional form (`#321 <https://github.com/Ekumen-OS/beluga/issues/321>`_)
* Add uniform free-space grid distribution (`#319 <https://github.com/Ekumen-OS/beluga/issues/319>`_)
* Make test matchers and printers available to users (`#300 <https://github.com/Ekumen-OS/beluga/issues/300>`_)
* Extend multivariate distribution to support different result types (`#298 <https://github.com/Ekumen-OS/beluga/issues/298>`_)
* Add normalize action (`#297 <https://github.com/Ekumen-OS/beluga/issues/297>`_)
* Extend sample view to sample from random distributions (`#296 <https://github.com/Ekumen-OS/beluga/issues/296>`_)
* Add recovery probability estimator (`#295 <https://github.com/Ekumen-OS/beluga/issues/295>`_)
* Add declarative policies (`#294 <https://github.com/Ekumen-OS/beluga/issues/294>`_)
* Run system tests with parallel execution enabled (`#293 <https://github.com/Ekumen-OS/beluga/issues/293>`_)
* Add system tests with the range-based API (`#292 <https://github.com/Ekumen-OS/beluga/issues/292>`_)
* Motion and sensor models are not mixins (`#291 <https://github.com/Ekumen-OS/beluga/issues/291>`_)
* Integrate views with mixins (`#284 <https://github.com/Ekumen-OS/beluga/issues/284>`_)
* Transform points in laser frame to robot frame (`#276 <https://github.com/Ekumen-OS/beluga/issues/276>`_)
* Generalize planar laser scan support (`#271 <https://github.com/Ekumen-OS/beluga/issues/271>`_)
* Add new discrete landmark and bearing sensor models to the beluga library (`#268 <https://github.com/Ekumen-OS/beluga/issues/268>`_)
* Add `beluga_ros` package (`#270 <https://github.com/Ekumen-OS/beluga/issues/270>`_)
* Fix missing control to force a filter update on pose updates (`#248 <https://github.com/Ekumen-OS/beluga/issues/248>`_)
* Add cmake-format to pre-commit hooks (`#243 <https://github.com/Ekumen-OS/beluga/issues/243>`_)
* Refactor resampling policies into filter update control (`#233 <https://github.com/Ekumen-OS/beluga/issues/233>`_)
* Wrap up ROS Noetic support (`#225 <https://github.com/Ekumen-OS/beluga/issues/225>`_)
* Re-organize infrastructure scripts (`#197 <https://github.com/Ekumen-OS/beluga/issues/197>`_)
* Add support for map updates in laser localization filters (`#189 <https://github.com/Ekumen-OS/beluga/issues/189>`_)
* Improve documentation and guidelines (`#186 <https://github.com/Ekumen-OS/beluga/issues/186>`_)
* Add beam sensor model (`#160 <https://github.com/Ekumen-OS/beluga/issues/160>`_)
* Remove Git LFS from the repository (`#162 <https://github.com/Ekumen-OS/beluga/issues/162>`_)
* Rename `importance_sample` method to `reweight` (`#158 <https://github.com/Ekumen-OS/beluga/issues/158>`_)
* Add support for per-axis resolution when clustering (`#146 <https://github.com/Ekumen-OS/beluga/issues/146>`_)
* Remove ament_clang_format from system tests (`#155 <https://github.com/Ekumen-OS/beluga/issues/155>`_)
* Run clang-tidy in CI instead of using pre-commit for it (`#147 <https://github.com/Ekumen-OS/beluga/issues/147>`_)
* Rename `mixin::make_unique` to `make_mixin` (`#152 <https://github.com/Ekumen-OS/beluga/issues/152>`_)
* Add system tests (`#138 <https://github.com/Ekumen-OS/beluga/issues/138>`_)

* Contributors: Gerardo Puga, Ivan Santiago Paunovic, Michel Hidalgo, Nahuel Espinosa, Ramiro Serra
