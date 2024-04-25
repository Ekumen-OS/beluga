# Design principles

## UX centered

### Easy to (re)use

Beluga's design regularly trades complex generic implementations for simple, reusable interfaces. Much like a box of tools, Beluga provides the user with algorithms, data structures, even full programs, that can be mixed and matched to support an implementation. Great lengths have been covered to afford readable user code without loss of performance: no type is specialized unless strictly necessary, and simple control and data flow idioms apply when programming with Beluga.

### STL consistent

For minimum surprise, Beluga reproduces and specializes many standard library (STL) concepts, such as random distributions, generic containers, type traits, and range views and actions. Conversely, standard classes and functions can often be used in place for Beluga's (when appropriate).

## Performance focused

### Statically bound

To make the most out of compile-time optimizations and minimize the overhead of its abstractions, Beluga's design strives for statically bound types, keeping runtime polymorphism and type erasure use down to the bare minimum. As a result, Beluga relies heavily on generic programming and metaprogramming techniques to support a wide-range of applications and use cases. The flip side of this is longer compilation times (e.g. when evaluating complex compile-time logic) and larger binaries (e.g. when aggregate data types need unwrapping).

### Lazy evaluated

Owing to the adoption of `Eigen`, `Sophus`, and `rangesv3` abstractions and concepts in core libraries, Beluga's design is naturally lazy when performing computations, sparing cycles if wisely composed. The flip side of this is a higher cognitive load when extending Beluga e.g. care must be exercised to avoid partial in-place mutations of sampled distributions from affecting the computation itself.

### Memory aware

To avoid burning throughput to onerous memory allocations and access, Beluga's design favors early stack allocations and zero-copy abstractions. Furthermore, it affords some degree of control over memory layouts to optimize for cache locality.
