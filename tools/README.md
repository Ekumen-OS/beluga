# Tools

Repository-level tools for static analysis, code coverage, and documentation generation for Beluga.

See the [getting started](../GETTING_STARTED.md) tutorial to create a development container where you can run the following commands.

1. **Run tests and generate code coverage report**. You will find a report in `lcov/index.html`.
   ```bash
   cd /ws
   ./src/beluga/tools/test.sh
   ```

1. **Run static analysis tools**.
   ```bash
   cd /ws
   ./src/beluga/tools/clang-tidy.sh
   ```

1. **Generate API documentation**.
   ```bash
   cd /ws
   ./src/beluga/tools/doxygen.sh
   ```
