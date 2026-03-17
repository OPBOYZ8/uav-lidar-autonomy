# Contributing

Thank you for your interest in this project. Contributions are welcome in the following areas:

- Bug reports and bug fixes
- Documentation improvements
- Navigation algorithm improvements
- New sensor or environment support
- ROS 2 port
- Hardware deployment support

## Development Setup

Follow the [Installation](README.md#installation) steps to build the workspace. Use the
`--cmake-args -DCMAKE_BUILD_TYPE=Debug` flag when debugging:

```bash
cd catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Code Style

- **Python**: follow PEP 8. Maximum line length 100 characters.
- **ROS nodes**: use `rospy.loginfo / logwarn / logerr` — never `print()`.
- **Launch files**: all tuneable parameters must be exposed as `<arg>` elements with
  `doc=` strings and sensible defaults.
- **Comments**: explain *why*, not *what*. Geometry calculations and algorithm decisions
  must be documented inline.

## Pull Request Process

1. Fork the repository and create a feature branch from `main`.
2. Write or update unit tests for any changed navigation or sensor logic.
3. Run the test suite before submitting:
   ```bash
   python3 -m pytest /tmp/wall_fix_tests_v3.py -v
   ```
4. Ensure `catkin_make` exits without errors or warnings.
5. Update `CHANGELOG.md` under the `[Unreleased]` section.
6. Open a pull request with a clear description of the change, its motivation,
   and any relevant test results.

## Reporting Bugs

Please open a GitHub issue with:
- ROS version (`rosversion -d`)
- Ubuntu version (`lsb_release -a`)
- Minimal steps to reproduce
- Observed vs. expected behaviour
- Relevant `rostopic echo /uav/nav_debug` output if the bug is navigation-related
