# Contributing to LibGNSS++

We welcome contributions to LibGNSS++! This document provides guidelines for contributing to the project.

## Getting Started

1. Fork the repository on GitHub
2. Clone your fork locally
3. Create a new branch for your feature or bugfix
4. Make your changes
5. Test your changes
6. Submit a pull request

## Development Environment

### Prerequisites

- C++17 compatible compiler (GCC 7+, Clang 6+, MSVC 2019+)
- CMake 3.15+
- Eigen3 library
- Google Test (for running tests)

### Building

```bash
git clone https://github.com/your-username/libgnss++.git
cd libgnss++
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Running Tests

```bash
cd build
ctest --output-on-failure
```

## Code Style

- Follow modern C++ best practices
- Use meaningful variable and function names
- Add comprehensive documentation for public APIs
- Include unit tests for new functionality
- Use const-correctness
- Prefer RAII and smart pointers
- Follow the existing code style in the project

### Naming Conventions

- Classes: `PascalCase` (e.g., `GNSSProcessor`)
- Functions and variables: `snake_case` (e.g., `process_epoch`)
- Constants: `UPPER_SNAKE_CASE` (e.g., `SPEED_OF_LIGHT`)
- Private members: trailing underscore (e.g., `member_variable_`)

## Documentation

- Use Doxygen-style comments for all public APIs
- Include usage examples in documentation
- Update README.md if adding new features
- Add inline comments for complex algorithms

## Testing

- Write unit tests for all new functionality
- Ensure tests pass on all supported platforms
- Include edge cases and error conditions
- Use Google Test framework
- Aim for high code coverage

## Pull Request Process

1. Ensure your code builds without warnings
2. Run all tests and ensure they pass
3. Update documentation as needed
4. Add entry to CHANGELOG.md if applicable
5. Create a clear pull request description explaining:
   - What changes were made
   - Why the changes were necessary
   - How to test the changes

## Reporting Issues

When reporting issues, please include:

- Operating system and version
- Compiler and version
- LibGNSS++ version
- Minimal code example reproducing the issue
- Expected vs actual behavior
- Any relevant error messages

## Feature Requests

For feature requests, please:

- Check if the feature already exists
- Describe the use case clearly
- Explain why the feature would be beneficial
- Consider if it fits with the project's goals

## Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Help newcomers get started
- Maintain professional communication

## License

By contributing to LibGNSS++, you agree that your contributions will be licensed under the MIT License.

## Questions?

If you have questions about contributing, please:

- Check the documentation
- Search existing issues
- Create a new issue with the "question" label
- Join our community discussions

Thank you for contributing to LibGNSS++!
