# LibGNSS++ - Next-Generation GNSS Processing Library

A modern, high-performance C++ library for GNSS positioning and navigation, designed to democratize GNSS development.

## Overview

LibGNSS++ is a next-generation open-source GNSS processing library that aims to provide:

- **Multi-constellation support**: GPS, GLONASS, Galileo, BeiDou, QZSS
- **Advanced algorithms**: Modern RTK, PPP, and hybrid positioning
- **High performance**: Optimized C++17/20 implementation
- **Modular design**: Extensible architecture for research and development
- **Easy integration**: Clean API and comprehensive documentation

## Features

### Core Capabilities
- Real-time and post-processing positioning
- Single Point Positioning (SPP)
- Real-Time Kinematic (RTK) positioning
- Precise Point Positioning (PPP)
- Multi-frequency, multi-constellation processing
- Ionospheric and tropospheric modeling
- Ambiguity resolution algorithms

### Modern Architecture
- Header-only template library for performance
- Plugin-based architecture for algorithms
- Thread-safe design for multi-threaded applications
- Memory-efficient data structures
- SIMD optimizations where applicable

### Supported Data Formats
- RINEX 2.x/3.x observation and navigation files
- RTCM 2.x/3.x real-time correction streams
- UBX, NMEA, and other receiver formats
- SP3 precise orbit files
- ANTEX antenna models

## Quick Start

```cpp
#include <libgnss++/gnss.hpp>

int main() {
    // Initialize GNSS processor
    libgnss::GNSSProcessor processor;
    
    // Load configuration
    processor.loadConfig("config.yaml");
    
    // Process RINEX observation file
    auto solution = processor.processFile("observation.obs", "navigation.nav");
    
    // Output results
    solution.writeToFile("solution.pos");
    
    return 0;
}
```

## Building

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Requirements

- C++17 compatible compiler (GCC 7+, Clang 6+, MSVC 2019+)
- CMake 3.15+
- Eigen3 (for linear algebra)
- Optional: OpenMP (for parallelization)

## License

MIT License - see LICENSE file for details.

## Contributing

We welcome contributions! Please see CONTRIBUTING.md for guidelines.

## Acknowledgments

Inspired by RTKLIB and the need for a modern, accessible GNSS processing platform.
