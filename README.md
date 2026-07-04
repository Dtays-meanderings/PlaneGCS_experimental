# PlaneGCS

PlaneGCS is a standalone, modernized 2D geometric constraint solver. Originally extracted from FreeCAD's GCS solver, this version is completely decoupled from Boost and other heavy external dependencies. It features native CPython bindings and high-performance optimizations.

## Features
* **Standalone C++ Core**: Zero external dependencies other than Eigen3. Decoupled from Boost.Graph.
* **Modern Memory Management**: Automatic resource cleanup using smart pointers (`std::unique_ptr`).
* **CPython Bindings**: Native wrapper module (`planegcs`) built using `pybind11` for programmatic constraint solving in Python.
* **Numerical Optimizations**: Solves reduced Hessian equations using high-speed `Eigen::LDLT` with robust QR fallback.
* **Parallel Solving**: Solves topologically partitioned, independent subsystems concurrently on multi-core CPUs.
* **Log Callbacks**: Connects python callbacks directly to solver diagnostics.

## Building and Testing

### Prerequisites
* CMake (>= 3.15)
* C++17 compatible compiler
* Eigen3

### Build Standalone Solver & Tests
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

### Run C++ Tests
```bash
./build/Release/PlaneGcsTests
```

### Install CPython Bindings
```bash
pip install .
```
