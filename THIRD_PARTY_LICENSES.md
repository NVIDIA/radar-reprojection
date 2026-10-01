# Third-Party Software Licenses

Radar Reprojection uses the third-party open-source software listed below.
These dependencies are resolved separately by Bazel, pip, or the user's Conda
environment; their source code and binary distributions are not vendored in
this repository. Each dependency remains subject to its own license.

## C++ and Bazel dependencies

| Dependency | Version | Use | License |
|---|---:|---|---|
| [Eigen](https://gitlab.com/libeigen/eigen) | 3.4 | C++ linear algebra; supplied by the user's Conda environment | MPL-2.0 |
| [nlohmann/json](https://github.com/nlohmann/json) | 3.11.3 | JSON parsing and serialization | MIT |
| [gRPC](https://github.com/grpc/grpc) | 1.66.0 | RPC runtime and code generation | Apache-2.0 |
| [Protocol Buffers](https://github.com/protocolbuffers/protobuf) | 29.0 | Message serialization and code generation | BSD-3-Clause |
| [GoogleTest](https://github.com/google/googletest) | 1.15.2 | Tests only | BSD-3-Clause |
| [rules_cc](https://github.com/bazelbuild/rules_cc) | 0.0.17 | Bazel C/C++ build rules | Apache-2.0 |
| [platforms](https://github.com/bazelbuild/platforms) | 0.0.10 | Bazel platform constraints | Apache-2.0 |
| [rules_proto](https://github.com/bazelbuild/rules_proto) | 7.0.2 | Bazel Protocol Buffers rules | Apache-2.0 |
| [rules_python](https://github.com/bazelbuild/rules_python) | 0.36.0 | Bazel Python and pip rules | Apache-2.0 |

The Bazel module resolver may download additional transitive build/runtime
dependencies recorded in `MODULE.bazel.lock`. Those packages are not included
in this repository and remain governed by the license information distributed
with each package.

## Python dependencies

| Dependency | Version | License |
|---|---:|---|
| [grpcio](https://pypi.org/project/grpcio/1.83.1/) | 1.83.1 | Apache-2.0 |
| [typing_extensions](https://pypi.org/project/typing_extensions/4.16.0/) | 4.16.0 | PSF-2.0 |
| [protobuf](https://pypi.org/project/protobuf/5.29.6/) | 5.29.6 | BSD-3-Clause |
| [NumPy](https://pypi.org/project/numpy/1.26.4/) | 1.26.4 | BSD-3-Clause |
| [PyArrow](https://pypi.org/project/pyarrow/25.0.1/) | 25.0.1 | Apache-2.0 |
| [Matplotlib](https://pypi.org/project/matplotlib/3.8.4/) | 3.8.4 | PSF-based Matplotlib License |
| [ContourPy](https://pypi.org/project/contourpy/1.2.1/) | 1.2.1 | BSD-3-Clause |
| [Cycler](https://pypi.org/project/cycler/0.12.1/) | 0.12.1 | BSD-3-Clause |
| [FontTools](https://pypi.org/project/fonttools/4.64.0/) | 4.64.0 | MIT |
| [KiwiSolver](https://pypi.org/project/kiwisolver/1.4.5/) | 1.4.5 | BSD-3-Clause |
| [packaging](https://pypi.org/project/packaging/24.0/) | 24.0 | Apache-2.0 OR BSD-2-Clause |
| [Pillow](https://pypi.org/project/pillow/12.3.0/) | 12.3.0 | MIT-CMU |
| [pyparsing](https://pypi.org/project/pyparsing/3.1.2/) | 3.1.2 | MIT |
| [python-dateutil](https://pypi.org/project/python-dateutil/2.9.0.post0/) | 2.9.0.post0 | Apache-2.0 OR BSD-3-Clause |
| [six](https://pypi.org/project/six/1.16.0/) | 1.16.0 | MIT |

## Optional dataset tooling and data

The optional [PhysicalAI-AV Python
devkit](https://pypi.org/project/physical-ai-av/) is MIT-licensed and is used
only by the sample-data download workflow. PhysicalAI-Autonomous-Vehicles
dataset content is not included in this repository and remains subject to the
separate license presented by its distribution service.
