# Security Policy: Radar Reprojection

## Reporting a Vulnerability

If you discover a potential security vulnerability, please **do not open a
public issue or pull request**.

- Report it through the
  [NVIDIA Vulnerability Disclosure Program](https://www.nvidia.com/en-us/security/)
  (preferred).
- Email [psirt@nvidia.com](mailto:psirt@nvidia.com). For secure communication,
  use the [NVIDIA public PGP key](https://www.nvidia.com/en-us/security/pgp-key).
- On GitHub, use the repository's **Security** tab and select
  **Report a vulnerability**.

Include the affected version or commit, vulnerability type, reproduction steps,
proof-of-concept material when available, and the potential impact. NVIDIA's
Product Security Incident Response Team will acknowledge the report, validate
and assess it, coordinate a fix, and publish a security bulletin when
appropriate.

## Security Architecture and Context

Radar Reprojection is a C++ library with a gRPC service and Python example
clients for synthesizing radar detections from logged radar and trajectory data.
Its primary security boundaries are the gRPC request interface, local dataset
and configuration files, generated output files, and third-party dependencies.

**Repository Exposure Classification:** Public.

Basis: the repository is being prepared for an open-source release; this
document is written for public consumption.

**Service Exposure Classification:** External / Regulated (high confidence).

Basis: the software is intended for external distribution and includes a
network-listening gRPC server, even though the documented workflow is local
simulation and evaluation.

The server in `server/MainServer.cpp` uses cleartext gRPC and does not implement
authentication or authorization. The supplied clients load local radar,
calibration, label, and trajectory data and send it to the server. Deployments
must establish appropriate network, identity, and data-access controls outside
the application.

### Threat Model

1. **Unauthenticated gRPC access:** The server listens on a configurable address
   and accepts `Initialize` and `Reproject` calls without authenticating the
   caller. An exposed instance could allow an untrusted caller to replace loaded
   state, retrieve derived radar data, or consume compute and memory resources.
2. **Resource exhaustion through large inputs:** `Initialize` accepts nested log
   data and the server supports large gRPC messages. Oversized or adversarial
   requests can cause excessive allocation, preprocessing, or lock contention.
3. **Malformed radar and trajectory data:** Protobuf conversion, JSON loading,
   and numerical reprojection operate on caller-controlled shapes, transforms,
   timestamps, and floating-point values. Invalid values may cause failed
   requests, unstable numerical results, or denial of service.
4. **Untrusted dataset archives and paths:** The sample-data helper downloads and
   extracts dataset archives. Users must trust the selected data source and
   ensure output directories do not expose or overwrite sensitive files.
5. **Dependency and generated-code compromise:** Bazel modules, Python packages,
   generated protobuf bindings, and the locally supplied Eigen headers become
   part of the build or runtime trust boundary. A compromised dependency or
   build environment could alter produced binaries.
6. **Sensitive information in logs or output:** Dataset paths, radar identifiers,
   generated plots, and error strings may disclose information about local data
   or deployment layout when logs and artifacts are shared.

### Critical Security Assumptions

- The gRPC server is bound to a trusted interface or protected by an external
  authenticated and encrypted transport. It is not safe to expose directly to
  an untrusted network with the default configuration.
- Callers validate dataset provenance, authorization, dimensions, and numerical
  ranges before submitting data to the server.
- The host operating system enforces process and filesystem isolation, and the
  service runs with only the permissions needed for its workload.
- Dependency sources, the Bazel toolchain, the Python environment, and generated
  protobuf code are obtained from trusted and integrity-checked sources.
- Downloaded PhysicalAI-AV data remains subject to its separate license and is
  not redistributed with this project.

## Supported Versions

Security fixes are applied to the latest release and the current `main` branch.
Users should reproduce reports against the newest available version whenever
possible.
