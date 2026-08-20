# Radar Reprojection gRPC Server — Design

Date: 2026-05-08
Status: Draft (pending implementation plan)
Authors: yahlu@nvidia.com

## 1. Overview & scope

Expose `algorithm::RadarReprojector::initialize()` and `reproject()` over gRPC.
The server is a thin translation layer: protobuf → algorithm structs →
algorithm call → algorithm structs → protobuf. No dataset loading, no
parquet-derived enrichment fields, no clip-uuid awareness inside the server.

`RadarReprojectionManager` (the previous in-process facade that wrapped the
algorithm with `PhysicalAIAVLoader` and added enrichment) is removed. Any
caller that previously linked `RadarReprojectionManager` now connects to the
server over gRPC instead. The `algorithm_demo` binary continues to call the
algorithm library directly and is the canonical "no gRPC" path.

### Repository delta

| Path                                                       | Status                                                                              |
| ---------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| `algorithm/`                                               | Unchanged. `algorithm:algorithm` and `algorithm_demo` still build.                  |
| `proto/radar_reprojector.proto`                            | Unchanged (already authored).                                                       |
| `proto/BUILD.bazel`                                        | **New.** `proto_library` + C++ and Python codegen rules.                            |
| `server/`                                                  | **New.** `RadarReprojectionServer`, `Conversions`, `MainServer`, tests.             |
| `examples/python/smoke_client.py`                          | **New.** End-to-end smoke check; not a real client library.                         |
| `RadarReprojectionManager.{hpp,cpp}` + its top-level target | **Removed.**                                                                        |
| `dataset/test/RadarReprojectionManager_test.cpp`           | **Removed.**                                                                        |
| `dataset/PhysicalAIAVLoader` and the rest of `dataset/`    | Largely untouched (one stale comment in `RadarScan.hpp` updated — see §10). Orphan from the server's POV; not linked into the new server binary. |
| `MODULE.bazel`                                             | **Edited.** New `bazel_dep`s for `grpc`, `protobuf`, `rules_proto`, `rules_python`. |
| `README.md`                                                | **Edited.** Replace the RRM section with a server quick-start.                      |

## 2. Wire format

The proto file `proto/radar_reprojector.proto` already mirrors the algorithm
structs. Two RPCs:

```proto
service RadarReprojectorService {
    rpc Initialize(InitializeRequest) returns (InitializeResponse);
    rpc Reproject(ReprojectRequest)   returns (ReprojectResponse);
}
```

Notable design points already baked into the proto:

- **Pose representation.** `Pose { Vec3f translation; Quaternionf rotation }`.
  `Quaternionf` ordering is `(x, y, z, w)` to match `Eigen::Quaternionf`.
  Rigid transforms only — the algorithm never uses scaled or sheared affines.
- **`int64` for timestamps.** `dwTime_t` in the algorithm is `int64_t` (μs).
- **`from_world` redundancy.** `ActorState.from_world` is on the wire for
  parity with the C++ struct, but is **always recomputed server-side** as
  `to_world.inverse()` — see §5.
- **Error fields.** Per the locked decision in §8, `InitializeResponse` and
  `ReprojectResponse` carry only the success payload (the reprojected scan).
  Errors are returned as gRPC `Status` codes; we will **remove** the
  `success` and `error_message` fields from `InitializeResponse` before
  generating code, leaving an empty message that exists only for future
  extensibility.

### Message-size budget

A clip can serialize to tens of MB of `LogData`. We bump server and client
limits to 256 MB:

- Server: `grpc::ServerBuilder::SetMaxReceiveMessageSize(256 << 20)` and
  `SetMaxSendMessageSize(256 << 20)`.
- C++ tests / Python smoke client: matching channel options.

Initialize is **unary** despite the size — locked decision; chunked-streaming
was rejected to keep the Python client trivial.

## 3. Module layout & build graph

```
proto/
  radar_reprojector.proto
  BUILD.bazel             # proto_library, cc_proto_library, cc_grpc_library,
                          # py_proto_library, py_grpc_library
server/
  BUILD.bazel
  RadarReprojectionServer.hpp
  RadarReprojectionServer.cpp
  Conversions.hpp
  Conversions.cpp
  MainServer.cpp
  test/
    Conversions_test.cpp
    RadarReprojectionServer_test.cpp
examples/python/
  BUILD.bazel
  smoke_client.py
docs/superpowers/specs/
  2026-05-08-radar-reprojection-grpc-server-design.md   # this file
```

### Bazel targets

`proto/BUILD.bazel`:

| Target                              | Rule                                | Purpose                                      |
| ----------------------------------- | ----------------------------------- | -------------------------------------------- |
| `:radar_reprojector_proto`          | `proto_library`                     | Source of truth, language-agnostic.          |
| `:radar_reprojector_cc_proto`       | `cc_proto_library`                  | Generated C++ message classes.               |
| `:radar_reprojector_cc_grpc`        | `cc_grpc_library`                   | Generated C++ service base & stub.           |
| `:radar_reprojector_py_proto`       | `py_proto_library`                  | Generated Python message classes.            |
| `:radar_reprojector_py_grpc`        | `py_grpc_library`                   | Generated Python service stub.               |

`server/BUILD.bazel`:

| Target                                | Rule         | Deps                                                                                                                        |
| ------------------------------------- | ------------ | --------------------------------------------------------------------------------------------------------------------------- |
| `:conversions`                        | `cc_library` | `//algorithm:algorithm`, `//proto:radar_reprojector_cc_proto`, `@eigen`                                                     |
| `:radar_reprojection_server_lib`      | `cc_library` | `:conversions`, `//proto:radar_reprojector_cc_grpc`, `//algorithm:algorithm`, `@grpc//:grpc++`                              |
| `:radar_reprojection_server`          | `cc_binary`  | `:radar_reprojection_server_lib` (entrypoint = `MainServer.cpp`)                                                            |
| `:conversions_test`                   | `cc_test`    | `:conversions`, `@googletest//:gtest_main`                                                                                  |
| `:radar_reprojection_server_test`     | `cc_test`    | `:radar_reprojection_server_lib`, `@grpc//:grpc++`, `@googletest//:gtest_main`                                              |

`examples/python/BUILD.bazel`:

| Target          | Rule        | Deps                                                                            |
| --------------- | ----------- | ------------------------------------------------------------------------------- |
| `:smoke_client` | `py_binary` | `//proto:radar_reprojector_py_proto`, `//proto:radar_reprojector_py_grpc`, `grpcio` |

### `MODULE.bazel` changes

```python
bazel_dep(name = "grpc",          version = "1.66.0")  # provides @grpc, @protobuf transitively
bazel_dep(name = "protobuf",      version = "29.0")
bazel_dep(name = "rules_proto",   version = "7.0.2")
bazel_dep(name = "rules_python",  version = "0.36.0")
```

Versions above are starting points; the implementation plan must verify
compatibility with the existing `rules_cc 0.0.17` and `nlohmann_json 3.11.3`
pins, and adjust if the BCR resolution surfaces conflicts.

## 4. Server class design

```cpp
namespace radar_reprojection::server {

class RadarReprojectionServer final
    : public proto::RadarReprojectorService::Service {
public:
    RadarReprojectionServer();

    grpc::Status Initialize(grpc::ServerContext* ctx,
                            const proto::InitializeRequest* req,
                            proto::InitializeResponse* resp) override;

    grpc::Status Reproject(grpc::ServerContext* ctx,
                           const proto::ReprojectRequest* req,
                           proto::ReprojectResponse* resp) override;

private:
    std::mutex                                       m_mutex;
    std::unique_ptr<algorithm::RadarReprojector>     m_reprojector;     // null until first successful Initialize
    std::shared_ptr<algorithm::LogData>              m_logData;         // owned to satisfy the algorithm's shared_ptr signature
    std::unordered_map<std::string, algorithm::RadarParam> m_radarParams; // for Reproject radarId lookup
};

} // namespace
```

### Lifecycle and locking

- A single `std::mutex` (`m_mutex`) serializes both `Initialize` and
  `Reproject`. `RadarReprojector::reproject()` writes `m_reprojectContext`
  and is not internally thread-safe; rather than retrofit fine-grained
  locking, we wrap the whole RPC body.
- `Initialize` may be called repeatedly. Each successful call replaces the
  existing reprojector (the previous one is destroyed under the lock, so any
  in-flight `Reproject` on the old one finishes first).
- `Reproject` before any successful `Initialize` returns
  `FAILED_PRECONDITION`. `Reproject` with a `radar_param.id` that wasn't
  present in the most-recently-initialized `LogData` returns
  `INVALID_ARGUMENT` — the algorithm itself doesn't validate this; we
  pre-check via `m_radarParams`.
- Server shutdown: `MainServer.cpp` installs a `SIGINT` / `SIGTERM` handler
  that calls `grpc::Server::Shutdown(deadline)` — see §6.

### Concurrency consequences

Single-mutex serialization means throughput is bounded by one core. That is
acceptable for the research / bench use case the user described. If
multi-client load ever becomes a concern, the algorithm itself would need
re-entrancy work; we don't paper over it here.

## 5. Conversion layer

`server/Conversions.{hpp,cpp}` is a pure translation unit between
`namespace radar_reprojection::proto` and
`namespace radar_reprojection::algorithm`. No gRPC types, no I/O. The header
declares free functions, two per type pair, named `fromProto` / `toProto`:

```cpp
algorithm::ReprojectorParam fromProto(const proto::ReprojectorParam& p);
algorithm::LogData          fromProto(const proto::LogData& p);
algorithm::ReprojectInput   fromProto(const proto::ReprojectRequest& p);

proto::RadarScan            toProto(const algorithm::RadarScan& s);
// ... etc.
```

### Geometry: quaternion ↔ Affine3f

```cpp
Eigen::Affine3f fromProto(const proto::Pose& p) {
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    t.translate(Eigen::Vector3f{p.translation().x(), p.translation().y(), p.translation().z()});
    t.rotate(Eigen::Quaternionf{p.rotation().w(), p.rotation().x(), p.rotation().y(), p.rotation().z()});
    return t;
}

proto::Pose toProto(const Eigen::Affine3f& a) {
    Eigen::Quaternionf q{a.linear()};               // assumes pure rotation
    proto::Pose p;
    p.mutable_translation()->set_x(a.translation().x());
    p.mutable_translation()->set_y(a.translation().y());
    p.mutable_translation()->set_z(a.translation().z());
    p.mutable_rotation()->set_x(q.x());
    p.mutable_rotation()->set_y(q.y());
    p.mutable_rotation()->set_z(q.z());
    p.mutable_rotation()->set_w(q.w());
    return p;
}
```

Note `Eigen::Quaternionf` constructor order is `(w, x, y, z)`, while the
wire layout puts `w` last for convention. The conversion makes that
crossover explicit in one place.

### `from_world` is recomputed, not trusted

In `fromProto(const proto::ActorState&)`, the `from_world` field on the wire
is **ignored**. We compute `state.fromWorld = state.toWorld.inverse()`
locally. Rationale: `from_world` and `to_world` must be exact inverses; a
client that desynchronizes them would silently corrupt downstream geometry.

`toProto(const algorithm::ActorState&)` writes both fields out for
round-trip fidelity (used in unit tests).

### Maps and string keys

`proto::LogData::log_radar_data` and `LogActorData::actor_to_trajectory` are
`map<string, …>`; the algorithm uses `std::map<std::string, …>`. Direct
copy.

`proto::ReprojectRequest::actor_to_state` → `std::map<std::string,
ActorState>` (the algorithm uses `std::map`, not `unordered_map`).

## 6. Server entry point (`MainServer.cpp`)

CLI flags (parsed by hand — no third-party flag library):

| Flag                | Default               | Effect                                                          |
| ------------------- | --------------------- | --------------------------------------------------------------- |
| `--address`         | `0.0.0.0:50051`       | Listen address. Passed to `ServerBuilder::AddListeningPort`.    |
| `--max-message-mb`  | `256`                 | Bumps both send and receive message size limits.                |
| `--num-threads`     | (unset → grpc default)| `ServerBuilder::SetSyncServerOption(NUM_CQS, …)` if specified.  |
| `--help`            | n/a                   | Print usage and exit 0.                                         |

Startup sequence:
1. Parse flags. On unknown flag → print usage to stderr, exit 1.
2. Construct `RadarReprojectionServer`.
3. Build `grpc::Server` with the size limits and listener.
4. Install `SIGINT` / `SIGTERM` handlers that call
   `server->Shutdown(deadline = now + 10s)`.
5. `server->Wait()`.
6. On shutdown signal, log "shutting down" and return 0.

Logging: reuse `algorithm/Logger.hpp` (`DW_LOGI` macros) for consistency
with `algorithm_demo`. No new logging dependency.

## 7. Python client (smoke test only)

`examples/python/smoke_client.py`:

- Connects to `localhost:50051` (or `--address` flag).
- Builds a minimal `LogData` in-memory: one radar with one synthetic scan,
  one ego trajectory point.
- Calls `Initialize`. Asserts no `RpcError`.
- Calls `Reproject` at the recording timestamp. Prints
  `len(response.scan.detections)`.
- Exits 0 on success, non-zero on any exception.

This is **not** a usable client library — its only job is to prove the
generated stubs link correctly and the wire format is sound. A full Python
client wrapping `PhysicalAIAVLoader`-equivalent loading is explicitly out of
scope for this iteration (locked decision).

`grpcio` Python dependency is provided through `rules_python` and a
`requirements.txt` file resolved by `pip.parse`. The implementation plan
will pin a specific version compatible with the C++ grpc module.

## 8. Error model

All errors travel as gRPC `Status` codes (locked decision):

| Condition                                             | Status code            |
| ----------------------------------------------------- | ---------------------- |
| Reproject called before any successful Initialize     | `FAILED_PRECONDITION`  |
| Reproject with a `radar_param.id` not in current LogData | `INVALID_ARGUMENT`  |
| `RadarReprojector::initialize()` returns `false`      | `INVALID_ARGUMENT`     |
| Algorithm or conversion code throws an exception      | `INTERNAL`             |
| Any other malformed input caught at conversion        | `INVALID_ARGUMENT`     |

`Status::error_message` carries a human-readable description. No structured
error details (`google.rpc.Status` payloads) — the existing API doesn't
distinguish enough error subtypes to justify them.

`InitializeResponse` and `ReprojectResponse` therefore carry only the
success payload. Concretely:

- `InitializeResponse` becomes an empty message (kept as a named type for
  future extensibility — adding fields later is wire-compatible).
- `ReprojectResponse` keeps `RadarScan scan = 1`.

The proto file must be updated to drop `success` and `error_message` from
`InitializeResponse` before code generation. This is the only proto change
arising from this design.

## 9. Test plan

### Unit tests

`server/test/Conversions_test.cpp` (gtest):

- Round-trip every type pair: build a populated algorithm struct, convert
  to proto, convert back, assert deep equality (with float tolerance).
- Quaternion edge cases: identity, 180° rotation about each axis, a
  composed rotation. Check that `fromWorld` recomputation produces an
  inverse to within tolerance.
- `LogData` with empty containers (no actors, no scans) round-trips
  without crashing.

`server/test/RadarReprojectionServer_test.cpp` (gtest, in-process):

- Spin up the service via `grpc::ServerBuilder` with an in-process channel.
- `Reproject` before `Initialize` → `FAILED_PRECONDITION`.
- `Initialize` with synthetic minimal `LogData`, then `Reproject` returns
  a scan (count of detections may be zero — we are not validating the
  algorithm here, only the plumbing).
- `Reproject` with unknown radar id → `INVALID_ARGUMENT`.
- Re-`Initialize` replaces state (second `Reproject` uses the new data).

### Integration

`examples/python/smoke_client.py` is invoked manually against a running
`radar_reprojection_server`. It is **not** wired into `bazel test` — its
purpose is to verify the Python codegen path end-to-end during
implementation.

### Out of scope

- Algorithm correctness — already covered by the existing
  `algorithm_demo` workflow and the (separate) Python evaluation harness.
- Performance / load testing.

## 10. Migration & cleanup

- Delete `RadarReprojectionManager.{hpp,cpp}` and remove the
  `radar_reprojection_manager` `cc_library` from `BUILD.bazel`.
- Delete `dataset/test/RadarReprojectionManager_test.cpp` and remove its
  `cc_test` from `dataset/BUILD.bazel`.
- Update the comment in `dataset/RadarScan.hpp` that references
  `RadarReprojectionManager::getRadarScan` to point at the gRPC API
  instead, or remove the type entirely if it has no other consumer (the
  implementation plan must verify this).
- Update `README.md`: replace the RRM build/test section with a server
  quick-start (`bazel run //server:radar_reprojection_server`).
- The `dataset/PhysicalAIAVLoader` library and the rest of `dataset/`
  remain in the repo unchanged. They are retained for future client-side
  tooling but are no longer linked into any binary produced by this work.

## 11. Risks & open issues

- **`grpc` BCR module + `nlohmann_json 3.11.3` compatibility.** The grpc
  BCR module pulls a newer `protobuf`, which may transitively conflict
  with `nlohmann_json`'s pin or with `googletest`. Implementation plan
  must run `bazel mod graph` early and adjust versions if needed.
- **Eigen + protobuf header order.** Mixing protobuf-generated headers and
  Eigen in the same TU sometimes triggers warnings under
  `-Wno-error=maybe-uninitialized`. Keep conversion code in `.cpp` and
  the header light to limit blast radius.
- **256 MB max message.** A pathologically large clip might exceed even
  this. The bound is documented in `MainServer.cpp` and configurable via
  `--max-message-mb`; we don't try to be clever beyond that.
- **`dataset/RadarScan.hpp` orphan status.** The struct only had RRM as a
  consumer. Its disposition (delete vs. keep) is deferred to the
  implementation plan, which has more context once the deletes are
  staged.
- **No TLS.** The server listens in cleartext. Acceptable for a localhost
  research tool; production deployment is out of scope.

## 12. Decisions locked during brainstorming

For traceability:

1. Server is a pure algorithm wrapper. No dataset coupling; no parquet
   enrichment fields.
2. `Initialize` is unary; bump server/client max message size to 256 MB.
3. Single global reprojector serialized by one mutex.
4. Errors via gRPC `Status` codes only. No body-level success flags.
5. Python deliverable is generated stubs + a smoke-test script.
6. Server config via CLI flags.
7. Bazel: canonical bzlmod path with `grpc`, `protobuf`, `rules_proto`,
   `rules_python`. No `rules_proto_grpc`.
8. Module layout: dedicated `server/` subdirectory with split
   conversions, server class, and main entry.
9. `from_world` recomputed server-side from `to_world.inverse()`.
