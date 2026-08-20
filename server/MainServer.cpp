// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>

#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

#include "server/RadarReprojectionServer.hpp"

namespace {

std::atomic<bool> g_shutdownRequested{false};

void onSignal(int /*sig*/) {
    // Only async-signal-safe ops here — no mutex acquire, no notify_one.
    // The watcher thread polls the flag with a short timeout below.
    g_shutdownRequested.store(true, std::memory_order_relaxed);
}

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0
        << " [--address=HOST:PORT] [--max-message-mb=N] [--num-threads=N]\n"
        << "  --address         default 0.0.0.0:50051\n"
        << "  --max-message-mb  default 256 (max 2047)\n"
        << "  --num-threads     sets grpc MAX_POLLERS; default unset (grpc default)\n";
}

bool parseInt(std::string_view s, int& out) {
    try { out = std::stoi(std::string{s}); return true; }
    catch (...) { return false; }
}

} // namespace

int main(int argc, char** argv) {
    std::string address     = "0.0.0.0:50051";
    int         maxMessageMb = 256;
    int         numThreads   = -1;

    for (int i = 1; i < argc; ++i) {
        std::string_view a = argv[i];
        if (a == "--help" || a == "-h") { printUsage(argv[0]); return 0; }
        constexpr std::string_view kAddr = "--address=";
        constexpr std::string_view kMmb  = "--max-message-mb=";
        constexpr std::string_view kNt   = "--num-threads=";
        if (a.starts_with(kAddr))      { address = std::string{a.substr(kAddr.size())}; continue; }
        if (a.starts_with(kMmb)) {
            if (!parseInt(a.substr(kMmb.size()), maxMessageMb)) {
                std::cerr << "bad --max-message-mb\n"; return 1;
            }
            if (maxMessageMb < 1 || maxMessageMb > 2047) {
                std::cerr << "--max-message-mb out of range [1, 2047]\n"; return 1;
            }
            continue;
        }
        if (a.starts_with(kNt)) {
            if (!parseInt(a.substr(kNt.size()), numThreads)) {
                std::cerr << "bad --num-threads\n"; return 1;
            }
            continue;
        }
        std::cerr << "unknown flag: " << a << "\n"; printUsage(argv[0]); return 1;
    }

    radar_reprojection::server::RadarReprojectionServer service;

    grpc::ServerBuilder builder;
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    builder.SetMaxReceiveMessageSize(maxMessageMb << 20);
    builder.SetMaxSendMessageSize(maxMessageMb << 20);
    if (numThreads > 0) {
        // MAX_POLLERS bounds the number of polling/worker threads, which is
        // closer to the user-visible "number of threads" semantics than NUM_CQS.
        builder.SetSyncServerOption(grpc::ServerBuilder::SyncServerOption::MAX_POLLERS,
                                    numThreads);
    }

    auto server = builder.BuildAndStart();
    if (!server) {
        std::cerr << "failed to start server on " << address << "\n";
        return 1;
    }
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    // Watcher polls the atomic flag at 100ms — shutdown latency is the
    // poll interval. Calling Shutdown() from a normal thread context
    // avoids the absl mutex-in-signal-handler deadlock.
    std::thread shutdownWatcher([&server]() {
        using namespace std::chrono_literals;
        while (!g_shutdownRequested.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(100ms);
        }
        server->Shutdown(std::chrono::system_clock::now() + std::chrono::seconds(10));
    });

    std::cerr << "RadarReprojectionServer listening on " << address
              << " (max-message-mb=" << maxMessageMb << ")\n";
    server->Wait();
    shutdownWatcher.join();
    std::cerr << "RadarReprojectionServer shut down cleanly\n";
    return 0;
}
