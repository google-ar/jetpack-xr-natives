// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_SPLIT_ENGINE_GRPC_TRANSPORT_SM_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_SPLIT_ENGINE_GRPC_TRANSPORT_SM_H_

#include "core/split_engine/desktop/split_engine_desktop_bridge_client.h"
#include "core/split_engine/transport/basic_transport_shmem.h"

namespace imp::split_engine {

// Single Machine Desktop Split Engine gRPC Transport.
//
// SplitEngineGrpcTransportSm is full specialization of BasicTransport.
// It relies on MessageGroupMonitor to maintain the list of active message
// groups.
using SplitEngineGrpcTransportSm =
    BasicSharedMemoryTransport<SplitEngineDesktopBridgeClient>;

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_SPLIT_ENGINE_GRPC_TRANSPORT_SM_H_
