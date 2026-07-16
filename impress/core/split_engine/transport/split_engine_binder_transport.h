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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_BINDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_BINDER_H_

#include "core/split_engine/android/extensions/split_engine_bridge.h"
#include "core/split_engine/transport/basic_transport_shmem.h"

namespace imp::split_engine {

// Android Split Engine transport.
//
// SplitEngineBinderTransport is full specialization of BasicTransport.
//
// It relies on MessageGroupMonitor to maintain the list of _really_ active
// message groups (see `EnqueueMessageGroup`, `ReleaseMessageGroup`).
//
using SplitEngineBinderTransport =
    BasicSharedMemoryTransport<SplitEngineBridge>;

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_BASIC_TRANSPORT_BINDER_H_
