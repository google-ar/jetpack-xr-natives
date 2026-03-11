// Copyright 2024 Google LLC
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

#include "core/common/trace.h"

#if IMP_TRACE_USE_PERFETTO

#include "perfetto/tracing/backend_type.h"
#include "perfetto/tracing/string_helpers.h"
#include "perfetto/tracing/tracing.h"
#include "perfetto/tracing/track.h"
#include "perfetto/tracing/track_event.h"
#include "perfetto/tracing/track_event_category_registry.h"

PERFETTO_TRACK_EVENT_STATIC_STORAGE_IN_NAMESPACE(imp_perfetto_tracing);

void imp_perfetto_tracing::InitializePerfetto() {
  perfetto::TracingInitArgs args = {};
  args.backends = perfetto::BackendType::kSystemBackend;
  perfetto::Tracing::Initialize(args);
  imp_perfetto_tracing::TrackEvent::Register();
}

#endif  // IMP_TRACE_USE_PERFETTO
