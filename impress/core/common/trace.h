/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_H_

#include <stdint.h>

#include "core/config.h"

// IMP_TRACE_USE_PERFETTO can be defined by adding:
//   --//third_party/impress/core:imp_trace_use_perfetto=True
// to your `blaze build`. Note that Perfetto does not support WASM; see
// (broken link) for details.
#if IMP_TRACE_USE_PERFETTO

#include "third_party/perfetto/include/perfetto/tracing/backend_type.h"
#include "third_party/perfetto/include/perfetto/tracing/string_helpers.h"
#include "third_party/perfetto/include/perfetto/tracing/tracing.h"
#include "third_party/perfetto/include/perfetto/tracing/track.h"
#include "third_party/perfetto/include/perfetto/tracing/track_event.h"
#include "third_party/perfetto/include/perfetto/tracing/track_event_category_registry.h"

#define IMP_TRACE_CATEGORY "Impress"

namespace imp_perfetto_tracing {
void InitializePerfetto();
}  // namespace imp_perfetto_tracing

PERFETTO_DEFINE_CATEGORIES_IN_NAMESPACE(imp_perfetto_tracing,
                                        perfetto::Category(IMP_TRACE_CATEGORY));

PERFETTO_USE_CATEGORIES_FROM_NAMESPACE(imp_perfetto_tracing);

#define IMP_TRACE_PRIVATE_IMPL(name) \
  TRACE_EVENT(IMP_TRACE_CATEGORY, perfetto::StaticString{name})

#define IMP_TRACE_INIT() imp_perfetto_tracing::InitializePerfetto();
#define IMP_TRACE_ENABLED 1
#else  // IMP_TRACE_USE_PERFETTO
#include "filament/libs/utils/include/utils/Systrace.h"
#define IMP_TRACE_PRIVATE_IMPL(name) SYSTRACE_NAME(name)
#define IMP_TRACE_ENABLED (defined(SYSTRACE_TAG) && SYSTRACE_TAG)
#endif  // IMP_TRACE_USE_PERFETTO

// Define Perfetto-specific symbols to empty when unused
#ifndef IMP_TRACE_INIT
#define IMP_TRACE_INIT()
#endif  // IMP_TRACE_INIT

#include "core/common/trace_details.h"

// IMP_TRACE() is a macro that will emit a systrace/perfetto event with the name
// of the calling function. Should be called at the very beginning of a
// function.
//
// Example:
//    void Foo() { IMP_TRACE(); }
//
// IMP_TRACE_TEMPLATED() is a macro that will emit a systrace/perfetto event
// with the name of the calling function and the provided types. This is like
// IMP_TRACE() but for templated functions.
// TODO: (broken link) - IMP_TRACE_TEMPLATED() can have nonzero cost when used.
// Avoid if possible until this bug is resolved.
//
// Example:
//    template <typename T, typename U>
//    void Foo() { IMP_TRACE_TEMPLATED(T, U); }
//
// IMP_TRACE_BLOCK() is a macro that will emit a systrace/perfetto event with
// the name of the calling function and will be active for the duration of the
// block. Useful if you want to trace lambdas within a function.
//
// Example:
//    void Foo() {
//      IMP_TRACE();
//      auto foo = []() {
//        IMP_TRACE_BLOCK("MyLambda");
//      };
//      foo();
//    }
//
// IMP_TRACE_NAME() is a macro that will emit a systrace/perfetto event with the
// name provided instead of the name of the function.
//
// Example:
//    void Foo() { IMP_TRACE_NAME("CustomTraceName"); }
//
// IMP_TRACE_NAME_TEMPLATED() is a macro that will emit a systrace/perfetto
// event with the name provided and the provided types.
//
// Example:
//    template <typename T, typename U>
//    void Foo() { IMP_TRACE_NAME_TEMPLATED("CustomTraceName", T, U); }

// IMP_TRACE() et al only emit code if systrace or perfetto is enabled
#if IMP_TRACE_ENABLED
#define IMP_TRACE() IMP_TRACE_PRIVATE()
#define IMP_TRACE_TEMPLATED(types...) IMP_TRACE_PRIVATE_TEMPLATED(types)
#define IMP_TRACE_BLOCK(name) IMP_TRACE_PRIVATE_BLOCK(name)
#define IMP_TRACE_NAME(name) IMP_TRACE_PRIVATE_NAME(name)
#define IMP_TRACE_NAME_TEMPLATED(name, types...) \
  IMP_TRACE_PRIVATE_NAME_TEMPLATED(name, types)
#else  // IMP_TRACE_ENABLED
#define IMP_TRACE()
#define IMP_TRACE_TEMPLATED(types...)
#define IMP_TRACE_BLOCK(name)
#define IMP_TRACE_NAME(name)
#define IMP_TRACE_NAME_TEMPLATED(name, types...)
#endif  // IMP_TRACE_ENABLED

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_H_
