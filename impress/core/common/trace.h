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

#if IMP_RUNTIME(DEV) && !IMP_TRACE_USE_PERFETTO
#include "core/performance/profiler.h"
#include "core/performance/timer.h"

#define IMP_TRACE_ENABLED 1
#define IMP_TRACE_IMPL_PROFILER(name) imp::Timer timer##__LINE__(name);
#define IMP_TRACE_COUNTER_IMPL_PROFILER(name, value)
#define IMP_PROFILE_START_FRAME() imp::Profiler::AdvanceFrame();
#define IMP_PROFILE_REGISTER_THREAD(name) imp::Profiler::SetThreadName(name);
#else  // IMP_RUNTIME(DEV)
#define IMP_PROFILE_START_FRAME()
#define IMP_TRACE_IMPL_PROFILER(name)
#define IMP_TRACE_COUNTER_IMPL_PROFILER(name, value)
#define IMP_PROFILE_REGISTER_THREAD(name)
#endif  // IMP_RUNTIME(DEV)

// The generated trace categories file establishes the legal category names and
// defines the macros that conditionally dispatch if those categories are
// included in the build.
#include "core/common/trace_categories.h"

// Establish the default trace category.
#ifndef IMP_TRACE_DEFAULT_CATEGORY
#define IMP_TRACE_DEFAULT_CATEGORY default
#endif

// IMP_TRACE_USE_PERFETTO can be defined by adding:
//   --//third_party/impress/core:imp_trace_use_perfetto=True
// to your `blaze build`. Note that Perfetto does not support WASM; see
// (broken link) for details.
#if IMP_TRACE_USE_PERFETTO

#if defined(SYSTRACE_TAG) && SYSTRACE_TAG
#error "Cannot enable both SYSTRACE and Perfetto tracing."
#endif

#include "perfetto/tracing/backend_type.h"
#include "perfetto/tracing/string_helpers.h"
#include "perfetto/tracing/tracing.h"
#include "perfetto/tracing/track.h"
#include "perfetto/tracing/track_event.h"
#include "perfetto/tracing/track_event_category_registry.h"


namespace imp_perfetto_tracing {
void InitializePerfetto();
}  // namespace imp_perfetto_tracing

PERFETTO_DEFINE_CATEGORIES_IN_NAMESPACE(imp_perfetto_tracing,
                                        IMP_PERFETTO_CATEGORIES);

PERFETTO_USE_CATEGORIES_FROM_NAMESPACE(imp_perfetto_tracing);

#define IMP_TRACE_STR(x) #x
#define IMP_TRACE_TO_STR(x) IMP_TRACE_STR(x)

#define IMP_TRACE_IMPL_PERFETTO(name, cat) \
  TRACE_EVENT("impress." IMP_TRACE_TO_STR(cat), perfetto::StaticString{name})

#define IMP_TRACE_COUNTER_IMPL_PERFETTO(name, value, cat) \
  TRACE_COUNTER("impress." IMP_TRACE_TO_STR(cat),         \
                perfetto::StaticString{name}, value)

#define IMP_TRACE_INIT() imp_perfetto_tracing::InitializePerfetto();

#ifndef IMP_TRACE_ENABLED
#define IMP_TRACE_ENABLED 1
#endif  // IMP_TRACE_ENABLED

#define IMP_TRACE_IMPL_SYSTRACE(name)
#define IMP_TRACE_COUNTER_IMPL_SYSTRACE(name, value)
#else  // IMP_TRACE_USE_PERFETTO
#include "filament/libs/utils/include/utils/Systrace.h"
#define IMP_TRACE_IMPL_SYSTRACE(name) SYSTRACE_NAME(name)
#define IMP_TRACE_COUNTER_IMPL_SYSTRACE(name, value) \
  SYSTRACE_VALUE32(name, value)

#ifndef IMP_TRACE_ENABLED
#if defined(SYSTRACE_TAG) && SYSTRACE_TAG
#define IMP_TRACE_ENABLED 1
#else
#define IMP_TRACE_ENABLED 0
#endif
#endif  // IMP_TRACE_ENABLED

#define IMP_TRACE_IMPL_PERFETTO(name, cat)
#define IMP_TRACE_COUNTER_IMPL_PERFETTO(name, value, cat)
#endif  // IMP_TRACE_USE_PERFETTO

#define IMP_TRACE_PRIVATE_IMPL(name, cat) \
  IMP_TRACE_IMPL_PERFETTO(name, cat);     \
  IMP_TRACE_IMPL_SYSTRACE(name);          \
  IMP_TRACE_IMPL_PROFILER(name);

#define IMP_TRACE_PRIVATE_COUNTER_IMPL(name, value, cat) \
  IMP_TRACE_COUNTER_IMPL_PERFETTO(name, value, cat);     \
  IMP_TRACE_COUNTER_IMPL_SYSTRACE(name, value);          \
  IMP_TRACE_COUNTER_IMPL_PROFILER(name, value);

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
//    void Bar() { IMP_TRACE_CATEGORY(client); }
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
//    template <typename T, typename U>
//    void Bar() { IMP_TRACE_TEMPLATED_CATEGORY(client, T, U); }
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
//    void Bar() { IMP_TRACE_CATEGORY_NAME(client, "CustomTraceName"); }
//
// IMP_TRACE_NAME_TEMPLATED() is a macro that will emit a systrace/perfetto
// event with the name provided and the provided types.
//
// Example:
//    template <typename T, typename U>
//    void Foo() { IMP_TRACE_NAME_TEMPLATED("CustomTraceName", T, U); }
//
// IMP_TRACE_COUNTER() is a macro that records a numerical counter update.
//
// Example:
//    void Foo() { IMP_TRACE_COUNTER("MyCounter", 10); }
//    void Bar() { IMP_TRACE_CATEGORY_COUNTER(client, "MyCounter", 10); }

// IMP_TRACE() et al only emit code if systrace or perfetto is enabled
#if IMP_TRACE_ENABLED
#define IMP_TRACE_CATEGORY(cat) IMP_TRACE_PRIVATE(cat)
#define IMP_TRACE_TEMPLATED_CATEGORY(cat, types...) \
  IMP_TRACE_PRIVATE_TEMPLATED(cat, types)
#define IMP_TRACE_CATEGORY_BLOCK(cat, name) IMP_TRACE_PRIVATE_BLOCK(cat, name)
#define IMP_TRACE_CATEGORY_NAME(cat, name) IMP_TRACE_PRIVATE_NAME(cat, name)
#define IMP_TRACE_NAME_TEMPLATED_CATEGORY(cat, name, types...) \
  IMP_TRACE_PRIVATE_NAME_TEMPLATED(cat, name, types)
#define IMP_TRACE_CATEGORY_COUNTER(cat, name, value) \
  IMP_TRACE_PRIVATE_COUNTER(cat, name, value)

#define IMP_TRACE() IMP_TRACE_CATEGORY(IMP_TRACE_DEFAULT_CATEGORY)
#define IMP_TRACE_TEMPLATED(types...) \
  IMP_TRACE_TEMPLATED_CATEGORY(IMP_TRACE_DEFAULT_CATEGORY, types)
#define IMP_TRACE_BLOCK(name) \
  IMP_TRACE_CATEGORY_BLOCK(IMP_TRACE_DEFAULT_CATEGORY, name)
#define IMP_TRACE_NAME(name) \
  IMP_TRACE_CATEGORY_NAME(IMP_TRACE_DEFAULT_CATEGORY, name)
#define IMP_TRACE_NAME_TEMPLATED(name, types...) \
  IMP_TRACE_NAME_TEMPLATED_CATEGORY(IMP_TRACE_DEFAULT_CATEGORY, name, types)
#define IMP_TRACE_COUNTER(name, value) \
  IMP_TRACE_CATEGORY_COUNTER(IMP_TRACE_DEFAULT_CATEGORY, name, value)
#else  // IMP_TRACE_ENABLED
#define IMP_TRACE_CATEGORY(cat)
#define IMP_TRACE_TEMPLATED_CATEGORY(cat, types...)
#define IMP_TRACE_CATEGORY_BLOCK(cat, name)
#define IMP_TRACE_CATEGORY_NAME(cat, name)
#define IMP_TRACE_NAME_TEMPLATED_CATEGORY(cat, name, types...)
#define IMP_TRACE_CATEGORY_COUNTER(cat, name, value)

#define IMP_TRACE()
#define IMP_TRACE_TEMPLATED(types...)
#define IMP_TRACE_BLOCK(name)
#define IMP_TRACE_NAME(name)
#define IMP_TRACE_NAME_TEMPLATED(name, types...)
#define IMP_TRACE_COUNTER(name, value)
#endif  // IMP_TRACE_ENABLED

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_H_
