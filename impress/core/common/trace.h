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

#include "core/common/trace_details.h"
#include "core/config.h"
#include "filament/libs/utils/include/utils/Systrace.h"

// IMP_TRACE() is a macro that will emit a systrace event with the name of the
// calling function. Should be called at the very beginning of a function.
//
// Example:
//    void Foo() { IMP_TRACE(); }
//
// IMP_TRACE_TEMPLATED() is a macro that will emit a systrace event with the
// name of the calling function and the provided types. This is like IMP_TRACE()
// but for templated functions.
//
// Example:
//    template <typename T, typename U>
//    void Foo() { IMP_TRACE_TEMPLATED(T, U); }
//
// IMP_TRACE_BLOCK() is a macro that will emit a systrace event with the name
// of the calling function and will be active for the duration of the block.
// Useful if you want to trace lambdas within a function.
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
// IMP_TRACE_NAME() is a macro that will emit a systrace event with the name
// provided instead of the name of the function.
//
// Example:
//    void Foo() { IMP_TRACE_NAME("CustomTraceName"); }
//
// IMP_TRACE_NAME_TEMPLATED() is a macro that will emit a systrace event with
// the name provided and the provided types.
//
// Example:
//    template <typename T, typename U>
//    void Foo() { IMP_TRACE_NAME_TEMPLATED("CustomTraceName", T, U); }

// IMP_TRACE() et al only emit code if systrace is enabled
#if defined(SYSTRACE_TAG) && SYSTRACE_TAG
#define IMP_TRACE() IMP_TRACE_PRIVATE()
#define IMP_TRACE_TEMPLATED(types...) IMP_TRACE_PRIVATE_TEMPLATED(types)
#define IMP_TRACE_BLOCK(name) IMP_TRACE_PRIVATE_BLOCK(name)
#define IMP_TRACE_NAME(name) IMP_TRACE_PRIVATE_NAME(name)
#define IMP_TRACE_NAME_TEMPLATED(name, types...) \
  IMP_TRACE_PRIVATE_NAME_TEMPLATED(name, types)
#else  // defined(SYSTRACE_TAG) && SYSTRACE_TAG
#define IMP_TRACE()
#define IMP_TRACE_TEMPLATED(types...)
#define IMP_TRACE_BLOCK(name)
#define IMP_TRACE_NAME(name)
#define IMP_TRACE_NAME_TEMPLATED(name, types...)
#endif  // defined(SYSTRACE_TAG) && SYSTRACE_TAG

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TRACE_H_
