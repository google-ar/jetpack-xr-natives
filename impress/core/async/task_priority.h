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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_PRIORITY_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_PRIORITY_H_

namespace imp {

/*
 * A default set of task priority values for use with Futures and Impress
 * Executors.
 *
 * Prefer scheduling most Futures with kNormalTaskPriority priority.
 *
 * Use kHighTaskPriority when logic is especially user-visible and/or
 * user-blocking. Conversely, use kLowTaskPriority when logic is especially
 * not so.
 */
constexpr int kMinimumTaskPriority = 0;
constexpr int kLowTaskPriority = 24;
constexpr int kNormalTaskPriority = 49;
constexpr int kHighTaskPriority = 74;
constexpr int kMaximumTaskPriority = 99;

constexpr int kNumTaskPriorities = 100;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_PRIORITY_H_
