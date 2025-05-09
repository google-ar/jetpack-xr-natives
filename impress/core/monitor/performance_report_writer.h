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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_PERFORMANCE_REPORT_WRITER_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_PERFORMANCE_REPORT_WRITER_H_

#include "logs/proto/vr/arcore/impress/impress_log.pb.h"
#include "absl/status/statusor.h"
#include "core/monitor/monitor.h"

namespace imp {
// Returns a Report generated from accumulated Measurements.
// Populates a Report will all the taken measurements.
// Report is a proto2 protobuf.
absl::StatusOr<logs::proto::vr::arcore::impress::PerformanceReport>
CreatePerformanceReport(Monitor& monitor);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_PERFORMANCE_REPORT_WRITER_H_
