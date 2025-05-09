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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_DURATION_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_DURATION_UTILS_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "core/proto/duration.proto.imp.h"

namespace imp {
// Encodes an absl::Duration as a google::protobuf::Duration, following the
// encoding rules specified at (broken link).
// Returns an error if the given absl::Duration is beyond the range allowed by
// the protobuf. Otherwise, truncates toward zero with nanosecond precision and
// returns the google::protobuf::Duration.
//
// Note: +/- absl::InfiniteDuration() cannot be encoded because they are not
// representable in the protobuf.
//
// Based on google3/util/time/protoutil.h which is for the GoogleApi.
::absl::Status EncodeImpProto(absl::Duration d,
                              ::google::protobuf::imp_proto::Duration& proto);

// Decodes the given protobuf and returns an absl::Duration, or returns an error
// status if the argument is invalid according to
// (broken link).
//
// Based on google3/util/time/protoutil.h which is for the GoogleApi.
::absl::StatusOr<absl::Duration> DecodeImpProto(
    const ::google::protobuf::imp_proto::Duration& proto);

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_DURATION_UTILS_H_
