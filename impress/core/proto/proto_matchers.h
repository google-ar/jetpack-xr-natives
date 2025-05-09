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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_MATCHERS_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_MATCHERS_H_

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/proto/proto_matcher_utils.h"
#include "json/writer.h"

namespace imp {
namespace testing {
namespace proto {

using ::testing::Matcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;

template <typename T>
class ProtoEqualsMatcher : public MatcherInterface<T> {
 public:
  explicit ProtoEqualsMatcher(const T& expected_proto, double epsilon)
      : expected_json_(
            matcher_utils::ToDebugJson<T>(const_cast<T&>(expected_proto))),
        epsilon_(epsilon) {}

  bool MatchAndExplain(T arg_proto,
                       MatchResultListener* listener) const override {
    Json::Value arg_json = matcher_utils::ToDebugJson(arg_proto);

    // Print the Actual as human-readable output. By default, imp protos get
    // printed as binary data and we don't have a good way to override that in
    // the generated proto code without pulling in json dependencies.
    // Unfortunately, MatcherInterface doesn't have a way to override how it
    // prints the actual output.
    *listener << "Printed Actual: " << JsonToFormattedString(arg_json) << ", ";

    return CompareAndExplainJson(arg_json, expected_json_, listener);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is same as " << JsonToFormattedString(expected_json_);
  }

  void DescribeNegationTo(std::ostream* os) const override {
    *os << "is not same as " << JsonToFormattedString(expected_json_);
  }

 private:
  bool CompareAndExplainJson(const Json::Value& arg,
                             const Json::Value& expected,
                             MatchResultListener* listener,
                             std::string field_path = "") const {
    if (arg.type() != expected.type()) {
      // Realistically, this should never happen since arg and expected were
      // both generated from the same type of proto.
      *listener << "Field " << field_path
                << " has mismatched types. Arg's type=" << arg.type()
                << ", Expected's type=" << expected.type() << ". ";
      return false;
    }

    bool result = true;

    if (arg.isObject()) {
      Json::Value::Members expected_members = expected.getMemberNames();
      for (auto& member_name : expected_members) {
        if (arg.isMember(member_name)) {
          result = result && CompareAndExplainJson(
                                 arg[member_name], expected[member_name],
                                 listener, field_path + "::" + member_name);
        } else {
          *listener << " Expected has field named [" << member_name
                    << "] which is not found in Arg";
          if (!field_path.empty()) {
            *listener << " in Field " << field_path;
          }
          *listener << ". ";
          result = false;
        }
      }

      // Find members in arg that are not in expected.
      Json::Value::Members arg_members = arg.getMemberNames();
      std::sort(arg_members.begin(), arg_members.end());
      std::sort(expected_members.begin(), expected_members.end());
      Json::Value::Members missing_members;
      std::set_difference(arg_members.begin(), arg_members.end(),
                          expected_members.begin(), expected_members.end(),
                          std::back_inserter(missing_members));
      if (!missing_members.empty()) {
        result = false;
        for (auto& missing_member_name : missing_members) {
          *listener << " Arg has field named [" << missing_member_name
                    << "] which is not found in Expected";
          if (!field_path.empty()) {
            *listener << " in Field " << field_path;
          }
          *listener << ". ";
        }
      }

    } else if (arg.isArray()) {
      if (expected.size() <= arg.size()) {
        for (int i = 0; i < arg.size(); i++) {
          result = result &&
                   CompareAndExplainJson(arg[i], expected[i], listener,
                                         field_path + "::" + std::to_string(i));
        }
      }

      int size_diff = expected.size() - arg.size();
      // Root is never an array, field_path always set.
      if (size_diff != 0) {
        *listener << "Size of Field " << field_path
                  << " does not match. Arg's Size=" << arg.size()
                  << " Expected's size=" << expected.size();
        result = false;
      }
    } else if (expected.isNumeric() && arg.isNumeric()) {
      if (abs(arg.asDouble() - expected.asDouble()) > epsilon_) {
        *listener << "Field " << field_path
                  << " does not match, Arg=" << JsonToFormattedString(arg)
                  << ", Expected=" << JsonToFormattedString(expected);
        result = false;
      }
    } else {
      if (arg != expected) {
        result = false;
        // field_path will always be set here, this case is never the root.
        *listener << "Field " << field_path
                  << " does not match, Arg=" << JsonToFormattedString(arg)
                  << ", Expected=" << JsonToFormattedString(expected);
      }
    }

    return result;
  }

  std::string JsonToFormattedString(const Json::Value& json_value) const {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    return Json::writeString(builder, json_value);
  }

  Json::Value expected_json_;

  double epsilon_;
};

// Registers a type of proto so that the ProtoEquals matcher is able to print
// the fields even if the proto is packed as an Any inside of another proto.
// Without doing this, an Any will print it's content as raw binary data.
//
// This is useful to get human-readable errors in tests that expect protos
// that are packed inside of Any's.
//
// Registration is necessary because imp protos do not support reflection and
// information about field names is not included in the proto wire format.
//
// Note: It would be possible to remove the need for registration by adding
// code-gen that does the registration as part static initialization. However,
// that would require the proto code-gen to take dependencies on debugging
// tools that we want to avoid leading to proto binary bloat.
template <typename T>
void RegisterKnownProtoType() {
  matcher_utils::DebugJsonWriter::RegisterKnownType<T>();
}

// Matcher for comparing two impress protobuf. Supports printing the contents
// of the protobuf in a human-readable way, and printing exactly what was
// different between the protobuf.
//
// Use RegisterKnownProtoType<T>(); to print Any's inside of messages in a
// human readable way.
template <typename T>
Matcher<const T&> ProtoEquals(const T& expected_proto) {
  return Matcher<const T&>(new ProtoEqualsMatcher<const T&>(expected_proto, 0));
}

template <typename T>
Matcher<const T&> ProtoApproximatelyEquals(const T& expected_proto,
                                           double epsilon) {
  return Matcher<const T&>(
      new ProtoEqualsMatcher<const T&>(expected_proto, epsilon));
}

}  // namespace proto
}  // namespace testing
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_MATCHERS_H_
