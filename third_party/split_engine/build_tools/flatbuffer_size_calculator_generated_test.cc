// Copyright 2026 Google LLC
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

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "split_engine/build_tools/test_data/test_schema_generated.h"
#include "split_engine/build_tools/test_data/test_schema_size_calculator_generated.h"
#include "split_engine/flatbuffer_size_calculator.h"

namespace split_engine::testing {
namespace {

TEST(FlatbufferSizeCalculatorGeneratedTest, TestRootTableObjectApi) {
  RootTableT root;
  root.b = true;
  root.i = 12345;
  root.e = TestEnum::ValueB;
  root.s = "hello, flatbuffers testing!";

  root.st =
      std::make_unique<TestStruct>(1.0f, 2.0f, std::array<int32_t, 2>{3, 5});

  root.sub = std::make_unique<SubTableT>();
  root.sub->id = 999;

  StringValueT str_union;
  str_union.value = "union string test";
  root.u.Set(std::move(str_union));

  root.v_int = {10, 20, 30, 40, 50};
  root.v_str = {"string1", "string2", "string3"};

  root.v_st.push_back(TestStruct(3.0f, 4.0f, std::array<int32_t, 2>{7, 8}));
  root.v_st.push_back(TestStruct(5.0f, 6.0f, std::array<int32_t, 2>{9, 10}));

  auto sub_vec1 = std::make_unique<SubTableT>();
  sub_vec1->id = 111;
  auto sub_vec2 = std::make_unique<SubTableT>();
  sub_vec2->id = 222;
  root.v_sub.push_back(std::move(sub_vec1));
  root.v_sub.push_back(std::move(sub_vec2));

  root.v_union.resize(2);
  SubTableT sub_u;
  sub_u.id = 333;
  root.v_union[0].Set(std::move(sub_u));

  StringValueT str_u;
  str_u.value = "vec union str";
  root.v_union[1].Set(std::move(str_u));

  flatbuffers::FlatBufferBuilder fbb;
  auto fbb_off = RootTable::Pack(fbb, &root);
  fbb.Finish(fbb_off);

  size_t estimated_size = EstimateSize(root);
  EXPECT_EQ(estimated_size, fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorGeneratedTest, TestRootTableObjectApiPooled) {
  RootTableT root;
  root.b = true;
  root.i = 12345;
  root.e = TestEnum::ValueB;
  root.s = "hello, flatbuffers testing!";

  root.st =
      std::make_unique<TestStruct>(1.0f, 2.0f, std::array<int32_t, 2>{3, 5});

  root.sub = std::make_unique<SubTableT>();
  root.sub->id = 999;

  StringValueT str_union;
  str_union.value = "union string test";
  root.u.Set(std::move(str_union));

  root.v_int = {10, 20, 30, 40, 50};
  root.v_str = {"string1", "string2", "string3"};

  root.v_st.push_back(TestStruct(3.0f, 4.0f, std::array<int32_t, 2>{7, 8}));
  root.v_st.push_back(TestStruct(5.0f, 6.0f, std::array<int32_t, 2>{9, 10}));

  auto sub_vec1 = std::make_unique<SubTableT>();
  sub_vec1->id = 111;
  auto sub_vec2 = std::make_unique<SubTableT>();
  sub_vec2->id = 222;
  root.v_sub.push_back(std::move(sub_vec1));
  root.v_sub.push_back(std::move(sub_vec2));

  root.v_union.resize(2);
  SubTableT sub_u;
  sub_u.id = 333;
  root.v_union[0].Set(std::move(sub_u));

  StringValueT str_u;
  str_u.value = "vec union str";
  root.v_union[1].Set(std::move(str_u));

  flatbuffers::FlatBufferBuilder fbb;
  auto fbb_off = RootTable::Pack(fbb, &root);
  fbb.Finish(fbb_off);

  FlatbufferSizeCalculator calc;
  // Put some dirty data/state in the calculator first to make sure it gets
  // cleared automatically.
  calc.AddElement<int32_t>(0, 12345, 0);
  calc.StartTable();
  calc.EndTable<void>();
  EXPECT_GT(calc.ComputeSize(), 0);

  size_t estimated_size_1 = EstimateSize(calc, root);
  EXPECT_EQ(estimated_size_1, fbb.GetSize());

  // The calculator is dirty from the previous run but should be cleared
  // automatically.
  size_t estimated_size_2 = EstimateSize(calc, root);
  EXPECT_EQ(estimated_size_2, fbb.GetSize());
}

}  // namespace
}  // namespace split_engine::testing
