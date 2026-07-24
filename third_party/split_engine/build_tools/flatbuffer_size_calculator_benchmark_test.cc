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

#include "testing/base/public/benchmark.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/string.h"
#include "split_engine/build_tools/test_data/test_schema_generated.h"
#include "split_engine/build_tools/test_data/test_schema_size_calculator_generated.h"
#include "split_engine/flatbuffer_size_calculator.h"

namespace split_engine {
namespace {
using ::split_engine::testing::EstimateSize;
using ::split_engine::testing::RootTableT;
using ::split_engine::testing::StringValueT;
using ::split_engine::testing::SubTableT;
using ::split_engine::testing::TestEnum;
using ::split_engine::testing::TestStruct;
using ::split_engine::testing::TestUnion;

// Benchmark measuring the performance of manual FlatbufferSizeCalculator
// simulation.
void BM_FlatbufferSizeCalculatorManual(benchmark::State& state) {
  absl::string_view s_str("hello, flatbuffers testing!");
  absl::string_view union_str("union string test");
  absl::string_view str1("string1");
  absl::string_view str2("string2");
  absl::string_view str3("string3");
  TestStruct st(1.0f, 2.0f, std::array<int32_t, 2>{3, 5});

  for (auto _ : state) {
    FlatbufferSizeCalculator calc;

    // 1. Sub-table 'sub'
    calc.StartTable();
    calc.AddElement<uint32_t>(4, 999, 0);
    auto sub_off = calc.EndTable<void>();

    // 2. Union 'u' (StringValue table)
    auto union_str_off = calc.CreateString(union_str);
    calc.StartTable();
    calc.AddOffset(4, union_str_off);
    auto union_off = calc.EndTable<void>();

    // 3. Vector 'v_int'
    auto v_int_off =
        calc.CreateVector<int32_t>(5, sizeof(int32_t), alignof(int32_t));

    // 4. Vector 'v_str'
    calc.CreateString(str1);
    calc.CreateString(str2);
    calc.CreateString(str3);
    auto v_str_off = calc.CreateOffsetVector<flatbuffers::String>(3);

    // 5. Vector 'v_st'
    auto v_st_off = calc.CreateVector<TestStruct>(2, sizeof(TestStruct),
                                                  alignof(TestStruct));

    // 6. Vector 'v_sub'
    calc.StartTable();
    calc.AddElement<uint32_t>(4, 111, 0);
    calc.EndTable<void>();

    calc.StartTable();
    calc.AddElement<uint32_t>(4, 222, 0);
    calc.EndTable<void>();

    auto v_sub_off = calc.CreateOffsetVector<void>(2);

    // 6.5 Vector 'v_union'
    calc.StartTable();
    calc.AddElement<uint32_t>(4, 333, 0);
    auto sub_u_off = calc.EndTable<void>();

    auto str_u_off = calc.CreateString("vec union str");
    calc.StartTable();
    calc.AddOffset(4, str_u_off);
    auto str_u_table_off = calc.EndTable<void>();

    auto v_union_off = calc.CreateOffsetVector<void>(2);
    auto v_union_type_off =
        calc.CreateVector<TestUnion>(2, sizeof(TestUnion), alignof(TestUnion));
    (void)sub_u_off;
    (void)str_u_table_off;

    // 7. RootTable
    calc.StartTable();
    calc.AddOffset(30, v_union_off);
    calc.AddOffset(28, v_union_type_off);
    calc.AddOffset(26, v_sub_off);
    calc.AddOffset(24, v_st_off);
    calc.AddOffset(22, v_str_off);
    calc.AddOffset(20, v_int_off);
    calc.AddOffset(18, union_off);
    calc.AddElement<uint8_t>(
        16,
        static_cast<uint8_t>(::split_engine::testing::TestUnion::StringValue),
        0);
    calc.AddOffset(14, sub_off);
    calc.AddStruct(12, &st);
    calc.AddOffset(10, calc.CreateString(s_str));
    calc.AddElement<uint8_t>(
        8, static_cast<uint8_t>(::split_engine::testing::TestEnum::ValueB), 0);
    calc.AddElement<int32_t>(6, 12345, 0);
    calc.AddElement<uint8_t>(4, 1, 0);

    auto root = calc.EndTable<void>();
    calc.Finish(root);
    size_t estimated_size = calc.ComputeSize();
    benchmark::DoNotOptimize(estimated_size);
  }
}
BENCHMARK(BM_FlatbufferSizeCalculatorManual);

// Benchmark measuring the performance of generated EstimateSize code on a
// populated Object API tree.
void BM_FlatbufferSizeCalculatorGenerated(benchmark::State& state) {
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

  for (auto _ : state) {
    size_t estimated_size = EstimateSize(root);
    benchmark::DoNotOptimize(estimated_size);
  }
}
BENCHMARK(BM_FlatbufferSizeCalculatorGenerated);

void BM_FlatbufferSizeCalculatorGeneratedPooled(benchmark::State& state) {
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

  FlatbufferSizeCalculator calc;
  for (auto _ : state) {
    size_t estimated_size = EstimateSize(calc, root);
    benchmark::DoNotOptimize(estimated_size);
  }
}
BENCHMARK(BM_FlatbufferSizeCalculatorGeneratedPooled);

}  // namespace
}  // namespace split_engine
