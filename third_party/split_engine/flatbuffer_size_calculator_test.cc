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

#include "split_engine/flatbuffer_size_calculator.h"

#include <cstdint>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "flatbuffers/base.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/table.h"
#include "flatbuffers/vector.h"

namespace split_engine {
namespace {

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfString) {
  flatbuffers::FlatBufferBuilder fbb;
  auto str_off = fbb.CreateString("hello, flatbuffers!");
  fbb.Finish(str_off);

  FlatbufferSizeCalculator calc;
  auto calc_str = calc.CreateString("hello, flatbuffers!");
  calc.Finish(calc_str);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfVector) {
  uint32_t values[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  flatbuffers::FlatBufferBuilder fbb;
  auto vec_off = fbb.CreateVector(values, 10);
  fbb.Finish(vec_off);

  FlatbufferSizeCalculator calc;
  auto calc_vec = calc.CreateVector<uint32_t>(10, sizeof(uint32_t), 4);
  calc.Finish(calc_vec);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfVectorInferred) {
  uint32_t values[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  flatbuffers::FlatBufferBuilder fbb;
  auto vec_off = fbb.CreateVector(values, 10);
  fbb.Finish(vec_off);

  FlatbufferSizeCalculator calc;
  auto calc_vec = calc.CreateVector<uint32_t>(10);
  calc.Finish(calc_vec);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfTable) {
  flatbuffers::FlatBufferBuilder fbb;
  auto str_off = fbb.CreateString("test_string");
  auto start = fbb.StartTable();
  fbb.AddElement<int32_t>(4, 42, 0);
  fbb.AddOffset(6, str_off);
  fbb.AddElement<uint64_t>(8, 123456789ULL, 0);
  auto table_off = fbb.EndTable(start);
  fbb.Finish(flatbuffers::Offset<flatbuffers::Table>(table_off));

  FlatbufferSizeCalculator calc;
  auto calc_str = calc.CreateString("test_string");
  calc.StartTable();
  calc.AddElement<int32_t>(4, 42, 0);
  calc.AddOffset(6, calc_str);
  calc.AddElement<uint64_t>(8, 123456789ULL, 0);
  auto calc_table = calc.EndTable<void>();
  calc.Finish(calc_table);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfEmptyTable) {
  flatbuffers::FlatBufferBuilder fbb;
  auto start = fbb.StartTable();
  auto table_off = fbb.EndTable(start);
  fbb.Finish(flatbuffers::Offset<flatbuffers::Table>(table_off));

  FlatbufferSizeCalculator calc;
  calc.StartTable();
  auto calc_table = calc.EndTable<void>();
  calc.Finish(calc_table);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfAlignedVectorAfterString) {
  uint64_t values[] = {1, 2, 3, 4};
  flatbuffers::FlatBufferBuilder fbb;
  auto str_off = fbb.CreateString("short");
  fbb.ForceVectorAlignment(4, sizeof(uint64_t), 8);
  auto vec_off = fbb.CreateVector(values, 4);
  auto start = fbb.StartTable();
  fbb.AddOffset(4, str_off);
  fbb.AddOffset(6, vec_off);
  auto table_off = fbb.EndTable(start);
  fbb.Finish(flatbuffers::Offset<flatbuffers::Table>(table_off));

  FlatbufferSizeCalculator calc;
  auto calc_str = calc.CreateString("short");
  auto calc_vec = calc.CreateVector<uint64_t>(4, sizeof(uint64_t), 8);
  calc.StartTable();
  calc.AddOffset(4, calc_str);
  calc.AddOffset(6, calc_vec);
  auto calc_table = calc.EndTable<void>();
  calc.Finish(calc_table);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfDeviousVector) {
  uint8_t values[] = {1, 2, 3};
  flatbuffers::FlatBufferBuilder fbb;
  fbb.PushElement<uint8_t>(42);  // Make GetSize() == 1
  fbb.ForceVectorAlignment(3, sizeof(uint8_t), 4);
  auto vec_off = fbb.CreateVector(values, 3);
  fbb.Finish(flatbuffers::Offset<flatbuffers::Vector<uint8_t>>(vec_off));

  FlatbufferSizeCalculator calc;
  calc.AddElement<uint8_t>(0, 42, 0);  // Make offset_ == 1
  auto calc_vec = calc.CreateVector<uint8_t>(3, sizeof(uint8_t), 4);
  calc.Finish(calc_vec);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) TestStruct FLATBUFFERS_FINAL_CLASS {
 private:
  int32_t a_;
  float b_;

 public:
  TestStruct(int32_t _a, float _b)
      : a_(flatbuffers::EndianScalar(_a)), b_(flatbuffers::EndianScalar(_b)) {}
  int32_t a() const { return flatbuffers::EndianScalar(a_); }
  float b() const { return flatbuffers::EndianScalar(b_); }
};
FLATBUFFERS_STRUCT_END(TestStruct, 8);

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfStruct) {
  TestStruct test_struct(42, 3.14f);
  flatbuffers::FlatBufferBuilder fbb;
  auto struct_off = fbb.CreateStruct(test_struct);
  fbb.Finish(struct_off);

  FlatbufferSizeCalculator calc;
  auto calc_struct = calc.CreateStruct(test_struct);
  calc.Finish(calc_struct);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorTest, ComputeSizeOfVectorOfStructs) {
  TestStruct test_structs[] = {TestStruct(1, 1.0f), TestStruct(2, 2.0f),
                               TestStruct(3, 3.0f)};
  flatbuffers::FlatBufferBuilder fbb;
  auto vec_off = fbb.CreateVectorOfStructs(test_structs, 3);
  fbb.Finish(vec_off);

  FlatbufferSizeCalculator calc;
  auto calc_vec = calc.CreateVectorOfStructs<TestStruct>(3);
  calc.Finish(calc_vec);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

TEST(FlatbufferSizeCalculatorTest, ClearResetsState) {
  FlatbufferSizeCalculator calc;

  // Perform some simulations.
  auto str_off = calc.CreateString("hello, flatbuffers!");
  calc.StartTable();
  calc.AddElement<int32_t>(4, 42, 0);
  calc.AddOffset(6, str_off);
  auto table_off = calc.EndTable<void>();
  calc.Finish(table_off);

  EXPECT_GT(calc.ComputeSize(), 0);

  // Clear the state.
  calc.Clear();

  EXPECT_EQ(calc.ComputeSize(), 0);

  // Verify we can reuse the calculator after Clear and get the expected size.
  flatbuffers::FlatBufferBuilder fbb;
  auto fbb_str_off = fbb.CreateString("hello, flatbuffers!");
  fbb.Finish(fbb_str_off);

  auto calc_str = calc.CreateString("hello, flatbuffers!");
  calc.Finish(calc_str);

  EXPECT_EQ(calc.ComputeSize(), fbb.GetSize());
}

}  // namespace
}  // namespace split_engine
