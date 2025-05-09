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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_NODE_CONVERTERS_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_NODE_CONVERTERS_H_

#include <functional>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/assets/gltf/behavior/converted_graph.h"
#include "core/model/model_data.h"

namespace imp::gltf::behavior {

constexpr absl::string_view kDefaultOutputValueSocket = "value";
constexpr absl::string_view kDefaultOutputFlowSocket = "out";
constexpr absl::string_view kDefaultOutputAsyncDoneSocket = "done";
constexpr absl::string_view kDeltaTimeValueSocket = "timeSinceLastTick";
constexpr absl::string_view kElapsedTimeValueSocket = "timeSinceStart";

// Function to convert a behavior node to recipe nodes.
//
// If the conversion fails, an error will be returned. If it succeeds, a vector
// of RecipeNode will be returned.
using NodeConverterFunction = std::function<absl::Status(
    const model::ModelData::BehaviorData::NodeData&, ConvertedGraph&)>;

// BehaviorNodeConverter contains information on how behavior nodes will be
// converted into recipe nodes.
struct NodeConverter {
  absl::string_view node_type;
  NodeConverterFunction function;
};

// Converter for "lifecycle/onStart" nodes.
NodeConverter GetLifeCycleOnStartConverter();

// Converter for "lifecycle/onTick" nodes.
NodeConverter GetLifeCycleOnTickConverter();

// Converter for "debug/consoleLog" nodes
NodeConverter GetDebugConsoleConverter();

// Converter for "variable/set" nodes.
NodeConverter GetVariableSetConverter();

// Converter for "variable/get" nodes.
NodeConverter GetVariableGetConverter();

// Converter for "world/startAnimation" nodes.
NodeConverter GetWorldStartAnimationConverter();

// Converter for "world/stopAnimation" nodes.
NodeConverter GetWorldStopAnimationConverter();

// Converter for "node/OnSelect" nodes.
NodeConverter GetNodeOnSelectConverter();

// Converter for "customEvent/receive" nodes.
NodeConverter GetCustomEventReceiveConverter();

// Converter for "customEvent/send" nodes.
NodeConverter GetCustomEventSendConverter();

// Converter for "world/get" nodes.
NodeConverter GetWorldGetConverter();

// Converter for "world/set" nodes.
NodeConverter GetWorldSetConverter();

// Converter for "world/animateTo" nodes.
NodeConverter GetWorldAnimateToConverter();

// Converter for "math/pi" node.
NodeConverter GetMathPiConverter();

// Converter for "math/add" - add nodes.
NodeConverter GetMathAddConverter();

// Converter for "math/sub" - subtract nodes.
NodeConverter GetMathSubConverter();

// Converter for "math/mul" multiply nodes.
NodeConverter GetMathMulConverter();

// Converter for "math/div" - divide nodes.
NodeConverter GetMathDivConverter();

// Converter for "math/rem" - remainder nodes.
NodeConverter GetMathRemConverter();

// Converter for "math/min" - min of two values.
NodeConverter GetMathMinConverter();

// Converter for "math/max" - max of two values.
NodeConverter GetMathMaxConverter();

// Converter for "math/eq" - equal to nodes.
NodeConverter GetMathEqConverter();

// Converter for "math/lt" - less than nodes.
NodeConverter GetMathLtConverter();

// Converter for "math/le" - less than or equal to nodes.
NodeConverter GetMathLeConverter();

// Converter for "math/gt" - greater than nodes.
NodeConverter GetMathGtConverter();

// Converter for "math/ge" - greater or equal to nodes.
NodeConverter GetMathGeConverter();

// Converter for "math/dot" - dot product of two float vectors.
NodeConverter GetMathDotConverter();

// Converter for "math/cross" - cross product of two float3 vectors.
NodeConverter GetMathCrossConverter();

// Converter for "math/clamp" - clamps the input between min and max, inclusive.
NodeConverter GetMathClampConverter();

// Convertor for "math/abs" - absolute value of the input value.
NodeConverter GetMathAbsConverter();

// Converter for "math/sqrt" - returns square root of input value.
NodeConverter GetMathSqrtConverter();

// Converter for "math/log" - returns natural logarithm of input value.
NodeConverter GetMathLogConverter();

// Convertor for "math/sin" - sin of input. Must be in radians.
NodeConverter GetMathSinConverter();

// Convertor for "math/cos" - cos of input. Must be in radians.
NodeConverter GetMathCosConverter();

// Convertor for "math/tan" - tan of input. Must be in radians.
NodeConverter GetMathTanConverter();

// Convertor for "math/asin" - asin of input. Must be in radians.
NodeConverter GetMathAsinConverter();

// Convertor for "math/acos" - acos of input. Must be in radians.
NodeConverter GetMathAcosConverter();

// Convertor for "math/atan" - atan of input. Must be in radians.
NodeConverter GetMathAtanConverter();

// Convertor for "math/atan2" - atan of x and y axis values.
NodeConverter GetMathAtanTwoConverter();

// Converter for "math/sign" - sign of input.
NodeConverter GetMathSignConverter();

// Converter for "math/normalize" - normalized value of input.
NodeConverter GetMathNormalizeConverter();

// Converter for type nodes that casts from one type to another.
NodeConverter GetTypeCastBoolToIntConverter();
NodeConverter GetTypeCastBoolToFloatConverter();
NodeConverter GetTypeCastIntToBoolConverter();
NodeConverter GetTypeCastIntToFloatConverter();
NodeConverter GetTypeCastFloatToBoolConverter();
NodeConverter GetTypeCastFloatToIntConverter();

// Converter for "math/makeVector" - combines float inputs into a float vector.
NodeConverter GetMathMakeVector2Converter();
NodeConverter GetMathMakeVector3Converter();
NodeConverter GetMathMakeVector4Converter();

// Converter for "math/breakVector" - breaks float vector into separate floats.
NodeConverter GetMathBreakVector2Converter();
NodeConverter GetMathBreakVector3Converter();
NodeConverter GetMathBreakVector4Converter();

// Converter for "math/compose" -
// takes in position, rotation, scale, outputs a transformation matrix.
NodeConverter GetMathComposeConverter();

// Converter for "math/decompose" -
// takes in a transformation matrix, outputs position, rotation and scale.
NodeConverter GetMathDecomposeConverter();

// Converter for "math/inverse", inverts the matrix passed to it.
NodeConverter GetMathInverseConverter();

// Converter for "math/matmul" - multiplies two matrices passed to it.
NodeConverter GetMathMatMulConverter();

// Converter for "flow/forLoop" nodes.
NodeConverter GetFlowForLoopConverter();

// Converter for "flow/sequence" nodes.
NodeConverter GetFlowSequenceConverter();

// Converter for "flow/delay" nodes.
NodeConverter GetFlowDelayConverter();

// Converter for "flow/branch" nodes.
NodeConverter GetFlowBranchConverter();

// Converter for "flow/stopAudio" nodes.
NodeConverter GetFlowStopAudioConverter();

// Converter for "async/playSound" nodes.
NodeConverter GetAsyncPlaySoundConverter();

}  // namespace imp::gltf::behavior

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_BEHAVIOR_NODE_CONVERTERS_H_
