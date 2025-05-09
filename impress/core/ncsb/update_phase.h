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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATE_PHASE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATE_PHASE_H_

#include <array>
#include <cstddef>
namespace imp {

// Used to determine which Update methods to call within a particular phase
// while advancing time.
//
// This is used to control the order of updates in a coarse-grained way that
// doesn't require concrete dependencies / ordering.
//
// If fine-grained ordering is needed, then UpdateDependencies and
// UpdateDependents should be used instead.
//
// Update phases occur in the order of this enum.
enum class UpdatePhase {
  // Runs at the start of advancing time.
  // This is before View::Update, the foreground executor, and input runs.
  //
  // Impress does internal work that must be done early in the frame
  // including updating animations in this phase.
  kStart,
  // Runs just before kDefault.
  //
  // This is after View::Update, the foreground executor, and input runs.
  kPreDefault,
  // Runs immediately after kPreDefault in the middle of advancing time. This is
  // when Component's are updated by default.
  kDefault,
  // Runs immediately after kDefault.
  kPostDefault,
  // Runs at the end of advancing time just before rendering begins.
  //
  // Impress does internal work that must be done at the end of the frame
  // including applying skinning in this phase.
  kEnd,
};

static constexpr int kNumUpdatePhases = static_cast<int>(UpdatePhase::kEnd) + 1;

// Helper used to verify at compile time that UpdateDependencies and
// UpdateDependents are all on the within the same phase.
template <std::size_t N>
constexpr bool DoAllPhasesMatch(UpdatePhase to_match,
                                std::array<UpdatePhase, N> phases) {
  if constexpr (N != 0) {
    for (UpdatePhase phase : phases) {
      if (phase != to_match) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_UPDATE_PHASE_H_
