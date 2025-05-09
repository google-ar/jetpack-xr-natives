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

#ifndef THIRD_PARTY_ARCORE_AR_SCENEFORM_LOADER_CONFIG_H_
#define THIRD_PARTY_ARCORE_AR_SCENEFORM_LOADER_CONFIG_H_

// expect LOADER_API_CONFIG=OPENGL or
// LOADER_API_CONFIG=METAL or
// LOADER_API_CONFIG=VULKAN
#define LOADER_API_PRIVATE_DEFINITION_OPENGL() 0
#define LOADER_API_PRIVATE_DEFINITION_METAL() 1
#define LOADER_API_PRIVATE_DEFINITION_VULKAN() 2

#define LOADER_API_XSMASH(X, Y) X##Y()
#define LOADER_API_SMASH(X, Y) LOADER_API_XSMASH(X, Y)
#define LOADER_API(X)                     \
  (LOADER_API_PRIVATE_DEFINITION_##X() == \
   LOADER_API_SMASH(LOADER_API_PRIVATE_DEFINITION_, LOADER_API_CONFIG))

#endif  // THIRD_PARTY_ARCORE_AR_SCENEFORM_LOADER_CONFIG_H_
