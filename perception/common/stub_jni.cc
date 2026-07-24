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

#include <jni.h>

extern "C" {

// Queries whether the testing stub library has been loaded, allowing
// non-OpenXR devices to emulate OpenXR support during AOSP device tests.
//
// In test environments, the testing stub library is loaded before Jetpack XR
// dynamically loads its runtime. A successful call to this method signals JXR
// to override runtime selection and use the OpenXR emulation stub.
//
// Although this method returns a boolean, the primary signal is the
// presence of the function itself. It is defined only in the testing stub
// library and intentionally left unimplemented in real production builds.
JNIEXPORT jboolean JNICALL
Java_androidx_xr_runtime_ServiceLoaderExtKt_nativeIsStub(JNIEnv* /*env*/,
                                                         jclass /*clazz*/) {
  return true;
}

}  // extern "C"
