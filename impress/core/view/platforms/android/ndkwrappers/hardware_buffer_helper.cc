// Copyright 2024 Google LLC
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

#include "core/view/platforms/android/ndkwrappers/hardware_buffer_helper.h"

#include <android/data_space.h>
#include <android/hardware_buffer.h>
#include <dlfcn.h>

#include <cstdint>

#include "absl/cleanup/cleanup.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/common/robin_map.h"
#include "core/render/android/android_defines.h"

#define QC_AUXILIARY_VIEW_MASK 0x02

namespace imp::android {

FPAHardwareBuffer_getAuxiliaryViewInfo
    AHardwareBufferHelper::AHardwareBuffer_getAuxiliaryViewInfo_ = nullptr;
FPAHardwareBuffer_getAuxiliaryBuffer
    AHardwareBufferHelper::AHardwareBuffer_getAuxiliaryBuffer_ = nullptr;
FPAHardwareBuffer_getDataSpace
    AHardwareBufferHelper::AHardwareBuffer_getDataSpace_ = nullptr;

bool AHardwareBufferHelper::mvhevc_enabled_ = false;
bool AHardwareBufferHelper::data_space_enabled_ = false;

std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
AHardwareBufferHelper::GetAvailableViews(AHardwareBuffer* primary_view_ahb) {
  auto view_hardware_buffers =
      absl::WrapUnique(new RobinMap<SurfaceViewType, const AHardwareBuffer*>());
  *view_hardware_buffers = {{SurfaceViewType::kPrimaryView, primary_view_ahb}};
  if (mvhevc_enabled_) {
    uint32_t view_masks =
        AHardwareBuffer_getAuxiliaryViewInfo_(primary_view_ahb);
    if (view_masks & QC_AUXILIARY_VIEW_MASK) {
      // Retrieve the auxiliary view hardware buffer.
      const AHardwareBuffer* auxiliary_buffer =
          reinterpret_cast<const AHardwareBuffer*>(
              AHardwareBuffer_getAuxiliaryBuffer_(primary_view_ahb,
                                                  QC_AUXILIARY_VIEW_MASK));
      if (auxiliary_buffer) {
        (*view_hardware_buffers)[SurfaceViewType::kAuxiliaryView] =
            auxiliary_buffer;
      } else {
        IMP_LOG(imp::ERROR) << "Auxiliary view hardware buffer was expected but is not "
                      "available.";
      }
    }
  }
  return view_hardware_buffers;
}

ADataSpace AHardwareBufferHelper::GetDataSpace(AHardwareBuffer* ahb) {
  if (data_space_enabled_) {
    return AHardwareBuffer_getDataSpace_(ahb);
  }
  return ADATASPACE_UNKNOWN;
}

absl::Status AHardwareBufferHelper::LoadRuntimeLibraries() {
  // Static initializers are guaranteed to be evaluated only once.
  static absl::Status initialized = [] {
    mvhevc_enabled_ = false;
    // Load native window library which is used by the media NDK.
    void* libnativewindow_ptr = dlopen("libnativewindow.so", RTLD_NOW);
    if (!libnativewindow_ptr) {
      return absl::InternalError(
          absl::StrCat("Unable to open libnativewindow.so: ", dlerror()));
    }
    absl::Cleanup libnativewindow_cleanup = [libnativewindow_ptr] {
      dlclose(libnativewindow_ptr);
    };

    AHardwareBuffer_getAuxiliaryViewInfo_ =
        reinterpret_cast<FPAHardwareBuffer_getAuxiliaryViewInfo>(
            dlsym(libnativewindow_ptr,
                  "_ZN7android36AHardwareBuffer_"
                  "getAuxiliaryViewInfoEPK15AHardwareBuffer"));
    AHardwareBuffer_getAuxiliaryBuffer_ =
        reinterpret_cast<FPAHardwareBuffer_getAuxiliaryBuffer>(
            dlsym(libnativewindow_ptr,
                  "_ZN7android34AHardwareBuffer_"
                  "getAuxiliaryBufferEP15AHardwareBufferj"));
    AHardwareBuffer_getDataSpace_ =
        reinterpret_cast<FPAHardwareBuffer_getDataSpace>(dlsym(
            libnativewindow_ptr,
            "_ZN7android28AHardwareBuffer_getDataSpaceEP15AHardwareBuffer"));

    if (!AHardwareBuffer_getAuxiliaryViewInfo_ ||
        !AHardwareBuffer_getAuxiliaryBuffer_) {
      IMP_LOG(imp::WARNING) << "Unable to load the dynamic symbols for MV-HEVC "
                      "playback from libnativewindow.so. MV-HEVC playback will "
                      "not be supported through dlsym. Relying on the "
                      "ImageAPIProvider to provide the required APIs.";
    } else {
      mvhevc_enabled_ = true;
      IMP_LOG(imp::INFO)
          << "MV-HEVC playback API available in libnativewindow.so. This will "
             "be used as a fallback if the ImageAPIProvider is not set.";
    }

    if (!AHardwareBuffer_getDataSpace_) {
      IMP_LOG(imp::WARNING)
          << "Unable to load the dynamic symbols for "
             "AHardwareBuffer_getDataSpace "
             "from libnativewindow.so. Image dataspace extraction will "
             "not be supported through dlsym. Relying on the "
             "ImageAPIProvider to provide the required APIs.";
    } else {
      data_space_enabled_ = true;
      IMP_LOG(imp::INFO) << "AHardwareBuffer_getDataSpace API available in "
                   "libnativewindow.so. This will be used as a fallback if the "
                   "ImageAPIProvider is not set.";
    }

    std::move(libnativewindow_cleanup).Cancel();
    return absl::OkStatus();
  }();
  return initialized;
}

}  // namespace imp::android
