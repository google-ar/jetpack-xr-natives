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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_CORE_POINTERS_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_CORE_POINTERS_H_

#include <memory>

#include "third_party/arcore/ar/core/c_api/arcore_c_api.h"

namespace imp {
namespace ar {

// These classes are basically auto pointers for the ARCore native types.
// This lets us hold onto references to sessions, frames, etc. created from
// the ARCore c API and they will be automatically deleted when their owning
// class gets deleted.

class ArSessionDeleter {
 public:
  ArSessionDeleter() noexcept {}
  void operator()(ArSession_* p) { ArSession_destroy(p); }
};
using UniqueArSession = std::unique_ptr<ArSession_, ArSessionDeleter>;

class ArConfigDeleter {
 public:
  ArConfigDeleter() noexcept {}
  void operator()(ArConfig_* p) { ArConfig_destroy(p); }
};
using UniqueArConfig = std::unique_ptr<ArConfig_, ArConfigDeleter>;

class ArRecordingConfigDeleter {
 public:
  ArRecordingConfigDeleter() noexcept {}
  void operator()(ArRecordingConfig_* p) { ArRecordingConfig_destroy(p); }
};
using UniqueArRecordingConfig =
    std::unique_ptr<ArRecordingConfig_, ArRecordingConfigDeleter>;

class ArFrameDeleter {
 public:
  ArFrameDeleter() noexcept {}
  void operator()(ArFrame_* p) { ArFrame_destroy(p); }
};
using UniqueArFrame = std::unique_ptr<ArFrame_, ArFrameDeleter>;

class ArImageDeleter {
 public:
  ArImageDeleter() noexcept {}
  void operator()(ArImage_* p) { ArImage_release(p); }
};
using UniqueArImage = std::unique_ptr<ArImage_, ArImageDeleter>;

class ArPoseDeleter {
 public:
  ArPoseDeleter() noexcept {}
  void operator()(ArPose_* p) { ArPose_destroy(p); }
};
using UniqueArPose = std::unique_ptr<ArPose_, ArPoseDeleter>;

class ArCameraDeleter {
 public:
  ArCameraDeleter() noexcept {}
  void operator()(ArCamera_* p) { ArCamera_release(p); }
};
using UniqueArCamera = std::unique_ptr<ArCamera_, ArCameraDeleter>;

class ArCameraIntrinsicsDeleter {
 public:
  ArCameraIntrinsicsDeleter() noexcept {}
  void operator()(ArCameraIntrinsics_* p) { ArCameraIntrinsics_destroy(p); }
};
using UniqueArCameraIntrinsics =
    std::unique_ptr<ArCameraIntrinsics_, ArCameraIntrinsicsDeleter>;

class ArTrackableDeleter {
 public:
  ArTrackableDeleter() noexcept {}
  void operator()(ArTrackable_* p) { ArTrackable_release(p); }
};
using UniqueArTrackable = std::unique_ptr<ArTrackable_, ArTrackableDeleter>;

class ArPlaneDeleter {
 public:
  ArPlaneDeleter() noexcept {}
  void operator()(ArPlane_* p) { ArTrackable_release(ArAsTrackable(p)); }
};
using UniqueArPlane = std::unique_ptr<ArPlane_, ArPlaneDeleter>;

class ArPointDeleter {
 public:
  ArPointDeleter() noexcept {}
  void operator()(ArPoint_* p) { ArTrackable_release(ArAsTrackable(p)); }
};
using UniqueArPoint = std::unique_ptr<ArPoint_, ArPointDeleter>;

class ArLightEstimateDeleter {
 public:
  ArLightEstimateDeleter() noexcept {}
  void operator()(ArLightEstimate_* p) { ArLightEstimate_destroy(p); }
};
using UniqueArLightEstimate =
    std::unique_ptr<ArLightEstimate_, ArLightEstimateDeleter>;

class ArHitResultDeleter {
 public:
  ArHitResultDeleter() noexcept {}
  void operator()(ArHitResult_* p) { ArHitResult_destroy(p); }
};
using UniqueArHitResult = std::unique_ptr<ArHitResult_, ArHitResultDeleter>;

// A hit result pointer that automatically creates the underlying resource.
class ArHitResultPtr {
 public:
  explicit ArHitResultPtr(ArSession_* ar_session) {
    ArHitResult_* ar_hit_result;
    ArHitResult_create(ar_session, &ar_hit_result);
    hit_result_ = UniqueArHitResult(ar_hit_result);
  }
  ArHitResult_* get() const { return hit_result_.get(); }
  ArHitResult_* operator->() const { return get(); }
  explicit operator bool() { return get() != nullptr; }

 private:
  UniqueArHitResult hit_result_;
};

class ArAnchorDeleter {
 public:
  ArAnchorDeleter() noexcept {}
  void operator()(ArAnchor_* p) { ArAnchor_release(p); }
};
using UniqueArAnchor = std::unique_ptr<ArAnchor_, ArAnchorDeleter>;

class ArHitResultListDeleter {
 public:
  ArHitResultListDeleter() noexcept {}
  void operator()(ArHitResultList_* p) { ArHitResultList_destroy(p); }
};
using UniqueArHitResultList =
    std::unique_ptr<ArHitResultList_, ArHitResultListDeleter>;

// A hit result list pointer that automatically creates the underlying resource.
class ArHitResultListPtr {
 public:
  explicit ArHitResultListPtr(ArSession_* ar_session_) {
    ArHitResultList_* ar_hit_result_list;
    ArHitResultList_create(ar_session_, &ar_hit_result_list);
    hit_result_list_ = UniqueArHitResultList(ar_hit_result_list);
  }
  ArHitResultList_* get() const { return hit_result_list_.get(); }
  ArHitResultList_* operator->() const { return get(); }
  explicit operator bool() { return get() != nullptr; }

 private:
  UniqueArHitResultList hit_result_list_;
};

class ArTrackableListDeleter {
 public:
  ArTrackableListDeleter() noexcept {}
  void operator()(ArTrackableList_* p) { ArTrackableList_destroy(p); }
};
using UniqueArTrackableList =
    std::unique_ptr<ArTrackableList_, ArTrackableListDeleter>;

// Helper class for operating on a trackable list.
class ArTrackableListHelper {
 public:
  explicit ArTrackableListHelper(ArSession_* ar_session)
      : ar_session_(ar_session) {
    // TODO Reduce memory allocations here.
    ArTrackableList_* trackable_list;
    ArTrackableList_create(ar_session, &trackable_list);
    unique_trackable_list_ = UniqueArTrackableList(trackable_list);
  }

  int32_t GetSize() const {
    int32_t size = 0;
    ArTrackableList_getSize(ar_session_, unique_trackable_list_.get(), &size);
    return size;
  }

  ArTrackableList_* get() const { return unique_trackable_list_.get(); }

  explicit operator ArTrackableList_*() { return get(); }

  UniqueArTrackableList Move() {
    return UniqueArTrackableList(unique_trackable_list_.release());
  }

  template <typename FN>
  void ForEach(FN callback) const {
    int list_size = GetSize();
    ArTrackable_* trackable = nullptr;
    for (int i = 0; i < list_size; ++i) {
      ArTrackableList_acquireItem(ar_session_, get(), i, &trackable);
      UniqueArTrackable trackable_ptr(trackable);
      callback(std::move(trackable_ptr));
    }
  }

 private:
  UniqueArTrackableList unique_trackable_list_;
  ArSession_* ar_session_;
};

class ArAnchorListDeleter {
 public:
  ArAnchorListDeleter() noexcept {}
  void operator()(ArAnchorList_* p) { ArAnchorList_destroy(p); }
};
using UniqueArAnchorList = std::unique_ptr<ArAnchorList_, ArAnchorListDeleter>;

// Helper class for operating on an anchor list.
// TODO: Refactor ListHelpers to avoid code duplication.
class ArAnchorListHelper {
 public:
  explicit ArAnchorListHelper(ArSession_* ar_session)
      : ar_session_(ar_session) {
    // TODO Reduce memory allocations here.
    ArAnchorList_* anchor_list;
    ArAnchorList_create(ar_session, &anchor_list);
    unique_anchor_list_ = UniqueArAnchorList(anchor_list);
  }

  int32_t GetSize() const {
    int32_t size = 0;
    ArAnchorList_getSize(ar_session_, unique_anchor_list_.get(), &size);
    return size;
  }

  ArAnchorList_* get() const { return unique_anchor_list_.get(); }

  explicit operator ArAnchorList_*() { return unique_anchor_list_.get(); }

  UniqueArAnchorList Move() {
    return UniqueArAnchorList(unique_anchor_list_.release());
  }

  template <typename FN>
  void ForEach(FN callback) const {
    int list_size = GetSize();
    ArAnchor_* anchor = nullptr;
    for (int i = 0; i < list_size; ++i) {
      ArAnchorList_acquireItem(ar_session_, unique_anchor_list_.get(), i,
                               &anchor);
      callback(UniqueArAnchor(anchor));
    }
  }

 private:
  UniqueArAnchorList unique_anchor_list_;
  ArSession_* ar_session_;
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_AR_CORE_POINTERS_H_
