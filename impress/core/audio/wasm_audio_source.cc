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

#include <emscripten/bind.h>
#include <emscripten/emscripten.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/assets/asset_ptr.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/audio/audio_source.h"
#include "core/common/platform_helpers.h"
#include "core/common/resource_helpers.h"
#include "core/media/media_asset.h"
#include "core/media/media_source.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace audio {
namespace {

class WasmAudioSource : public AudioSource {
 public:
  explicit WasmAudioSource(BaseView* view);

  WasmAudioSource(const WasmAudioSource&) = delete;
  WasmAudioSource& operator=(const WasmAudioSource&) = delete;

  ~WasmAudioSource();

  Future<absl::Status> Load(absl::string_view asset_url);
  Future<absl::Status> Load(const MediaAsset* audio_asset) override;
  absl::Status LoadSync(const MediaAsset* audio_asset) override;

  absl::Status Play() override;

  absl::Status Pause() override;

  absl::Status Stop() override;

  absl::Status SetPlaybackSpeed(float speed) override;

  absl::Status SeekTo(float seconds, SeekType seek_type) override;

  absl::Status SetLoopCount(int loop) override;

  absl::Status SetVolume(float volume) override;

  absl::StatusOr<absl::Duration> GetDuration() const override;

  absl::StatusOr<absl::Duration> GetPlaybackTime() const override;

  absl::StatusOr<int> GetLoopCount() const override;

  AudioSource::State GetState() const override;

  void SetOnPlaybackCompleteCallback(std::function<void()> callback) override;

  void SetOnSeekCompleteCallback(std::function<void()> callback) override;

  void SetOnBufferingCallback(
      std::function<void(BufferingState)> callback) override;

  void OnAudioSourceLoaded(bool success);
  void OnPlaybackComplete();

  static WasmAudioSource* FromHandle(intptr_t handle);

 private:
  intptr_t GetHandle() const;

  // Calls the given function on Module['wasmAudioManager'] with the given arg.
  // Note: the double returned here has different meaning depending on the case.
  // In some cases, this call is used as a "getter" to get a value from JS such
  // as the duration of the audio clip in seconds.
  // In other cases, this call returns 1 if the call succeeded (OkStatus) or
  // 0 if the call failed (InternalError).
  double CallWebFunction(absl::string_view function,
                         const std::string& arg) const;
  double CallWebFunction(absl::string_view function, double arg) const;
  double CallWebFunction(absl::string_view function) const;

  // Returns a standard internal error status when a JS error occurs.
  absl::Status GetInternalError() const;

  State state_;
  std::function<void(bool)> on_ready_callback_;
  std::function<void()> on_playback_complete_callback_;
};

// These bindings allow the JS-side of the WASM audio source to call back to
// the native WasmAudioSource object for resolving the initial load and
// notifying that playback has completed.
EMSCRIPTEN_BINDINGS(audio_bindings) {
  function(
      "onAudioSourceLoaded",
      (void (*)(intptr_t, bool))[](intptr_t source, bool success) {
        WasmAudioSource* wasm_audio_source =
            WasmAudioSource::FromHandle(source);
        wasm_audio_source->OnAudioSourceLoaded(success);
      },
      emscripten::allow_raw_pointers());
  function(
      "onAudioSourcePlaybackComplete",
      (void (*)(intptr_t))[](intptr_t source) {
        WasmAudioSource* wasm_audio_source =
            WasmAudioSource::FromHandle(source);
        wasm_audio_source->OnPlaybackComplete();
      },
      emscripten::allow_raw_pointers());
}

inline constexpr absl::string_view kSourceManagerFunctionAddSource =
    "addSource";
inline constexpr absl::string_view kSourceManagerFunctionRemoveSource =
    "removeSource";
inline constexpr absl::string_view kSourceManagerFunctionPlay = "play";
inline constexpr absl::string_view kSourceManagerFunctionPause = "pause";
inline constexpr absl::string_view kSourceManagerFunctionSeek = "seek";
inline constexpr absl::string_view kSourceManagerFunctionStop = "stop";
inline constexpr absl::string_view kSourceManagerFunctionSetLoop = "setLoop";
inline constexpr absl::string_view kSourceManagerFunctionSetVolume =
    "setVolume";
inline constexpr absl::string_view kSourceManagerFunctionGetDuration =
    "getDuration";
inline constexpr absl::string_view kSourceManagerFunctionGetCurrentTime =
    "getCurrentTime";
inline constexpr absl::string_view kSourceManagerFunctionGetLoopCount =
    "getLoopCount";

WasmAudioSource::WasmAudioSource(BaseView* view) : AudioSource(view) {}

WasmAudioSource::~WasmAudioSource() {
  if (!CallWebFunction(kSourceManagerFunctionRemoveSource)) {
    IMP_LOG(imp::ERROR) << "Failed to cleanup audio source!";
  }
}

Future<absl::Status> WasmAudioSource::Load(absl::string_view asset_url) {
  if (asset_url.empty()) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Empty asset_url cannot be loaded!"));
  }

  absl::Status load_failed_error = absl::InternalError(
      "Failed to load audio asset: " + std::string(asset_url));
  Future<absl::Status> on_ready_result;
  on_ready_callback_ = [on_ready_result, load_failed_error](bool success) {
    if (success) {
      on_ready_result.Return(absl::OkStatus());
    } else {
      on_ready_result.Return(load_failed_error);
    }
  };

  if (!CallWebFunction(kSourceManagerFunctionAddSource,
                       std::string(asset_url))) {
    on_ready_result.Return(load_failed_error);
  }

  return on_ready_result;
}

Future<absl::Status> WasmAudioSource::Load(const MediaAsset* audio_asset) {
  return Future<absl::Status>::Schedule(
      [this, audio_asset]() -> absl::Status { return LoadSync(audio_asset); },
      Executor::Type::kForeground);
}

absl::Status WasmAudioSource::LoadSync(const MediaAsset* audio_asset) {
  return absl::UnavailableError("WASM Audio source cannot load Media Assets.");
}

void WasmAudioSource::OnAudioSourceLoaded(bool success) {
  if (on_ready_callback_) {
    on_ready_callback_(success);
    on_ready_callback_ = nullptr;
  }
}

intptr_t WasmAudioSource::GetHandle() const {
  return reinterpret_cast<intptr_t>(this);
}

WasmAudioSource* WasmAudioSource::FromHandle(intptr_t handle) {
  return reinterpret_cast<WasmAudioSource*>(handle);
}

double WasmAudioSource::CallWebFunction(absl::string_view function,
                                        double arg) const {
  // Note: strings passed as pointers to EM_ASM_INT must be null-terminated.
  return MAIN_THREAD_EM_ASM_DOUBLE(
      {
        // This is all Javascript code. Have to convert back to a string from
        // a raw pointer since EM_ASM_INT doesn't share scope with C++ and can
        // only accept int arguments.
        const functionName = UTF8ToString($0);
        const sourceId = $1;
        const arg = $2;
        return Module['wasmAudioManager'][functionName](sourceId, arg);
      },
      std::string(function).c_str(), GetHandle(), arg);
}

double WasmAudioSource::CallWebFunction(absl::string_view function,
                                        const std::string& arg) const {
  // Note: strings passed as pointers to EM_ASM_INT must be null-terminated.
  return MAIN_THREAD_EM_ASM_DOUBLE(
      {
        // This is all Javascript code. Have to convert back to a string from
        // a raw pointer since EM_ASM_INT doesn't share scope with C++ and can
        // only accept int arguments.
        const functionName = UTF8ToString($0);
        const sourceId = $1;
        const arg = UTF8ToString($2);
        return Module['wasmAudioManager'][functionName](sourceId, arg);
      },
      std::string(function).c_str(), GetHandle(), arg.c_str());
}

double WasmAudioSource::CallWebFunction(absl::string_view function) const {
  return CallWebFunction(function, 0);
}

absl::Status WasmAudioSource::GetInternalError() const {
  return absl::InternalError("Error code received from WasmAudioManager JS!");
}

absl::Status WasmAudioSource::Play() {
  if (CallWebFunction(kSourceManagerFunctionPlay)) {
    state_ = State::kPlaying;
    return absl::OkStatus();
  }
  return GetInternalError();
}

absl::Status WasmAudioSource::Pause() {
  if (CallWebFunction(kSourceManagerFunctionPause)) {
    state_ = State::kReady;
    return absl::OkStatus();
  }
  return GetInternalError();
}

absl::Status WasmAudioSource::Stop() {
  if (CallWebFunction(kSourceManagerFunctionStop)) {
    state_ = State::kStopped;
    OnPlaybackComplete();
    return absl::OkStatus();
  }
  return GetInternalError();
}

absl::Status WasmAudioSource::SetPlaybackSpeed(float speed) {
  return absl::UnimplementedError("Not implemented");
}

absl::Status WasmAudioSource::SeekTo(const float seconds, SeekType seek_type) {
  if (CallWebFunction(kSourceManagerFunctionSeek, seconds)) {
    return absl::OkStatus();
  }
  return GetInternalError();
}

absl::Status WasmAudioSource::SetLoopCount(const int loop) {
  if (CallWebFunction(kSourceManagerFunctionSetLoop, loop)) {
    return absl::OkStatus();
  }
  return GetInternalError();
}

absl::Status WasmAudioSource::SetVolume(const float volume) {
  if (CallWebFunction(kSourceManagerFunctionSetVolume, volume)) {
    return absl::OkStatus();
  }
  return GetInternalError();
}

absl::StatusOr<absl::Duration> WasmAudioSource::GetDuration() const {
  return absl::Seconds(CallWebFunction(kSourceManagerFunctionGetDuration));
}

absl::StatusOr<absl::Duration> WasmAudioSource::GetPlaybackTime() const {
  return absl::Seconds(CallWebFunction(kSourceManagerFunctionGetCurrentTime));
}

absl::StatusOr<int> WasmAudioSource::GetLoopCount() const {
  return CallWebFunction(kSourceManagerFunctionGetLoopCount);
}

AudioSource::State WasmAudioSource::GetState() const { return state_; }

void WasmAudioSource::SetOnPlaybackCompleteCallback(
    std::function<void()> callback) {
  on_playback_complete_callback_ = callback;
}

void WasmAudioSource::SetOnSeekCompleteCallback(
    std::function<void()> callback) {
  // Not implemented.
}

void WasmAudioSource::SetOnBufferingCallback(
    std::function<void(BufferingState)> callback) {
  // Not implemented.
}

void WasmAudioSource::OnPlaybackComplete() {
  state_ = AudioSource::State::kStopped;
  if (on_playback_complete_callback_) {
    on_playback_complete_callback_();
  }
}

std::string ConvertMediaAssetToBase64(
    const AssetPtr<media::MediaAsset>& media_asset) {
  std::string encoded =
      SerializeToBase64(media_asset->GetData(), media_asset->GetSize());
  // TODO: Support other kinds of formats besides MP3.
  return absl::StrCat("data:audio/mp3;base64,", encoded);
}

Future<std::string> LoadMediaAssetToBase64(BaseView& base_view,
                                           absl::string_view asset_url) {
  return base_view.GetAssetManager().LoadMedia(asset_url).Then(
      [](const AssetPtr<media::MediaAsset>& media_asset) mutable
      -> absl::StatusOr<std::string> {
        return ConvertMediaAssetToBase64(media_asset);
      });
}

}  // namespace

Future<std::unique_ptr<AudioSource>> CreateAudioSource(
    BaseView& base_view, absl::string_view asset_url) {
  if (asset_url.empty()) {
    return Future<std::unique_ptr<AudioSource>>(
        absl::InvalidArgumentError("Empty asset_url cannot be loaded!"));
  }

  Future<std::string> audio_url_or_base64;
  if (!resources::ResourceManager::IsRemoteUrl(asset_url)) {
    audio_url_or_base64 = LoadMediaAssetToBase64(base_view, asset_url);
  } else {
    audio_url_or_base64 = Future<std::string>(std::string(asset_url));
  }

  auto wasm_audio_source = std::make_unique<WasmAudioSource>(&base_view);
  return audio_url_or_base64.Then(
      [wasm_audio_source = std::move(wasm_audio_source)](
          absl::StatusOr<std::string> audio_url_or) mutable
      -> Future<std::unique_ptr<AudioSource>> {
        if (!audio_url_or.ok()) {
          return Future<std::unique_ptr<AudioSource>>(audio_url_or.status());
        }
        return wasm_audio_source->Load(audio_url_or.value())
            .Then([wasm_audio_source = std::move(wasm_audio_source)](
                      absl::Status status) mutable
                  -> absl::StatusOr<std::unique_ptr<AudioSource>> {
              MP_RETURN_IF_ERROR(status);
              std::unique_ptr<AudioSource> audio_source =
                  std::move(wasm_audio_source);
              return audio_source;
            });
      });
}

Future<std::unique_ptr<AudioSource>> CreateAudioSource(BaseView& base_view,
                                                       absl::Cord content) {
  std::optional<absl::string_view> flattened_string = content.TryFlat();
  Future<AssetPtr<MediaAsset>> media_asset;
  if (flattened_string) {
    media_asset = base_view.GetAssetManager().LoadAsset<MediaAsset>(
        content, /*asset_url=*/"");
  } else {
    media_asset =
        Future<absl::Cord>::Schedule(
            [contents = std::move(content)]() mutable {
              contents.Flatten();
              return std::move(contents);
            },
            Executor::Type::kBackground)
            .Then([&base_view](absl::Cord flattened_cord) mutable {
              return base_view.GetAssetManager().LoadAsset<MediaAsset>(
                  std::move(flattened_cord), /*asset_url=*/"");
            });
  }

  auto wasm_audio_source = std::make_unique<WasmAudioSource>(&base_view);
  return media_asset.Then(
      [wasm_audio_source = std::move(wasm_audio_source)](
          const AssetPtr<media::MediaAsset>& media_asset) mutable
      -> Future<std::unique_ptr<AudioSource>> {
        WasmAudioSource* wasm_audio_source_ptr = wasm_audio_source.get();
        return wasm_audio_source_ptr
            ->Load(ConvertMediaAssetToBase64(media_asset))
            .Then([wasm_audio_source = std::move(wasm_audio_source)](
                      absl::Status status) mutable
                  -> absl::StatusOr<std::unique_ptr<AudioSource>> {
              MP_RETURN_IF_ERROR(status);
              std::unique_ptr<AudioSource> audio_source =
                  std::move(wasm_audio_source);
              return audio_source;
            });
      });
}

}  // namespace audio
}  // namespace imp
