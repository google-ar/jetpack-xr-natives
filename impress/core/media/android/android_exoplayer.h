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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_EXOPLAYER_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_EXOPLAYER_H_

#include <jni.h>

#include <cstdint>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/media/android/android_exoplayer_listener.h"
#include "core/media/media_color_space.h"
#include "core/media/media_source.h"
#include "core/media/media_type.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::media {

class AndroidExoPlayer : public JavaWrapper {
 public:
  explicit AndroidExoPlayer(const Context& context)
      : JavaWrapper(context, "com/google/ar/imp/core/media/ImpExoPlayer",
                    "(Landroid/content/Context;)V",
                    context.GetActivityContext()) {
    release_ = GetMethodHandle("release", "()V");
    set_video_surface_ =
        GetMethodHandle("setVideoSurface", "(Landroid/view/Surface;)V");
    set_media_item_ = GetMethodHandle("setMediaItem", "(Ljava/lang/String;)V");
    set_protected_media_item_ = GetMethodHandle(
        "setProtectedMediaItem",
        "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
    set_listener_ = GetMethodHandle(
        "setListener",
        "(Lcom/google/ar/imp/core/media/ImpExoPlayerListener;)V");
    play_ = GetMethodHandle("play", "()V");
    pause_ = GetMethodHandle("pause", "()V");
    stop_ = GetMethodHandle("stop", "()V");
    prepare_ = GetMethodHandle("prepare", "()V");
    set_playback_speed_ = GetMethodHandle("setPlaybackSpeed", "(F)V");
    set_looping_ = GetMethodHandle("setLooping", "(Z)V");
    seek_to_ = GetMethodHandle("seekTo", "(JI)V");
    seek_quick_ = GetStaticFieldHandle("SEEK_QUICK", "I");
    seek_precise_ = GetStaticFieldHandle("SEEK_PRECISE", "I");
    set_volume_ = GetMethodHandle("setVolume", "(F)V");
    is_playing_ = GetMethodHandle("isPlaying", "()Z");
    is_looping_ = GetMethodHandle("isLooping", "()Z");
    get_duration_ = GetMethodHandle("getDuration", "()J");
    get_current_position_ = GetMethodHandle("getCurrentPosition", "()J");
    get_video_width_ = GetMethodHandle("getVideoWidth", "()I");
    get_video_height_ = GetMethodHandle("getVideoHeight", "()I");
    get_stereo_mode_ = GetMethodHandle("getStereoMode", "()I");

    get_color_standard_ = GetMethodHandle("getColorStandard", "()I");
    get_color_transfer_ = GetMethodHandle("getColorTransfer", "()I");
    get_color_range_ = GetMethodHandle("getColorRange", "()I");
    get_luma_bitdepth_ = GetMethodHandle("getLumaBitDepth", "()I");
    get_chroma_bitdepth_ = GetMethodHandle("getChromaBitDepth", "()I");
    get_max_content_light_level_ =
        GetMethodHandle("getMaxContentLightLevel", "()I");

    buffering_state_ = GetStaticFieldHandle("IMP_STATE_BUFFERING", "I");
    buffering_done_state_ =
        GetStaticFieldHandle("IMP_STATE_BUFFERING_DONE", "I");
  }

  ~AndroidExoPlayer() override { CallVoidMethod(release_); }

  void SetVideoSurface(android::Surface* surface) {
    CallVoidMethod(set_video_surface_, surface->Reference());
  }

  void SetMediaItem(std::string video_url) {
    CallVoidMethod(set_media_item_, ToString(Env(), video_url));
  }

  void SetProtectedMediaItem(std::string video_url, std::string drm_license_url,
                             std::string drm_scheme_uuid) {
    JNIEnv* env = Env();
    jstring video_url_jstring = ToString(env, video_url);
    jstring license_url_jstring = ToString(env, drm_license_url);
    jstring uuid_jstring = ToString(env, drm_scheme_uuid);
    CallVoidMethod(set_protected_media_item_, video_url_jstring,
                   license_url_jstring, uuid_jstring);
  }

  void SetListener(ImpExoPlayerListener* exoplayer_listener) {
    CallVoidMethod(set_listener_, exoplayer_listener->Reference());
  }

  bool Play() {
    CallVoidMethod(play_);
    return !CheckIfException(Env());
  }

  bool Pause() {
    CallVoidMethod(pause_);
    return !CheckIfException(Env());
  }

  bool Stop() {
    CallVoidMethod(stop_);
    return !CheckIfException(Env());
  }

  void Prepare() { CallVoidMethod(prepare_); }

  bool SetPlaybackSpeed(float speed) {
    CallVoidMethod(set_playback_speed_, speed);
    return !CheckIfException(Env());
  }

  bool SetLooping(bool loop) {
    CallVoidMethod(set_looping_, loop);
    return !CheckIfException(Env());
  }

  bool SeekTo(int milliseconds, MediaSource::SeekType seek_type) {
    int mode;
    switch (seek_type) {
      case MediaSource::SeekType::QUICK:
        mode = GetStaticIntField(seek_quick_);
        break;
      case MediaSource::SeekType::PRECISE:
        mode = GetStaticIntField(seek_precise_);
        break;
    }
    CallVoidMethod(seek_to_, static_cast<int64_t>(milliseconds), mode);
    return !CheckIfException(Env());
  }

  bool SetVolume(float volume) {
    CallVoidMethod(set_volume_, volume);
    return !CheckIfException(Env());
  }

  absl::StatusOr<bool> IsPlaying() {
    bool is_playing = CallBooleanMethod(is_playing_);
    if (CheckIfException(Env())) {
      return absl::InternalError("Could not get playing status");
    }
    return is_playing;
  }

  absl::StatusOr<bool> IsLooping() {
    bool is_looping = CallBooleanMethod(is_looping_);
    if (CheckIfException(Env())) {
      return absl::InternalError("Could not get looping status");
    }
    return is_looping;
  }

  absl::StatusOr<int> GetDuration() {
    int result = CallIntMethod(get_duration_);
    if (CheckIfException(Env())) {
      return absl::InternalError("Could not get duration");
    }
    return result;
  }

  absl::StatusOr<int> GetCurrentPosition() {
    int result = CallIntMethod(get_current_position_);
    if (CheckIfException(Env())) {
      return absl::InternalError("Could not get current position");
    }
    return result;
  }

  absl::StatusOr<int> GetVideoWidth() {
    int result = CallIntMethod(get_video_width_);
    if (CheckIfException(Env())) {
      return absl::InternalError("Could not get video width");
    }
    return result;
  }

  absl::StatusOr<int> GetVideoHeight() {
    int result = CallIntMethod(get_video_height_);
    if (CheckIfException(Env())) {
      return absl::InternalError("Could not get video height");
    }
    return result;
  }

  absl::StatusOr<MediaStereoMode> GetStereoMode() {
    int result = CallIntMethod(get_stereo_mode_);
    if (CheckIfException(Env())) {
      return absl::InternalError("Could not get stereo mode");
    }
    return static_cast<MediaStereoMode>(result);
  }

  absl::StatusOr<MediaColorSpace> GetColorSpace() {
    MP_ASSIGN_OR_RETURN(
        MediaColorSpace::Standard standard,
        MediaColorSpace::ToColorStandard(CallIntMethod(get_color_standard_)));

    MP_ASSIGN_OR_RETURN(
        MediaColorSpace::Transfer transfer,
        MediaColorSpace::ToColorTransfer(CallIntMethod(get_color_transfer_)));

    MP_ASSIGN_OR_RETURN(
        MediaColorSpace::Range range,
        MediaColorSpace::ToColorRange(CallIntMethod(get_color_range_)));
    MP_ASSIGN_OR_RETURN(uint16_t max_content_light_level,
                     MediaColorSpace::ToMaxContentLightLevel(
                         CallIntMethod(get_max_content_light_level_)));

    MediaColorSpace color_space(standard, transfer, range,
                                max_content_light_level);
    color_space.SetLumaBitDepth(CallIntMethod(get_luma_bitdepth_));
    color_space.SetChromaBitDepth(CallIntMethod(get_chroma_bitdepth_));
    return color_space;
  }

  int BufferingState() { return GetStaticIntField(buffering_state_); }

  int BufferingDoneState() { return GetStaticIntField(buffering_done_state_); }

 private:
  bool CheckIfException(JNIEnv* env) {
    if (env->ExceptionCheck()) {
      env->ExceptionClear();
      return true;
    }
    return false;
  }

  JniHandle release_;
  JniHandle set_video_surface_;
  JniHandle set_media_item_;
  JniHandle set_protected_media_item_;
  JniHandle set_listener_;
  JniHandle play_;
  JniHandle pause_;
  JniHandle stop_;
  JniHandle prepare_;
  JniHandle set_playback_speed_;
  JniHandle set_looping_;
  JniHandle seek_to_;
  JniHandle seek_quick_;
  JniHandle seek_precise_;
  JniHandle set_volume_;
  JniHandle is_playing_;
  JniHandle is_looping_;
  JniHandle get_duration_;
  JniHandle get_current_position_;
  JniHandle get_video_width_;
  JniHandle get_video_height_;
  JniHandle get_stereo_mode_;
  JniHandle buffering_state_;
  JniHandle buffering_done_state_;
  JniHandle get_color_standard_;
  JniHandle get_color_transfer_;
  JniHandle get_color_range_;
  JniHandle get_luma_bitdepth_;
  JniHandle get_chroma_bitdepth_;
  JniHandle get_max_content_light_level_;
};

}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_EXOPLAYER_H_
