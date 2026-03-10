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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_PLAYER_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_PLAYER_H_

#include <jni.h>

#include <cstdint>
#include <string>

#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/media/android/android_media_data_source.h"
#include "core/media/android/android_media_listener.h"
#include "core/media/android/android_playback_params.h"
#include "core/media/media_source.h"
#include "core/view/platforms/android/wrappers/surface.h"

namespace imp::media {

class AndroidMediaPlayer : public JavaWrapper {
 public:
  explicit AndroidMediaPlayer(const Context& context)
      : JavaWrapper(context.GetJniEnv(), "android/media/MediaPlayer", "()V") {
    set_data_source_ =
        GetMethodHandle("setDataSource", "(Landroid/media/MediaDataSource;)V");
    set_data_source_string_ =
        GetMethodHandle("setDataSource", "(Ljava/lang/String;)V");
    set_on_completion_listener_ =
        GetMethodHandle("setOnCompletionListener",
                        "(Landroid/media/MediaPlayer$OnCompletionListener;)V");
    set_on_seek_complete_listener_ = GetMethodHandle(
        "setOnSeekCompleteListener",
        "(Landroid/media/MediaPlayer$OnSeekCompleteListener;)V");
    set_on_prepared_listener_ =
        GetMethodHandle("setOnPreparedListener",
                        "(Landroid/media/MediaPlayer$OnPreparedListener;)V");
    set_on_error_listener_ = GetMethodHandle(
        "setOnErrorListener", "(Landroid/media/MediaPlayer$OnErrorListener;)V");
    set_on_info_listener_ = GetMethodHandle(
        "setOnInfoListener", "(Landroid/media/MediaPlayer$OnInfoListener;)V");
    set_surface_ = GetMethodHandle("setSurface", "(Landroid/view/Surface;)V");
    prepare_ = GetMethodHandle("prepare", "()V");
    prepare_async_ = GetMethodHandle("prepareAsync", "()V");
    pause_ = GetMethodHandle("pause", "()V");
    start_ = GetMethodHandle("start", "()V");
    stop_ = GetMethodHandle("stop", "()V");
    set_looping_ = GetMethodHandle("setLooping", "(Z)V");
    seek_to_ = GetMethodHandle("seekTo", "(JI)V");
    set_volume_ = GetMethodHandle("setVolume", "(FF)V");
    is_playing_ = GetMethodHandle("isPlaying", "()Z");
    is_looping_ = GetMethodHandle("isLooping", "()Z");
    reset_ = GetMethodHandle("reset", "()V");
    release_ = GetMethodHandle("release", "()V");
    get_duration_ = GetMethodHandle("getDuration", "()I");
    get_current_position_ = GetMethodHandle("getCurrentPosition", "()I");
    get_video_width_ = GetMethodHandle("getVideoWidth", "()I");
    get_video_height_ = GetMethodHandle("getVideoHeight", "()I");
    media_error_unknown_ = GetStaticFieldHandle("MEDIA_ERROR_UNKNOWN", "I");
    media_error_server_died_ =
        GetStaticFieldHandle("MEDIA_ERROR_SERVER_DIED", "I");
    media_error_io_ = GetStaticFieldHandle("MEDIA_ERROR_IO", "I");
    media_error_malformed_ = GetStaticFieldHandle("MEDIA_ERROR_MALFORMED", "I");
    media_error_unsupported_ =
        GetStaticFieldHandle("MEDIA_ERROR_UNSUPPORTED", "I");
    media_error_timed_out_ = GetStaticFieldHandle("MEDIA_ERROR_TIMED_OUT", "I");
    seek_previous_sync_ = GetStaticFieldHandle("SEEK_PREVIOUS_SYNC", "I");
    seek_closest_ = GetStaticFieldHandle("SEEK_CLOSEST", "I");
    media_info_buffering_start_ =
        GetStaticFieldHandle("MEDIA_INFO_BUFFERING_START", "I");
    media_info_buffering_end_ =
        GetStaticFieldHandle("MEDIA_INFO_BUFFERING_END", "I");
    get_playback_params_ = GetMethodHandle("getPlaybackParams",
                                           "()Landroid/media/PlaybackParams;");
    set_playback_params_ = GetMethodHandle("setPlaybackParams",
                                           "(Landroid/media/PlaybackParams;)V");
  }

  ~AndroidMediaPlayer() override {
    CallVoidMethod(reset_);
    CallVoidMethod(release_);
  }

  void SetDataSource(AndroidMediaDataSource* data_source) {
    CallVoidMethod(set_data_source_, data_source->Reference());
  }

  void SetDataSource(const std::string& data_source_string) {
    CallVoidMethod(set_data_source_string_,
                   ToJniString(Env(), data_source_string).get());
  }

  void SetOnCompletionListener(OnCompletionListener* on_completion_listener) {
    CallVoidMethod(set_on_completion_listener_,
                   on_completion_listener->Reference());
  }

  void SetOnSeekCompleteListener(
      OnSeekCompleteListener* on_seek_complete_listener) {
    CallVoidMethod(set_on_seek_complete_listener_,
                   on_seek_complete_listener->Reference());
  }

  void SetOnPreparedListener(OnPreparedListener* on_prepared_listener) {
    CallVoidMethod(set_on_prepared_listener_,
                   on_prepared_listener->Reference());
  }

  void SetOnErrorListener(OnErrorListener* on_error_listener) {
    CallVoidMethod(set_on_error_listener_, on_error_listener->Reference());
  }

  void SetOnInfoListener(OnInfoListener* on_info_listener) {
    CallVoidMethod(set_on_info_listener_, on_info_listener->Reference());
  }

  void SetSurface(android::Surface* surface) {
    CallVoidMethod(set_surface_, surface->Reference());
  }

  bool Pause() {
    CallVoidMethod(pause_);
    return !CheckIfException(Env());
  }

  bool Start() {
    CallVoidMethod(start_);
    return !CheckIfException(Env());
  }

  bool Stop() {
    CallVoidMethod(stop_);
    return !CheckIfException(Env());
  }

  bool SetPlaybackSpeed(float speed) {
    AndroidPlaybackParams playback_params(
        Env(), CallObjectMethod(get_playback_params_));
    if (!playback_params.SetSpeed(speed)) return false;
    CallVoidMethod(set_playback_params_, playback_params.Reference());
    return !CheckIfException(Env());
  }

  bool SetLooping(bool loop) {
    CallVoidMethod(set_looping_, loop);
    return !CheckIfException(Env());
  }

  bool SeekTo(int milliseconds, MediaSource::SeekType seek_type) {
    int mode = -1;
    switch (seek_type) {
      case MediaSource::SeekType::QUICK:
        mode = GetStaticIntField(seek_previous_sync_);
        break;
      case MediaSource::SeekType::PRECISE:
        mode = GetStaticIntField(seek_closest_);
        break;
    }
    CallVoidMethod(seek_to_, static_cast<int64_t>(milliseconds), mode);
    return !CheckIfException(Env());
  }

  bool SetVolume(float leftVolume, float rightVolume) {
    CallVoidMethod(set_volume_, leftVolume, rightVolume);
    return !CheckIfException(Env());
  }

  bool IsPlaying() {
    bool result = CallBooleanMethod(is_playing_);
    if (CheckIfException(Env())) {
      return false;
    }
    return result;
  }

  bool IsLooping() {
    bool result = CallBooleanMethod(is_looping_);
    if (CheckIfException(Env())) {
      return false;
    }
    return result;
  }

  int GetDuration() {
    int result = CallIntMethod(get_duration_);
    if (CheckIfException(Env())) {
      return -1;
    }
    return result;
  }

  int GetCurrentPosition() {
    int result = CallIntMethod(get_current_position_);
    if (CheckIfException(Env())) {
      return -1;
    }
    return result;
  }

  int GetVideoWidth() {
    int result = CallIntMethod(get_video_width_);
    if (CheckIfException(Env())) {
      return 0;
    }
    return result;
  }

  int GetVideoHeight() {
    int result = CallIntMethod(get_video_height_);
    if (CheckIfException(Env())) {
      return 0;
    }
    return result;
  }

  bool Prepare() {
    CallVoidMethod(prepare_);
    return !CheckIfException(Env());
  }

  bool PrepareAsync() {
    CallVoidMethod(prepare_async_);
    return !CheckIfException(Env());
  }

  int GetMediaErrorUnknown() { return GetStaticIntField(media_error_unknown_); }
  int GetMediaErrorServerDied() {
    return GetStaticIntField(media_error_server_died_);
  }
  int MediaErrorIO() { return GetStaticIntField(media_error_io_); }
  int MediaErrorMalformed() {
    return GetStaticIntField(media_error_malformed_);
  }
  int MediaErrorUnsupported() {
    return GetStaticIntField(media_error_unsupported_);
  }
  int MediaErrorTimedOut() { return GetStaticIntField(media_error_timed_out_); }

  // Unspecified low-level system error. This value originated from
  // UNKNOWN_ERROR in system/core/include/utils/Errors.h
  // see android.media.MediaPlayer.OnErrorListener
  int MediaErrorSystem() { return -2147483648; }

  int MediaInfoBufferingStart() {
    return GetStaticIntField(media_info_buffering_start_);
  }

  int MediaInfoBufferingEnd() {
    return GetStaticIntField(media_info_buffering_end_);
  }

 private:
  // TODO: Properly log the exceptions thrown
  bool CheckIfException(JNIEnv* env) {
    if (env->ExceptionCheck()) {
      env->ExceptionClear();
      return true;
    }
    return false;
  }

  JniHandle set_data_source_;
  JniHandle set_data_source_string_;
  JniHandle set_on_completion_listener_;
  JniHandle set_on_prepared_listener_;
  JniHandle set_on_error_listener_;
  JniHandle set_on_info_listener_;
  JniHandle set_on_seek_complete_listener_;
  JniHandle set_surface_;
  JniHandle prepare_;
  JniHandle prepare_async_;
  JniHandle pause_;
  JniHandle start_;
  JniHandle stop_;
  JniHandle seek_to_;
  JniHandle set_volume_;
  JniHandle set_looping_;
  JniHandle is_playing_;
  JniHandle is_looping_;
  JniHandle reset_;
  JniHandle release_;
  JniHandle get_duration_;
  JniHandle get_current_position_;
  JniHandle get_video_width_;
  JniHandle get_video_height_;
  JniHandle media_error_unknown_;
  JniHandle media_error_server_died_;
  JniHandle media_error_io_;
  JniHandle media_error_malformed_;
  JniHandle media_error_unsupported_;
  JniHandle media_error_timed_out_;
  JniHandle seek_previous_sync_;
  JniHandle seek_closest_;
  JniHandle media_info_buffering_start_;
  JniHandle media_info_buffering_end_;
  JniHandle get_playback_params_;
  JniHandle set_playback_params_;
};

}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_PLAYER_H_
