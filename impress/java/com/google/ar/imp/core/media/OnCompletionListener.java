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

package com.google.ar.imp.core.media;

import android.media.MediaPlayer;
import com.google.android.filament.proguard.UsedByNative;

/**
 * Implementation of the OnCompletionListener interface connected to JNI to be called when the
 * Android MediaPlayer has completed playback of the media data source.
 */

// LINT.IfChange(OnCompletionListener)
@UsedByNative("media_listener.cc")
class OnCompletionListener implements MediaPlayer.OnCompletionListener {
  // LINT.ThenChange(
  // //depot/google3/third_party/impress/core/media/android/android_media_listener.h:OnCompletionListener
  // )

  private final long nativeHandle;

  @UsedByNative("media_listener.cc")
  public OnCompletionListener(long nativeHandle) {
    this.nativeHandle = nativeHandle;
  }

  @Override
  public void onCompletion(MediaPlayer mediaplayer) {
    nOnCompletion(nativeHandle);
  }

  // LINT.IfChange(OnCompletionJni)
  private static native void nOnCompletion(long nativeHandle);
  // LINT.ThenChange(
  // //depot/google3/third_party/impress/core/media/android/android_media_listener.cc:OnCompletionJni
  // )
}
