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
 * Implementation of the OnInfoListener interface connected to JNI to be called by the Android
 * MediaPlayer to communicate some info and/or warning about the media or its playback.
 */

// LINT.IfChange(OnInfoListener)
@UsedByNative("media_listener.cc")
public class OnInfoListener implements MediaPlayer.OnInfoListener {
  // LINT.ThenChange(
  // //depot/google3/third_party/impress/core/media/android/android_media_listener.cc:OnInfoListener
  // )

  private final long nativeHandle;

  @UsedByNative("media_listener.cc")
  public OnInfoListener(long nativeHandle) {
    this.nativeHandle = nativeHandle;
  }

  @Override
  public boolean onInfo(MediaPlayer mp, int what, int extra) {
    return nOnInfo(nativeHandle, what, extra);
  }

  // LINT.IfChange(OnPreparedJni)
  private static native boolean nOnInfo(long nativeHandle, int what, int extra);
  // LINT.ThenChange(
  // //depot/google3/third_party/impress/core/media/android/android_media_listener.cc:nOnInfo
  // )
}
