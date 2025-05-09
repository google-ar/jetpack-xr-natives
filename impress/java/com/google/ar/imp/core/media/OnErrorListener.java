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
 * Implementation of the OnErrorListener interface connected to JNI to be called when the Android
 * MediaPlayer encounters an error in the media player.
 */

// LINT.IfChange(OnErrorListener)
@UsedByNative("media_listener.cc")
public class OnErrorListener implements MediaPlayer.OnErrorListener {
  // LINT.ThenChange(
  // //depot/google3/third_party/impress/core/media/android/android_media_listener.cc:OnErrorListener
  // )

  private final long nativeHandle;

  @UsedByNative("media_listener.cc")
  public OnErrorListener(long nativeHandle) {
    this.nativeHandle = nativeHandle;
  }

  @Override
  public boolean onError(MediaPlayer mp, int what, int extra) {
    return nOnError(nativeHandle, what, extra);
  }

  // LINT.IfChange(OnPreparedJni)
  private static native boolean nOnError(long nativeHandle, int what, int extra);
  // LINT.ThenChange(
  // //depot/google3/third_party/impress/core/media/android/android_media_listener.cc:nOnError
  // )
}
