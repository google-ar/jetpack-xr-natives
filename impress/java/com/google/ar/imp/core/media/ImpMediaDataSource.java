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

import android.media.MediaDataSource;
import com.google.android.filament.proguard.UsedByNative;
import java.io.IOException;

/** Data class for Android Media Player to read audio data from. */
// LINT.IfChange()
@UsedByNative("android_media_data_source.h")
class ImpMediaDataSource extends MediaDataSource {
  private byte[] data;

  @UsedByNative("android_media_data_source.h")
  public ImpMediaDataSource() {}

  // Setting data separately from constructor due to restrictions of preprocessing data in the JNI
  // constructor initialization list.
  @UsedByNative("android_media_data_source.h")
  public synchronized void setData(byte[] data) {
    this.data = data;
  }

  @Override
  public synchronized int readAt(long position, byte[] buffer, int offset, int size) {
    int length = data.length;
    if (position >= length) {
      return -1;
    }

    if (position + size > length) {
      size = (int) (size - (position + size) + length);
    }

    System.arraycopy(data, (int) position, buffer, offset, size);
    return size;
  }

  @Override
  public long getSize() {
    return data.length;
  }

  @Override
  public void close() throws IOException {}
}
// LINT.ThenChange(
// //depot/google3/third_party/impress/core/media/android/android_media_data_source.h
// )
