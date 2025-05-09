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

package com.google.ar.imp.apibindings;

import androidx.annotation.NonNull;

/**
 * Interface defining the callback that get triggered when an asset is loaded.
 *
 * @hide
 */
interface IAssetLoader {
  /** Called when the asset is successfully loaded where the long value is the asset token. */
  public void onSuccess(long value);

  /** Called when the asset fails to load. */
  public void onFailure(@NonNull String message);
}
