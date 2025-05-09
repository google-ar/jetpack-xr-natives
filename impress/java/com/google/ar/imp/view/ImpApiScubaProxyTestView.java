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

package com.google.ar.imp.view;

import android.content.Context;
import android.util.AttributeSet;
import androidx.annotation.Nullable;
import com.google.ar.imp.core.web.FragmentHost;

/** A placeholder view for apps that don't use ImpSurfaceView / ImpTextureView for scuba tests. */
public final class ImpApiScubaProxyTestView extends android.view.View
    implements ImpApiScubaProvider {

  @Nullable private ImpApiScuba impApiScuba;

  public ImpApiScubaProxyTestView(Context context) {
    super(context);
  }

  public ImpApiScubaProxyTestView(Context context, @Nullable AttributeSet attrs) {
    super(context, attrs);
  }

  public ImpApiScubaProxyTestView(Context context, FragmentHost host) {
    super(context);
  }

  public void setImpApiScuba(ImpApiScuba impApiScuba) {
    this.impApiScuba = impApiScuba;
  }

  @Override
  public ImpApiScuba getImpApiScuba() {
    return impApiScuba;
  }
}
