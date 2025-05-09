/*
 * Copyright 2025 Google LLC
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

package com.google.androidxr.splitengine;

import com.android.extensions.xr.node.Node;
import java.io.Closeable;

/** Container for a subspace Node and its subspace ID. */
public final class SubspaceNode {
  // LINT.IfChange(subspaceAnchor)
  /** The anchor type of the subspace. */
  public enum Anchor {
    TASK_SPACE,
    WORLD_SPACE,
  }

  // LINT.ThenChange(//depot/google3/third_party/split_engine/subspace_root.h:subspaceAnchor)

  private final Node subspaceNode;

  public final int subspaceId;
  public Anchor anchor;
  public Closeable nodeTransformSubscription;

  public SubspaceNode(int subspaceId, Node subspaceNode) {
    this.subspaceId = subspaceId;
    this.subspaceNode = subspaceNode;
    this.anchor = Anchor.TASK_SPACE;
  }

  /** Returns the subspace {@link Node}. */
  public Node getSubspaceNode() {
    return subspaceNode;
  }
}
