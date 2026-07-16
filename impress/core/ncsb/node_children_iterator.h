/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_CHILDREN_ITERATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_CHILDREN_ITERATOR_H_

#include <stddef.h>

#include <cstdint>
#include <iterator>

#include "filament/filament/include/filament/TransformManager.h"
#include "core/ncsb/node_handle.h"

namespace imp {

class NodeChildrenRange;

// Standard-compliant InputIterator for stepping through the children of a Node.
// Note: This iterator is cheap to copy, but gets invalidated if children are
// added, removed, or reparented while traversing.
class NodeChildrenIterator {
 public:
  using value_type = NodeHandle;
  using difference_type = ptrdiff_t;
  using pointer = NodeHandle*;
  using reference = NodeHandle;
  using iterator_category = std::input_iterator_tag;

  NodeChildrenIterator() = default;

  NodeChildrenIterator(const NodeChildrenIterator&) = default;

  NodeChildrenIterator& operator=(const NodeChildrenIterator& other) {
    if (this != &other) {
      // filament::TransformManager::children_iterator contains a reference
      // member. In standard C++, this implicitly deletes the copy-assignment
      // operator. To fulfill CopyAssignable requirements (so it works in
      // standard algorithms and containers like std::vector), we explicitly
      // destroy and reconstruct the object in-place via placement new. This
      // leverages the existing copy-constructor and matches exactly how
      // std::optional and std::variant reseat non-assignable member variations.
      this->~NodeChildrenIterator();
      new (this) NodeChildrenIterator(other);
    }
    return *this;
  }

  NodeChildrenIterator& operator++() {
    ++filament_iterator_;
    AssignNextValid();
    return *this;
  }

  NodeChildrenIterator operator++(int) {
    NodeChildrenIterator previous_iterator(
        *transform_manager_, filament_iterator_, end_filament_iterator_,
        current_node_);
    ++(*this);
    return previous_iterator;
  }

  bool operator==(const NodeChildrenIterator& other) const noexcept {
    return filament_iterator_ == other.filament_iterator_;
  }

  bool operator!=(const NodeChildrenIterator& other) const noexcept {
    return filament_iterator_ != other.filament_iterator_;
  }

  reference operator*() const { return current_node_; }

 private:
  NodeChildrenIterator(
      filament::TransformManager& transform_manager,
      filament::TransformManager::children_iterator filament_iterator,
      filament::TransformManager::children_iterator
          end_filament_iterator) noexcept
      : transform_manager_(&transform_manager),
        filament_iterator_(filament_iterator),
        end_filament_iterator_(end_filament_iterator) {
    AssignNextValid();
  }

  NodeChildrenIterator(
      filament::TransformManager& transform_manager,
      filament::TransformManager::children_iterator filament_iterator,
      filament::TransformManager::children_iterator end_filament_iterator,
      NodeHandle node) noexcept
      : transform_manager_(&transform_manager),
        filament_iterator_(filament_iterator),
        end_filament_iterator_(end_filament_iterator),
        current_node_(node) {}

  void AssignNextValid() {
    while (filament_iterator_ != end_filament_iterator_) {
      current_node_ =
          NodeHandle(transform_manager_->getEntity(*filament_iterator_));
      if (current_node_) {
        return;
      }
      ++filament_iterator_;
    }
    current_node_ = NodeHandle();
  }

  filament::TransformManager* transform_manager_ = nullptr;
  filament::TransformManager::children_iterator filament_iterator_;
  filament::TransformManager::children_iterator end_filament_iterator_;
  NodeHandle current_node_;

  friend class ::imp::NodeChildrenRange;
};

// A lightweight, iterable view over the children of a Node without immediate
// container instantiation (like std::vector).
//
// Usage:
//   for (NodeHandle child : node->GetChildrenRange()) { ... }
//
// Warning: Any modification to the immediate hierarchy of this node during
// traversal—such as creating, destroying, or re-parenting children—will
// silently invalidate current iteration bounds.
class NodeChildrenRange {
 public:
  NodeChildrenIterator begin() const {
    return NodeChildrenIterator(
        transform_manager_,
        transform_manager_.getChildrenBegin(parent_instance_),
        transform_manager_.getChildrenEnd(parent_instance_));
  }

  NodeChildrenIterator end() const {
    return NodeChildrenIterator(
        transform_manager_, transform_manager_.getChildrenEnd(parent_instance_),
        transform_manager_.getChildrenEnd(parent_instance_), NodeHandle{});
  }

  // Returns the number of children of this node.
  //
  // Note: This includes filament entities that are not Impress nodes. The
  // iterator will skip over those. This is an extremely rare edge case.
  int32_t GetCount() const {
    return transform_manager_.getChildCount(parent_instance_);
  }

 private:
  explicit NodeChildrenRange(
      filament::TransformManager& transform_manager,
      filament::TransformManager::Instance parent_instance) noexcept
      : transform_manager_(transform_manager),
        parent_instance_(parent_instance) {}

  filament::TransformManager& transform_manager_;
  filament::TransformManager::Instance parent_instance_;

  friend class ::imp::Node;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_CHILDREN_ITERATOR_H_
