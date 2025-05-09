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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SERVICE_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SERVICE_IMPL_H_

#include <jni.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "flatbuffers/verifier.h"
#include "core/async/executor.h"
#include "core/common/rememberer.h"
#include "core/split_engine/android/split_engine_android_surface_factory.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// This is the implementation of the Split Engine Bridge service backend
// independent of choice of NDK or platform binder implementation. It is
// responsible for managing the shared memory buffers, processing Split Engine
// schema messages, and handling bridge cleanup.
//
// This class is responsible for managing one or more bridges for different apps
// simultaneously. Each bridge is identified by a BridgeId, and each bridge can
// have one or more memory buffers registered.
//
// Essentially all methods are expected to be called on the binder thread except
// where noted (CleanupBridge for example). Work is scheduled on the Impress
// foreground executor where Nodes, Renderables, etc. live in order to create
// the 3D scene, but the majority of bookkeeping around buffers, message groups,
// and status reporting is done on the binder thread.
//
// There are a set of mutexes to ensure thread safety. Most mutexes are only
// needed during bridge cleanup since the cleanup must happen on the Impress
// foreground thread as part of tearing down the scene graph for the bridge.
//
// The key operations are:
// 1. RegisterBuffer - called by the client to register a shared memory buffer
//    for the bridge to marshall messages to the backend renderer from the app.
//    The client buffer is mapped into the system process address space and a
//    RenderBridgeBuffer record is created to track the buffer. Multiple buffers
//    can be registered for a single bridge at a given time and may be in flight
//    simultaneously. When all messages and, importantly, assets have been fully
//    processed, the buffer is released back to the app's shared memory pool.
// 2. ProcessRegion - called by the client to process a region of a shared
//    memory buffer. These regions are the binary backing data for flatbuffers
//    that are sent from the app to the system. The region is assumed to contain
//    one or more Split Engine schema messages, in particular one of
//    BeginMessageGroup, EndMessageGroup, or Command.
//    * BeginMessageGroup is the signal that a new group of Command messages has
//       been opened.
//    * EndMessageGroup is the signal that all Commands in the group have been
//       received and the group can be processed.
//    * Command is a message that contains operations for the Impress renderer.
//      All Command messages must be processed contiguously to avoid breaking
//      work across frame boundaries.
//
// Currently, a "zero-tolerance policy" is in-place for message validation. If
// any message is invalid, the bridge is shut down and all future calls return
// the last error status. All content associated with that bridge is destroyed.
//
class SplitEngineSharedMemoryBridgeServiceImpl : public Rememberer {
 public:
  using ResponseHandlerFunction =
      std::function<void(const std::vector<uint8_t>&)>;
  using ReleaseMessageGroupFunction = std::function<void(int32_t)>;

  // Base class for an object that encapsulates buffer storage.
  // This is needed to keep the buffer while processing messages,
  // even if the client terminates.
  class MessageGroupStorage {
   public:
    MessageGroupStorage() = default;
    virtual ~MessageGroupStorage() = default;
    MessageGroupStorage(const MessageGroupStorage&) = delete;
  };

  SplitEngineSharedMemoryBridgeServiceImpl(BaseView& view, Executor* executor);
  virtual ~SplitEngineSharedMemoryBridgeServiceImpl() = default;

  // This is not a normal Impress per-frame Update method. It is called to set a
  // new view/executor on the Bridge. This is only used by the phone emulator
  // version of Split Engine.
  void Update(BaseView& view, Executor* executor);

  absl::Status InitializeBridge(
      BridgeId bridge_id,
      ReleaseMessageGroupFunction release_message_group_function);
  absl::Status RegisterBuffer(BridgeId bridge_id, BufferId buffer_id,
                              int file_descriptor, size_t size_bytes);

  // For performance reasons, processRegion returns the status of the last
  // processed region, not the specified region.
  absl::Status ProcessRegion(BufferId buffer_id,
                             std::shared_ptr<MessageGroupStorage> storage,
                             int32_t offset_bytes, size_t region_length_bytes);

  // Returns the status of the bridge with the given bridge id.
  // If there is an error processing a message, the bridge will be shut down
  // and the last error status will be returned indefinitely.
  absl::Status GetBridgeStatus(BridgeId bridge_id);

  // Destroys all content and records associated with the bridge.
  void CleanupBridge(BridgeId bridge_id);
  // Releases the given buffer back to the app.
  void CleanupBuffer(BufferId buffer_id);

  // Creates an Android Surface to back the given texture IDs. This lets the
  // client render to the Surface and use the texture IDs on renderables in the
  // Impress scene.
  absl::Status CreateExternalTextureSurface(
      BridgeId bridge_id, const std::vector<TextureId>& in_texture_ids,
      jobject& out_surface);
  // Sets the size of the external texture surface. This is usually not needed
  // since the surface size usually set by Android when content is rendered to
  // it.
  absl::Status SetExternalTextureSurfaceSize(BridgeId bridge_id,
                                             TextureId in_texture_id,
                                             int32_t width, int32_t height);
  // Handles a generic Split Engine flatbuffer schema Request from the app.
  // This is a generic message that can be used to send arbitrary data between
  // the app and the system. The app sends a Request message to the system,
  // which is relayed back to the app via the ResponseHandlerFunction.
  absl::Status SendRequest(BridgeId bridge_id, const std::vector<uint8_t>& data,
                           ResponseHandlerFunction response_handler);
  BaseView& View() { return *view_; }

 protected:
  // Handles a Command message from the bridge and passes it to the renderer.
  virtual absl::Status HandleMessage(
      BridgeId bridge_id, flatbuffers::Verifier& verifier,
      const uint8_t* message,
      SplitEngineRenderer::OnFinishedCallback on_finished);

 private:
  // Handles a MessageGroup message from the bridge.
  absl::Status OnMessageGroupMessage(
      BridgeId bridge_id, BufferId buffer_id, const uint8_t* message,
      size_t size, std::shared_ptr<MessageGroupStorage> storage);
  // Handles the BeginMessageGroup message from the bridge. This is the signal
  // that a new group of Command messages has been opened.
  absl::Status OnBeginMessageGroupMessage(
      BridgeId bridge_id, BufferId buffer_id,
      const android_xr::schemas::MessageGroup* message_group,
      std::shared_ptr<MessageGroupStorage> storage);
  // Handles the EndMessageGroup message from the bridge. This is the signal
  // that all Command messages in the group have been received, and the group
  // can be processed. Calls HandleAllMessagesInGroup to process the group.
  absl::Status OnEndMessageGroupMessage(
      BridgeId bridge_id, BufferId buffer_id,
      const android_xr::schemas::MessageGroup* message_group,
      std::shared_ptr<MessageGroupStorage> storage);
  // Moves the MessageGroupTracker to the Impress foreground thread and
  // processes all Command messages in the group on that thread.
  absl::Status HandleAllMessagesInGroup(BufferId buffer_id,
                                        MessageGroupId message_group_id);
  // Notifies the MessageGroupTracker that a message has finished processing.
  // If the group is completely finished, the buffer release function is called.
  void ReleaseMessageGroupIfFinished(BridgeId bridge_id, BufferId buffer_id,
                                     MessageGroupId message_group_id);

  void SetBridgeStatus(BridgeId bridge_id, absl::Status status);

  BaseView* view_;
  Executor* foreground_executor_;

  // Guards bridge_status_, which is accessed on both the binder and foreground
  // threads.
  absl::Mutex bridge_status_mutex_;
  absl::flat_hash_map<BridgeId, absl::Status> bridge_status_
      ABSL_GUARDED_BY(bridge_status_mutex_);

  absl::Mutex bridge_release_message_group_functions_mutex_;
  // Each bridge has a single release function, which is created on the binder
  // thread and called on the foreground executor.
  absl::flat_hash_map<BridgeId, ReleaseMessageGroupFunction>
      bridge_release_message_group_functions_
          ABSL_GUARDED_BY(bridge_release_message_group_functions_mutex_);

  struct RenderBridgeBuffer {
    const BridgeId bridge_id;
    // Note: this cannot be const since ::munmap is not const.
    uint8_t* buffer_ptr;
    const size_t buffer_size_bytes;
  };
  absl::flat_hash_map<BufferId, const RenderBridgeBuffer> buffers_;

  // Represents a single message in a buffer by offset and size.
  struct MessageOffset {
    const int32_t offset_bytes;
    const size_t size_bytes;
  };

  // All the data for a group of flatbuffer messages in a buffer.
  class MessageGroupTracker {
   public:
    MessageGroupTracker() = default;
    MessageGroupTracker(BridgeId bridge_id, MessageGroupId message_group_id,
                        std::shared_ptr<MessageGroupStorage> storage)
        : bridge_id_(bridge_id),
          message_group_id_(message_group_id),
          storage_{storage} {}
    void OnMessageFinished() { ++num_messages_finished_; }
    BridgeId GetBridgeId() { return bridge_id_; }
    MessageGroupId GetMessageGroupId() { return message_group_id_; }

    bool IsFinished() {
      return num_messages_finished_ == message_offsets_.size();
    }
    void AddMessage(int32_t offset_bytes, size_t size_bytes) {
      message_offsets_.push_back(
          {.offset_bytes = offset_bytes, .size_bytes = size_bytes});
    }

    const std::vector<MessageOffset>& GetMessageOffsets() {
      return message_offsets_;
    }

   private:
    const BridgeId bridge_id_;
    const MessageGroupId message_group_id_ = -1;
    int num_messages_finished_ = 0;
    std::shared_ptr<const MessageGroupStorage> storage_;
    std::vector<MessageOffset> message_offsets_;
  };

  // Message groups that have been started but not finished. This member is only
  // accessed on the binder thread.
  // Note: the mutex is only needed during bridge cleanup since the cleanup
  // must happen on the Impress foreground thread as part of tearing down
  // renderables.
  absl::Mutex message_groups_mutex_;
  absl::flat_hash_map<BufferId, std::unique_ptr<MessageGroupTracker>>
      message_groups_ ABSL_GUARDED_BY(message_groups_mutex_);
  // Message groups that are currently being processed. This member is only
  // accessed on the foreground executor.
  absl::flat_hash_map<BufferId, std::unique_ptr<MessageGroupTracker>>
      message_groups_processing_;

  SplitEngineSurfaceFactory surface_factory_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_SERVICE_IMPL_H_
