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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_ERASED_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_ERASED_H_

#include <cstddef>
#include <cstdio>
#include <new>
#include <type_traits>
#include <utility>

#include "absl/log/check.h"
#include "core/common/type_traits.h"

namespace imp {

// Stores a type-erased object.
//
// It's purpose is similar to std::any, but with these key differences:
//
// - It can store move-only types.
// - It is move only.
// - No RTTI, which means that it can't know what type it's holding.
// - It has template parameters that allows specifying the size and alignment of
//   local storage that can be used to store objects that are small enough
//   without doing dynamic heap allocations. This is a technique called "small
//   buffer optimization" that can be used to improve performance.
//
// Notably, typical implementations of std::any also support small buffer
// optimization, but they do not allow specifying the size and alignment of the
// local storage.
//
// As a comparison, std::unique_ptr<void, void (*)(void const*)> can be used as
// a type erased object, but it will always do heap allocations. Another
// difference from using a unique_ptr for type erasure is that the memory
// address of the object held by TypeErased can change, because when held in
// local memory the object will be moved when the TypeErased is moved.
template <std::size_t StorageSize, std::size_t Alignment>
class SizedTypeErased {
 public:
  // Helper for determining if a type can be stored locally.
  template <typename T>
  static constexpr bool kIsStoredLocally = std::integral_constant<
      bool, sizeof(T) <= StorageSize && alignof(T) <= Alignment &&
                Alignment % alignof(T) == 0 &&
                std::is_nothrow_move_constructible_v<T>>::value;
  // Constructs an empty type erased object.
  SizedTypeErased();

  // Constructs a type erased object that holds the given type.
  template <typename T>
  explicit SizedTypeErased(T&& held) noexcept;
  ~SizedTypeErased();

  SizedTypeErased(const SizedTypeErased& other) = delete;
  SizedTypeErased& operator=(const SizedTypeErased& other) = delete;
  SizedTypeErased(SizedTypeErased&& other) noexcept;
  SizedTypeErased& operator=(SizedTypeErased&& other) noexcept;

  explicit operator bool() const;

  // Returns true if this TypeErased object is holding a value.
  bool HasValue() const;

  // Returns a reference to the object stored in the type erased object.
  //
  // The returned reference is only valid if HasValue() is true and the type
  // matches the type stored in the object.
  //
  // *WARNING*: This is similarly dangerous as casting a void* to a type, but
  // it's necessary to support type erasure without RTTI.  This should only be
  // called when it's guaranteed that the object is of the correct type.
  template <typename T>
  T& Get();

  // Returns a const reference to the object stored in the type erased object.
  //
  // The returned reference is only valid if HasValue() is true and the type
  // matches the type stored in the object.
  //
  // *WARNING*: This is similarly dangerous as casting a void* to a type, but
  // it's necessary to support type erasure without RTTI. This should only be
  // called when it's guaranteed that the object is of the correct type.
  template <typename T>
  const T& Get() const;

 private:
  struct Remote {
    // The pointer to the remote object on the heap.
    void* ptr;
    // The size of the remote object on the heap. Used for deletion for objects
    // with trivial destructors.
    std::size_t size;
  };

  union SizedTypeErasedStorage {
    // When the object is too large to fit in local memory, we allocate it on
    // the heap using this structure.
    Remote remote;

    // When the object is small enough to fit in local memory, instantiate it
    // in-place in this memory.
    alignas(Alignment) std::byte local_memory[StorageSize];
  };

  // Which type of operation to perform on the type erased object.
  enum class ControllerOperation : bool {
    // Used to transfer the object during move construction/assignment.
    //
    // It's expected that before kTransfer is called, the object is empty. This
    // is because kTransfer can only move the new type, it can't delete the old
    // held type.
    kTransfer,
    // Used to delete the object.
    kDelete,
  };

  // Used to perform operations on the type erased object to implement move
  // construction/assignment and deletion. This is intentionally done using a
  // single function pointer both for type-erasure and to have minimal memory
  // overhead (a single pointer).
  using ControllerFn = void (*)(ControllerOperation, SizedTypeErasedStorage*,
                                SizedTypeErasedStorage*);

  template <typename T>
  static constexpr ControllerFn GetControllerFn();

  // Used to get a pointer to the object stored in local memory.
  template <typename T>
  inline static T* GetFromLocalStorage(SizedTypeErasedStorage* storage);

  // Used to get a pointer to the object stored in local memory.
  template <typename T>
  inline static const T* GetFromLocalStorage(
      const SizedTypeErasedStorage* storage);

  // Controller assigned when the TypeErased object is empty.
  //
  // This is used instead of assigning the controller to nullptr because doing
  // nullptr checks on function pointers prevents the compiler from doing some
  // optimizations that reduce binary size.
  inline static void EmptyController(ControllerOperation op,
                                     SizedTypeErasedStorage* storage,
                                     SizedTypeErasedStorage* other_storage);

  // Controller assigned when the TypeErased object is holding a trivial type
  // stored locally. In this case, we don't need the type of the object to
  // perform any operations which keeps binary size down.
  inline static void LocalTrivialController(
      ControllerOperation op, SizedTypeErasedStorage* storage,
      SizedTypeErasedStorage* other_storage);

  // Controller assigned when the TypeErased object is holding a non-trivial
  // type stored locally. In this case, we need to know the type of the object
  // to perform operations.
  template <typename T>
  inline static void LocalNonTrivialController(
      ControllerOperation op, SizedTypeErasedStorage* storage,
      SizedTypeErasedStorage* other_storage);

  // Controller assigned when the TypeErased object is holding a trivial type
  // stored remotely. In this case, we don't need the type of the object to
  // perform any operations which keeps binary size down.
  inline static void RemoteTrivialController(
      ControllerOperation op, SizedTypeErasedStorage* storage,
      SizedTypeErasedStorage* other_storage);

  // Controller assigned when the TypeErased object is holding a non-trivial
  // type stored remotely. In this case, we need to know the type of the object
  // to perform operations.
  template <typename T>
  inline static void RemoteNonTrivialController(
      ControllerOperation op, SizedTypeErasedStorage* storage,
      SizedTypeErasedStorage* other_storage);

  // Stores the object either locally or remotely.
  SizedTypeErasedStorage storage_;

  // The controller to use to perform operations on the type erased object.
  ControllerFn controller_;
};

template <std::size_t StorageSize, std::size_t Alignment>
template <typename T>
constexpr typename SizedTypeErased<StorageSize, Alignment>::ControllerFn
SizedTypeErased<StorageSize, Alignment>::GetControllerFn() {
  if constexpr (kIsStoredLocally<T>) {
    if constexpr (std::is_trivially_destructible_v<T> &&
                  std::is_trivially_copy_assignable_v<T>) {
      return LocalTrivialController;
    } else {
      return LocalNonTrivialController<T>;
    }
  } else {
    if constexpr (std::is_trivially_destructible_v<T> &&
                  alignof(T) <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      return RemoteTrivialController;
    } else {
      return RemoteNonTrivialController<T>;
    }
  }
}

template <std::size_t StorageSize, std::size_t Alignment>
template <typename T>
T* SizedTypeErased<StorageSize, Alignment>::GetFromLocalStorage(
    SizedTypeErasedStorage* storage) {
  return std::launder(reinterpret_cast<T*>(&storage->local_memory));
}

template <std::size_t StorageSize, std::size_t Alignment>
template <typename T>
const T* SizedTypeErased<StorageSize, Alignment>::GetFromLocalStorage(
    const SizedTypeErasedStorage* storage) {
  return std::launder(reinterpret_cast<const T*>(&storage->local_memory));
}

template <std::size_t StorageSize, std::size_t Alignment>
void SizedTypeErased<StorageSize, Alignment>::EmptyController(
    ControllerOperation op, SizedTypeErasedStorage* storage,
    SizedTypeErasedStorage* other_storage) {}

template <std::size_t StorageSize, std::size_t Alignment>
void SizedTypeErased<StorageSize, Alignment>::LocalTrivialController(
    ControllerOperation op, SizedTypeErasedStorage* storage,
    SizedTypeErasedStorage* other_storage) {
  switch (op) {
    case ControllerOperation::kTransfer:
      // Just directly assign in the trivial case.
      *storage = *other_storage;
      break;
    case ControllerOperation::kDelete:
      // Do nothing in the trivial case.
      break;
  }
}

template <std::size_t StorageSize, std::size_t Alignment>
template <typename T>
void SizedTypeErased<StorageSize, Alignment>::LocalNonTrivialController(
    ControllerOperation op, SizedTypeErasedStorage* storage,
    SizedTypeErasedStorage* other_storage) {
  switch (op) {
    case ControllerOperation::kTransfer: {
      T& to_transfer = *GetFromLocalStorage<T>(other_storage);

      // Placement new the move constructor into the local memory.
      new (storage->local_memory) T(std::move(to_transfer));

      // Old object  should now be destroyed.
      to_transfer.~T();
      break;
    }
    case ControllerOperation::kDelete: {
      // Call the destructor manually for things created via placement new.
      GetFromLocalStorage<T>(storage)->~T();
      break;
    }
  }
}

template <std::size_t StorageSize, std::size_t Alignment>
void SizedTypeErased<StorageSize, Alignment>::RemoteTrivialController(
    ControllerOperation op, SizedTypeErasedStorage* storage,
    SizedTypeErasedStorage* other_storage) {
  switch (op) {
    case ControllerOperation::kTransfer: {
      // Don't use swap to keep the code size down. Clearing the other_storage
      // is not necessary because the controller will be set to EmptyController.
      storage->remote = other_storage->remote;
      break;
    }
    case ControllerOperation::kDelete: {
// Use the global delete operator along with the size of the allocated
// object to delete the object. This allows us to avoid knowing the type
// of the object being deleted (keeping code size down), which is safe
// because we know the destructor is trivial.
#if defined(__cpp_sized_deallocation)
      ::operator delete(storage->remote.ptr, storage->remote.size);
#else
      // Sized delete unavailable, so just use the generic global delete
      // operator. This is still safe because we know the ptr was created with
      // the global new operator as a void pointer.
      ::operator delete(storage->remote.ptr);
#endif
      break;
    }
  }
}

template <std::size_t StorageSize, std::size_t Alignment>
template <typename T>
void SizedTypeErased<StorageSize, Alignment>::RemoteNonTrivialController(
    ControllerOperation op, SizedTypeErasedStorage* storage,
    SizedTypeErasedStorage* other_storage) {
  switch (op) {
    case ControllerOperation::kTransfer: {
      // Don't use swap to keep the code size down. Clearing the other_storage
      // is not necessary because the controller will be set to EmptyController.
      storage->remote = other_storage->remote;
      break;
    }
    case ControllerOperation::kDelete: {
      delete static_cast<T*>(storage->remote.ptr);
      break;
    }
  }
}

template <std::size_t StorageSize, std::size_t Alignment>
SizedTypeErased<StorageSize, Alignment>::SizedTypeErased()
    : controller_(EmptyController) {}

template <std::size_t StorageSize, std::size_t Alignment>
template <typename T>
SizedTypeErased<StorageSize, Alignment>::SizedTypeErased(T&& held) noexcept {
  using DecayedT = std::decay_t<T>;

  constexpr ControllerFn kController = GetControllerFn<DecayedT>();

  if constexpr (kIsStoredLocally<DecayedT>) {
    //  Placement new the object into the local memory.
    new (storage_.local_memory) DecayedT(std::forward<T>(held));
  } else {
    //  Allocate the object on the heap.
    if constexpr (kController == RemoteTrivialController) {
      // Use the global new operator to allocate the memory for the object and
      // then placement new the object into the memory.
      //
      // This is done because we are later going to delete it using the global
      // delete operator, and they must be paired together. If you're doing
      // global delete, you need to use global new.
      storage_.remote.ptr = ::operator new(sizeof(T));  // NOLINT
      new (storage_.remote.ptr) DecayedT(std::forward<T>(held));

      // We store the size of the object to delete it without knowing the type
      // later, which is safe because we know the destructor is trivial.
      storage_.remote.size = sizeof(DecayedT);
    } else {
      storage_.remote.ptr = new DecayedT(std::forward<T>(held));
    }
  }

  controller_ = kController;
}

template <std::size_t StorageSize, std::size_t Alignment>
SizedTypeErased<StorageSize, Alignment>::~SizedTypeErased() {
  controller_(ControllerOperation::kDelete, &storage_, nullptr);
}

template <std::size_t StorageSize, std::size_t Alignment>
SizedTypeErased<StorageSize, Alignment>::SizedTypeErased(
    SizedTypeErased<StorageSize, Alignment>&& other) noexcept {
  other.controller_(ControllerOperation::kTransfer, &storage_, &other.storage_);
  controller_ = other.controller_;
  other.controller_ = EmptyController;
}

template <std::size_t StorageSize, std::size_t Alignment>
SizedTypeErased<StorageSize, Alignment>&
SizedTypeErased<StorageSize, Alignment>::operator=(
    SizedTypeErased<StorageSize, Alignment>&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  controller_(ControllerOperation::kDelete, &storage_, nullptr);

  other.controller_(ControllerOperation::kTransfer, &storage_, &other.storage_);
  controller_ = other.controller_;
  other.controller_ = EmptyController;

  return *this;
}
template <std::size_t StorageSize, std::size_t Alignment>
SizedTypeErased<StorageSize, Alignment>::operator bool() const {
  return HasValue();
}

template <std::size_t StorageSize, std::size_t Alignment>
bool SizedTypeErased<StorageSize, Alignment>::HasValue() const {
  return controller_ != EmptyController;
}

template <std::size_t StorageSize, std::size_t Alignment>
template <typename T>
T& SizedTypeErased<StorageSize, Alignment>::Get() {
  // This check helps catch cases where Get is called with the wrong type.
  //
  // This is a DCHECK because it's not worth the binary size & overhead to make
  // this a hard error in release builds.
  //
  // The check is not perfect, because when comparing two trivial remote types
  // or two trivial local types this check will incorrectly pass.
  //
  // However, to fix this we'd have to bloat the memory footprint by storing the
  // type of the object, or we'd need to bloat binary size by making the trivial
  // controllers templated based on the type. This solution avoids doing that
  // even in debug mode to limit differences between debug and release builds,
  // and provide a "good enough" solution.
  

  if constexpr (kIsStoredLocally<T>) {
    return *GetFromLocalStorage<T>(&storage_);
  } else {
    return *static_cast<T*>(storage_.remote.ptr);
  }
}

template <std::size_t StorageSize, std::size_t Alignment>
template <typename T>
const T& SizedTypeErased<StorageSize, Alignment>::Get() const {
  // This check helps catch cases where Get is called with the wrong type.
  //
  // This is a DCHECK because it's not worth the binary size & overhead to make
  // this a hard error in release builds.
  //
  // The check is not perfect, because when comparing two trivial remote types
  // or two trivial local types this check will incorrectly pass.
  //
  // However, to fix this we'd have to bloat the memory footprint by storing the
  // type of the object, or we'd need to bloat binary size by making the trivial
  // controllers templated based on the type. This solution avoids doing that
  // even in debug mode to limit differences between debug and release builds,
  // and provide a "good enough" solution.
  

  if constexpr (kIsStoredLocally<T>) {
    return *GetFromLocalStorage<T>(&storage_);
  } else {
    return *static_cast<const T*>(storage_.remote.ptr);
  }
}

// A SizeTypeErased object with a default storage size and alignment.
//
// The default size was picked to be large enough to hold std::unique_ptr,
// std::shared_ptr, imp::OwnedPtr, and imp::Future.
using TypeErased = SizedTypeErased<16, 8>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TYPE_ERASED_H_
