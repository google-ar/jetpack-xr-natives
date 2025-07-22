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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/base/log_severity.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/common/owned_ptr_traits.h"
#include "core/common/owned_ptr_utilities.h"
#include "core/common/ref_counter.h"
#include "core/common/small_source_location.h"
#include "core/common/type_traits.h"

namespace imp {

// Forward declaration of BorrowedPtr.
template <typename T>
class BorrowedPtr;

// OwnedPtr is a move-only type that holds a unique pointer to an object of
// type T.
//
// OwnedPtr is similar to std::unique_ptr, but instead of having a get()
// method that returns a raw pointer, it has a Borrow() method that returns a
// BorrowedPtr<T> object. The OwnedPtr tracks how many objects are borrowed, and
// will Fatal if it is destroyed while any borrowed objects are still
// outstanding.
//
// Like std::unique_ptr, this is intended to encourage single ownership of the
// underlying object, but with the added benefit of being able to track
// outstanding references to the object to detect & prevent accidental
// use-after-frees.
//
// This is somewhat inspired by how borrowing works in Rust, however Rust does
// compile-time checking of borrow rules, and this is only a runtime check.
//
// This is particularly useful for caching mechanisms that want to know when
// memory is no longer in use and internally decide when to actually destroy the
// memory. The cache can simply pass out BorrowedPtr<T> objects to users, and
// check GetBorrowedCount() to see if the object is still in use.
//
// Note: SetOwnedPtrLogSeverity can be used to override the default behavior of
// OwnedPtr to instead log a warning or error if it is destroyed while any
// BorrowedPtr objects are still outstanding.
//
// Not thread safe, but can be externally synchronized.
//
// Similar to std::unique_ptr, a custom Deleter may be specified, otherwise the
// default Deleter will be used.

template <typename T, typename Deleter = std::default_delete<T>>
class OwnedPtr {
 public:
  // Creates an empty OwnedPtr.
  OwnedPtr();

  // Creates an OwnedPtr by taking ownership of the raw pointer passed in.
  explicit OwnedPtr(T* ptr, Deleter deleter = {});

  // Creates an OwnedPtr by taking ownership of the std::unique_ptr passed in.
  //
  // This constructor allows implicit conversion from std::unique_ptr to
  // OwnedPtr. In general, implicit conversion is not done in google style.
  //
  // However, in this case it is helpful because it enables us to safely
  // incrementally migrate users of Impress to use OwnedPtr. This is why:
  //
  // TextureFactory::CreateTexture currently returns a unique_ptr. Changing the
  // return type to OwnedPtr would require all Impress users to simultaneously
  // migrate all texture usages. Creating an overload would require changing
  // the name of the method, because the only difference is the return type.
  // Implicit conversion allows usages of TextureFactory to migrate to OwnedPtr
  // incrementally prior to changing TextureFactory itself.
  //
  // Additionally, this use case falls cleanly into the described "known
  // good designs" in (broken link)
  //
  // TODO: Remove implicit conversion after migration is done.
  OwnedPtr(std::unique_ptr<T, Deleter> ptr);

  ~OwnedPtr();

  // Cannot be copied.
  OwnedPtr(const OwnedPtr&) = delete;
  OwnedPtr& operator=(const OwnedPtr&) = delete;

  // Can be moved.
  OwnedPtr(OwnedPtr&& other) noexcept;
  OwnedPtr& operator=(OwnedPtr&& other) noexcept;

  // Move constructor and assignment for upcasting OwnedPtrs.
  template <
      typename U, typename E,
      imp_owned_ptr_traits::EnableIfCanUpcastOwnedPtr<T, Deleter, U, E> = 0>
  OwnedPtr(OwnedPtr<U, E>&& other) noexcept;
  template <
      typename U, typename E,
      imp_owned_ptr_traits::EnableIfCanUpcastOwnedPtr<T, Deleter, U, E> = 0>
  OwnedPtr& operator=(OwnedPtr<U, E>&& other) noexcept;

  explicit operator bool() const;
  bool operator==(const OwnedPtr<T, Deleter>& other) const;
  bool operator!=(const OwnedPtr<T, Deleter>& other) const;
  bool operator==(const BorrowedPtr<T>& other) const;
  bool operator!=(const BorrowedPtr<T>& other) const;
  bool operator==(std::nullptr_t) const;
  bool operator!=(std::nullptr_t) const;

  T& operator*() const;
  T* operator->() const;

  // Borrows the object.
  //
  // The OwnedPtr will track the number of borrowed objects, and
  // will Fatal if it is destroyed while any borrowed objects are still
  // outstanding.
  //
  // The BorrowedPtr<T> object can be copied and moved, and the count will be
  // tracked correctly.
  //
  // The loc parameter automatically tracks where Borrow is called from so that
  // if the OwnedPtr is destroyed while the BorrowedPtr<T> is still outstanding,
  // the location can be logged.
  //
  // A location can be manually passed in. This is useful, for instance, if
  // Borrow is being called from within a cache and the cache wants to track the
  // location of the cache access instead of the location Borrow is called from.
  BorrowedPtr<T> Borrow(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Returns the number of outstanding tracked BorrowedPtr<T> objects.
  uint16_t GetBorrowedCount() const;

  // Resets the OwnedPtr, destroying the held object if it is non-null.
  //
  // The OwnedPtr will Fatal if any tracked BorrowedPtr<T> objects are still
  // outstanding.
  void Reset();

 private:
  // This is a trick to prevent the Deleter from increasing the memory
  // usage of the OwnedPtr in cases where the Deleter functor is stateless (e.g.
  // std::default_delete). The trick is called "empty base optimization". See
  // https://en.cppreference.com/w/cpp/language/ebo
  //
  // NOTE: There is a better way to do this using
  // ABSL_ATTRIBUTE_NO_UNIQUE_ADDRESS, but it requires C++20.
  //
  // TODO: Switch to ABSL_ATTRIBUTE_NO_UNIQUE_ADDRESS once C++20 is
  // supported on all platforms.
  class AdditionalFieldsHolder : Deleter {
   public:
    AdditionalFieldsHolder() = default;

    explicit AdditionalFieldsHolder(Deleter deleter)
        : Deleter(std::move(deleter)) {}

    AdditionalFieldsHolder(RefCounter ref_counter, Deleter deleter)
        : Deleter(std::move(deleter)), ref_counter_(std::move(ref_counter)) {}

    const RefCounter& GetRefCounter() const { return ref_counter_; }

    Deleter& GetDeleter() { return static_cast<Deleter&>(*this); }

   private:
    RefCounter ref_counter_;

    template <typename U, typename E>
    friend class OwnedPtr;
  };

  T* ptr_ = nullptr;
  AdditionalFieldsHolder additional_fields_;

  template <typename H>
  friend H AbslHashValue(H hash, const OwnedPtr<T, Deleter>& ptr) {
    return H::combine(std::move(hash), ptr.ptr_);
  }

  friend class BorrowedPtr<T>;
  template <typename U, typename E>
  friend class OwnedPtr;
};

// BorrowedPtr holds a non-owning pointer to an object of type T.
//
// It is obtained by calling Borrow() of an OwnedPtr<T>. If the OwnedPtr<T> is
// destroyed while the BorrowedPtr<T> is still outstanding, the OwnedPtr<T> will
// Fatal by default.
//
// If SetOwnedPtrLogSeverity is called to lower the default severity, then
// a BorrowedPtr<T> can outlive its OwnedPtr<T> without causing a fatal. In this
// case, the bool operator will return false, and the * and -> operators will
// fatal with information about the location of the Borrow call.
//
// Not thread safe, but can be externally synchronized.
template <typename T>
class BorrowedPtr {
 public:
  BorrowedPtr();

  BorrowedPtr<T>(const BorrowedPtr&) = default;
  BorrowedPtr<T>& operator=(const BorrowedPtr&) = default;

  // Copy constructor and assignment for upcasting BorrowedPtrs.
  template <typename U,
            imp_owned_ptr_traits::EnableIfCanUpcastBorrowedPtr<T, U> = 0>
  BorrowedPtr(const BorrowedPtr<U>& other) noexcept;
  template <typename U,
            imp_owned_ptr_traits::EnableIfCanUpcastBorrowedPtr<T, U> = 0>
  BorrowedPtr& operator=(const BorrowedPtr<U>& other) noexcept;

  // Explicit copy constructor for downcasting BorrowedPtrs. This is separate
  // from upcasting to prevent implicit conversions when downcasting.
  template <typename U,
            imp_owned_ptr_traits::EnableIfCanDowncastBorrowedPtr<T, U> = 0>
  explicit BorrowedPtr(const BorrowedPtr<U>& other) noexcept;

  // Ensure that move constructor and assignment are noexcept.
  BorrowedPtr(BorrowedPtr&& other) noexcept = default;
  BorrowedPtr& operator=(BorrowedPtr&& other) noexcept = default;

  // Move constructor and assignment for upcasting BorrowedPtrs.
  template <typename U,
            imp_owned_ptr_traits::EnableIfCanUpcastBorrowedPtr<T, U> = 0>
  BorrowedPtr(BorrowedPtr<U>&& other) noexcept;
  template <typename U,
            imp_owned_ptr_traits::EnableIfCanUpcastBorrowedPtr<T, U> = 0>
  BorrowedPtr& operator=(BorrowedPtr<U>&& other) noexcept;

  // Explicit move constructor for downcasting BorrowedPtrs. This is separate
  // from upcasting to prevent implicit conversions when downcasting.
  template <typename U,
            imp_owned_ptr_traits::EnableIfCanDowncastBorrowedPtr<T, U> = 0>
  explicit BorrowedPtr(BorrowedPtr<U>&& other) noexcept;

  explicit operator bool() const;
  bool operator==(const BorrowedPtr<T>& other) const;
  bool operator!=(const BorrowedPtr<T>& other) const;
  template <typename Deleter>
  bool operator==(const OwnedPtr<T, Deleter>& other) const;
  template <typename Deleter>
  bool operator!=(const OwnedPtr<T, Deleter>& other) const;
  bool operator==(std::nullptr_t) const;
  bool operator!=(std::nullptr_t) const;

  T& operator*() const;
  T* operator->() const;

  // Returns the number of outstanding tracked BorrowedPtr<T> objects for the
  // OwnedPtr<T> this BorrowedPtr<T> comes from, inclusive of this
  // BorrowedPtr<T>.
  uint16_t GetBorrowedCount() const;

  // Returns the location that the BorrowedPtr<T> is associated with, either
  // from the original Borrow call or a WithNewLocation call.
  SmallSourceLocation GetLocation() const;

  // Returns a new BorrowedPtr<T> with the same underlying pointer, but with the
  // new location. The tracked count will include the new BorrowedPtr<T>.
  BorrowedPtr<T> WithNewLocation(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

 private:
  BorrowedPtr(T* ptr, RefCounter::Ref ref);

  void AssertNotDestroyed() const;

  T* ptr_ = nullptr;
  RefCounter::Ref ref_;

  template <typename H>
  friend H AbslHashValue(H hash, const BorrowedPtr<T>& ptr) {
    return H::combine(std::move(hash), ptr.ptr_);
  }
  template <typename U, typename Deleter>
  friend class OwnedPtr;
  template <typename U>
  friend class BorrowedPtr;
};

template <typename T, typename Deleter>
OwnedPtr<T, Deleter>::OwnedPtr() {}

template <typename T, typename Deleter>
OwnedPtr<T, Deleter>::OwnedPtr(T* ptr, Deleter deleter)
    : ptr_(ptr), additional_fields_(std::move(deleter)) {}

template <typename T, typename Deleter>
OwnedPtr<T, Deleter>::OwnedPtr(std::unique_ptr<T, Deleter> ptr)
    : ptr_(ptr.release()), additional_fields_(std::move(ptr.get_deleter())) {}

template <typename T, typename Deleter>
OwnedPtr<T, Deleter>::~OwnedPtr() {
  Reset();
}

template <typename T, typename Deleter>
OwnedPtr<T, Deleter>::OwnedPtr(OwnedPtr&& other) noexcept {
  Reset();

  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  additional_fields_ = std::move(other.additional_fields_);
}

template <typename T, typename Deleter>
OwnedPtr<T, Deleter>& OwnedPtr<T, Deleter>::operator=(
    OwnedPtr&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  Reset();

  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  additional_fields_ = std::move(other.additional_fields_);
  return *this;
}

template <typename T, typename Deleter>
template <typename U, typename E,
          imp_owned_ptr_traits::EnableIfCanUpcastOwnedPtr<T, Deleter, U, E>>
OwnedPtr<T, Deleter>::OwnedPtr(OwnedPtr<U, E>&& other) noexcept {
  Reset();

  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  additional_fields_ =
      AdditionalFieldsHolder(std::move(other.additional_fields_.ref_counter_),
                             std::move(other.additional_fields_.GetDeleter()));
}

template <typename T, typename Deleter>
template <typename U, typename E,
          imp_owned_ptr_traits::EnableIfCanUpcastOwnedPtr<T, Deleter, U, E>>
OwnedPtr<T, Deleter>& OwnedPtr<T, Deleter>::operator=(
    OwnedPtr<U, E>&& other) noexcept {
  Reset();

  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  additional_fields_ =
      AdditionalFieldsHolder(std::move(other.additional_fields_.ref_counter_),
                             std::move(other.additional_fields_.GetDeleter()));
  return *this;
}

template <typename T, typename Deleter>
T& OwnedPtr<T, Deleter>::operator*() const {
  return *ptr_;
}

template <typename T, typename Deleter>
T* OwnedPtr<T, Deleter>::operator->() const {
  return ptr_;
}

template <typename T, typename Deleter>
OwnedPtr<T, Deleter>::operator bool() const {
  return ptr_ != nullptr;
}

template <typename T, typename Deleter>
bool OwnedPtr<T, Deleter>::operator==(const OwnedPtr<T, Deleter>& other) const {
  return ptr_ == other.ptr_;
}

template <typename T, typename Deleter>
bool OwnedPtr<T, Deleter>::operator!=(const OwnedPtr<T, Deleter>& other) const {
  return ptr_ != other.ptr_;
}

template <typename T, typename Deleter>
bool OwnedPtr<T, Deleter>::operator==(const BorrowedPtr<T>& other) const {
  return ptr_ == other.ptr_;
}

template <typename T, typename Deleter>
bool OwnedPtr<T, Deleter>::operator!=(const BorrowedPtr<T>& other) const {
  return ptr_ != other.ptr_;
}

template <typename T, typename Deleter>
bool OwnedPtr<T, Deleter>::operator==(std::nullptr_t) const {
  return ptr_ == nullptr;
}

template <typename T, typename Deleter>
bool OwnedPtr<T, Deleter>::operator!=(std::nullptr_t) const {
  return ptr_ != nullptr;
}

template <typename T>
BorrowedPtr<T>::BorrowedPtr() {}

template <typename T>
BorrowedPtr<T>::BorrowedPtr(T* ptr, RefCounter::Ref ref)
    : ptr_(ptr), ref_(std::move(ref)) {}

template <typename T>
template <typename U, imp_owned_ptr_traits::EnableIfCanUpcastBorrowedPtr<T, U>>
BorrowedPtr<T>::BorrowedPtr(const BorrowedPtr<U>& other) noexcept {
  ptr_ = other.ptr_;
  ref_ = other.ref_;
}

template <typename T>
template <typename U, imp_owned_ptr_traits::EnableIfCanUpcastBorrowedPtr<T, U>>
BorrowedPtr<T>& BorrowedPtr<T>::operator=(
    const BorrowedPtr<U>& other) noexcept {
  ptr_ = other.ptr_;
  ref_ = other.ref_;
  return *this;
}

template <typename T>
template <typename U,
          imp_owned_ptr_traits::EnableIfCanDowncastBorrowedPtr<T, U>>
BorrowedPtr<T>::BorrowedPtr(const BorrowedPtr<U>& other) noexcept {
  ptr_ = static_cast<T*>(other.ptr_);
  ref_ = other.ref_;
}

template <typename T>
template <typename U, imp_owned_ptr_traits::EnableIfCanUpcastBorrowedPtr<T, U>>
BorrowedPtr<T>::BorrowedPtr(BorrowedPtr<U>&& other) noexcept {
  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  ref_ = std::move(other.ref_);
}

template <typename T>
template <typename U, imp_owned_ptr_traits::EnableIfCanUpcastBorrowedPtr<T, U>>
BorrowedPtr<T>& BorrowedPtr<T>::operator=(BorrowedPtr<U>&& other) noexcept {
  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  ref_ = std::move(other.ref_);
  return *this;
}

template <typename T>
template <typename U,
          imp_owned_ptr_traits::EnableIfCanDowncastBorrowedPtr<T, U>>
BorrowedPtr<T>::BorrowedPtr(BorrowedPtr<U>&& other) noexcept {
  ptr_ = static_cast<T*>(other.ptr_);
  other.ptr_ = nullptr;
  ref_ = std::move(other.ref_);
}

template <typename T>
T& BorrowedPtr<T>::operator*() const {
  AssertNotDestroyed();
  return *ptr_;
}

template <typename T>
T* BorrowedPtr<T>::operator->() const {
  AssertNotDestroyed();
  return ptr_;
}

template <typename T>
uint16_t BorrowedPtr<T>::GetBorrowedCount() const {
  return ref_.GetCount();
}

template <typename T>
SmallSourceLocation BorrowedPtr<T>::GetLocation() const {
  return ref_.GetLocation();
}

template <typename T>
BorrowedPtr<T> BorrowedPtr<T>::WithNewLocation(SmallSourceLocation loc) const {
  AssertNotDestroyed();
  return BorrowedPtr<T>(ptr_, ref_.WithNewLocation(loc));
}

template <typename T>
BorrowedPtr<T>::operator bool() const {
  return !ref_.IsCounterDestroyed() && ptr_ != nullptr;
}

template <typename T>
bool BorrowedPtr<T>::operator==(const BorrowedPtr<T>& other) const {
  return ptr_ == other.ptr_;
}

template <typename T>
bool BorrowedPtr<T>::operator!=(const BorrowedPtr<T>& other) const {
  return ptr_ != other.ptr_;
}

template <typename T>
template <typename Deleter>
bool BorrowedPtr<T>::operator==(const OwnedPtr<T, Deleter>& other) const {
  return ptr_ == other.ptr_;
}

template <typename T>
template <typename Deleter>
bool BorrowedPtr<T>::operator!=(const OwnedPtr<T, Deleter>& other) const {
  return ptr_ != other.ptr_;
}

template <typename T>
bool BorrowedPtr<T>::operator==(std::nullptr_t) const {
  return ptr_ == nullptr;
}

template <typename T>
bool BorrowedPtr<T>::operator!=(std::nullptr_t) const {
  return ptr_ != nullptr;
}

template <typename T>
void BorrowedPtr<T>::AssertNotDestroyed() const {
  // If the ref is destroyed, but the ptr_ is still non-null, then the OwnedPtr
  // was destroyed while the BorrowedPtr was still outstanding. If the ptr_ is
  // null, then this is just an empty BorrowedPtr.
  if (ref_.IsCounterDestroyed() && ptr_) {
    SmallSourceLocation loc = ref_.GetLocation();

    IMP_LOG(imp::FATAL) << "Can't access destroyed BorrowedPtr of type "
               << type_traits::kTypeName<T> << " borrowed from "
               << loc.GetFileName() << ":" << loc.GetLineNumber();
  }
}
template <typename T, typename Deleter>
BorrowedPtr<T> OwnedPtr<T, Deleter>::Borrow(SmallSourceLocation loc) const {
  if (!ptr_) {
    IMP_LOG(imp::FATAL) << "Borrow called on empty OwnedPtr";
  }
  return BorrowedPtr<T>(ptr_, additional_fields_.GetRefCounter().Retain(loc));
}

template <typename T, typename Deleter>
uint16_t OwnedPtr<T, Deleter>::GetBorrowedCount() const {
  return additional_fields_.GetRefCounter().GetCount();
}

template <typename T, typename Deleter>
void OwnedPtr<T, Deleter>::Reset() {
  // If there are any outstanding borrowed objects, then the OwnedPtr can't be
  // safely destroyed because it will cause the borrowed objects to become
  // dangling pointers.
  //
  // This will log a fatal error if there are any outstanding borrowed objects,
  // or a log with a different severity if the severity has been overridden.
  if (ptr_) {
    if (additional_fields_.GetRefCounter().GetCount() > 0) {
      std::string description;
      if constexpr (type_traits::kHasGetNameMethod<T>) {
        static constexpr size_t kMaxNameLength = 256;
        static constexpr absl::string_view kEllipsis = "...";
        absl::string_view name = ptr_->GetName();
        description = absl::StrFormat(
            "of type %s named %s%s", type_traits::kTypeName<T>,
            name.length() > kMaxNameLength
                ? name.substr(0, kMaxNameLength - kEllipsis.length())
                : name,
            name.length() > kMaxNameLength ? kEllipsis : "");
      } else {
        description = absl::StrFormat("of type %s", type_traits::kTypeName<T>);
      }

      const RefCounter::TrackedRefs& tracked_refs =
          additional_fields_.GetRefCounter().GetTrackedRefs();
      std::string borrowed_locations;
      if (!tracked_refs.locations_to_counts.empty()) {
        for (const auto& [loc, counter] : tracked_refs.locations_to_counts) {
          absl::StrAppendFormat(&borrowed_locations, "  %d from %s:%d\n",
                                counter, loc.GetFileName(),
                                loc.GetLineNumber());
        }
      }

      std::string log_message = absl::StrFormat(
          "OwnedPtr %s destroyed with %d outstanding borrowed objects from "
          "the following locations:\n%s",
          description, additional_fields_.GetRefCounter().GetCount(),
          borrowed_locations);

      // It would be cleaner to use IMP_LOG(imp::LEVEL(severity)) instead of a switch
      // statement, but we can't because it doesn't currently work in the bazel
      // version of Impress.
      switch (GetOwnedPtrLogSeverity()) {
        case absl::LogSeverity::kFatal:
          IMP_LOG(imp::FATAL) << log_message;
          break;
        case absl::LogSeverity::kError:
          IMP_LOG(imp::ERROR) << log_message;
          break;
        case absl::LogSeverity::kWarning:
          IMP_LOG(imp::WARNING) << log_message;
          break;
        case absl::LogSeverity::kInfo:
          IMP_LOG(imp::INFO) << log_message;
          break;
      }
    }

    // Destroy the object.
    additional_fields_.GetDeleter()(ptr_);
    ptr_ = nullptr;
    additional_fields_ = {};
  }
}

// Creates an OwnedPtr for a new instance of T.
//
// Similar to std::make_unique, but returns an OwnedPtr instead of a
// std::unique_ptr.
template <typename T, typename... Args>
OwnedPtr<T> MakeOwned(Args&&... args) {
  return OwnedPtr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_H_
