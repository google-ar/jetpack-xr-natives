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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_PROTO_VISITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_PROTO_VISITOR_H_

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/bit_flag.h"
#include "core/common/copyable_ptr.h"
#include "core/common/template_helpers.h"
#include "core/editor/editor_field_control.h"
#include "core/editor/editor_style.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/widgets/asset_type_helpers.h"
#include "core/proto/imp_editor.proto.imp.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_reader.h"
#include "core/proto/proto_writer.h"
#include "core/resources/resource_manager.h"

namespace imp::editor {

// The unique string for ImGui drag-and-drop payloads when dragging a vector
// field element.
constexpr absl::string_view kVectorFieldElementDragAndDropPayloadId =
    "VectorElement";

constexpr ImVec2 kVectorFieldElementDragAndDropTargetSize = ImVec2(256, 8);

namespace internal {
template <int field_type, typename T>
constexpr bool kIsImpressProtoMessage =
    field_type == imp::proto::TYPE_MESSAGE &&
    !proto_traits::kIsStandardProto<T>;
}

// A helper class to visit all fields of an Impress proto and show UI.
template <typename Proto>
class EditorProtoVisitor {
 public:
  using Cursor = int;

  explicit EditorProtoVisitor(Proto& state) : state_(state) {}

  template <int field_type, typename T>
  Cursor Visit(
      Cursor field_index, int field_id, T* field, T* other,
      EditorControlFlags editor_control_flags = EditorControlFlags::kDefault) {
    if (IsFieldDisabled(field_id)) {
      return ++field_index;
    }

    if (IsFieldReadonly(field_id)) {
      ImGui::BeginDisabled();
    }

    if constexpr (field_type == imp::proto::TYPE_MESSAGE &&
                  !EditorFieldControl::is_handled_type<T>::value) {
      // Skip protos that don't use Impress code generation.
      // TODO: support editing types like GltfNodeSceneHandle.
      if constexpr (imp::proto::HasFields<T>::value) {
        // Show a label for the nested field name if specified, indent, and
        // recurse.
        if (CheckBit(editor_control_flags, EditorControlFlags::kDisplayLabel)) {
          ImGui::Text("%s", proto::GetFieldName<Proto>(field_id).data());
          if (other) {
            updated_ |= EditorFieldControl::RevertToBasePopup(*field, *other,
                                                              "Message");
          }
          ImGui::Indent();

          CreateDragAndDropSourceForField(field);
          updated_ |= CreateDragAndDropTargetForField(field);
        }

        EditorProtoVisitor<T> nested_visitor(*field);
        ::imp::proto::VisitPaired(field, &nested_visitor, 0, other);
        updated_ |= nested_visitor.AnyFieldEdited();
        if (CheckBit(editor_control_flags, EditorControlFlags::kDisplayLabel)) {
          ImGui::Unindent();
        }
      }
    } else {
      if constexpr (field_type == imp::proto::TYPE_ENUM) {
        // Handle the case where the field is an enum.
        updated_ |= ShowEnumControl(field_index, field_id, field, other,
                                    editor_control_flags);
      } else {
        // Otherwise, just show the control for this field.
        updated_ |= ShowControl(field_index, field_id, field, other,
                                editor_control_flags);
      }
    }

    if (IsFieldReadonly(field_id)) {
      ImGui::EndDisabled();
    }
    return ++field_index;
  }

  // Helper for handling the UI for both std::optional and CopyablePtr.
  template <int field_type, typename T, typename OptionalT, typename HasValueFn,
            typename GetValueFn, typename AssignOptionalFn>
  Cursor VisitOptionalHelper(Cursor field_index, int field_id, OptionalT* field,
                             OptionalT* other, HasValueFn has_value_fn,
                             GetValueFn get_value_fn,
                             AssignOptionalFn assign_optional_fn) {
    if (IsFieldDisabled(field_id)) {
      return ++field_index;
    }

    if (IsFieldReadonly(field_id)) {
      ImGui::BeginDisabled();
    }

    if (!has_value_fn(field) && other && has_value_fn(other)) {
      assign_optional_fn(field, *get_value_fn(other));
      updated_ = true;
    }

    // Use the same button size for both the + and - buttons to make the UI look
    // consistent.
    ImVec2 button_size = ImGui::CalcTextSize("+");
    button_size.x += ImGui::GetStyle().FramePadding.x * 2.0f;
    button_size.y += ImGui::GetStyle().FramePadding.y * 2.0f;

    if (!has_value_fn(field)) {
      bool pushed_style = false;
      if (other && !has_value_fn(other)) {
        editor::PushBaseIsfElementStyle();
        pushed_style = true;
      }

      // If the optional field is not set, show the field label and a button
      // for emplacing a default object of type T.
      if (ImGui::Button(
              absl::StrFormat("+##OptionalFieldEmplace%p", field).c_str(),
              button_size)) {
        assign_optional_fn(field, T());
        updated_ = true;
      }

      ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x);

      ImGui::Text("%s", proto::GetFieldName<Proto>(field_id).data());

      if (pushed_style) {
        editor::PopBaseIsfElementStyle();
      }
    } else {
      bool did_show_button = false;
      if (!other || !has_value_fn(other)) {
        // If the optional field is set, show a button for resetting the field.
        // But if the base field has a value then it can't be cleared, so this
        // gets skipped.
        if (ImGui::Button(
                absl::StrFormat("-##OptionalFieldClear%p", field).c_str(),
                button_size)) {
          field->reset();
          updated_ = true;
        }
        did_show_button = true;
      }

      if (has_value_fn(field)) {
        if (did_show_button) {
          ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x);
        }

        T* base = other && has_value_fn(other) ? get_value_fn(other) : nullptr;

        // When handling optional fields with a 'base' value for comparison, we
        // need to treat primitive and message types differently to comply with
        // the proto3 spec on field presence (see
        // (broken link)).
        //
        //  - Optional primitive: Setting it to the default value (e.g., 0) is a
        //    valid state and should not cause it to be reset to "unset".
        //
        //  - Optional message: We need to reset its default fields to match
        //    the base message because the proto3 spec can't distinguish
        //    between a default-valued and an unset field within that message.
        //
        // These flags enforce that distinction.
        EditorControlFlags editor_control_flags =
            field_type == imp::proto::TYPE_MESSAGE
                ? EditorControlFlags::kDefault
                : EditorControlFlags::kDefaultWithoutResetUnsetValToBase;

        Visit<field_type, T>(field_index, field_id, get_value_fn(field), base,
                             editor_control_flags);
      }
    }

    if constexpr (internal::kIsImpressProtoMessage<field_type, T>) {
      // Allow drag-and-drop from the Asset Library into an optional.
      T maybe_emplaced;
      bool should_emplace = CreateDragAndDropTargetForField(&maybe_emplaced);
      if (should_emplace) {
        assign_optional_fn(field, std::move(maybe_emplaced));
        updated_ = true;
      }
    }

    if (IsFieldReadonly(field_id)) {
      ImGui::EndDisabled();
    }
    return ++field_index;
  }

  // Handles the case where field is a std::optional.
  template <int field_type, typename T>
  Cursor Visit(Cursor field_index, int field_id, std::optional<T>* field,
               std::optional<T>* other) {
    return VisitOptionalHelper<field_type, T>(
        field_index, field_id, field, other,
        +[](std::optional<T>* field) { return field->has_value(); },
        +[](std::optional<T>* field) { return &field->value(); },
        +[](std::optional<T>* field, T assignment) {
          field->emplace(std::move(assignment));
        });
  }

  // Handles the case where the field is a CopyablePtr
  template <int field_type, typename T>
  Cursor Visit(Cursor field_index, int field_id, CopyablePtr<T>* field,
               CopyablePtr<T>* other) {
    return VisitOptionalHelper<field_type, T>(
        field_index, field_id, field, other,
        +[](CopyablePtr<T>* field) { return field->get() != nullptr; },
        +[](CopyablePtr<T>* field) { return field->get(); },
        +[](CopyablePtr<T>* field, T assignment) {
          field->reset(new T(std::move(assignment)));
        });
  }

  // Handles the case where field is a vector.
  // TODO: Add unit tests for vector component editor widget
  template <int field_type, proto::RepeatedMergeStrategy merge_strategy,
            typename T>
  Cursor Visit(Cursor field_index, int field_id, std::vector<T>* field,
               std::vector<T>* other) {
    if (IsFieldDisabled(field_id)) {
      return ++field_index;
    }

    if (IsFieldReadonly(field_id)) {
      ImGui::BeginDisabled();
    }

    // Show a label for the vector field name, indent, and recursively visit
    // each element.
    ImGui::Text("%s", proto::GetFieldName<Proto>(field_id).data());
    if (other) {
      updated_ |=
          EditorFieldControl::RevertToBasePopup(*field, *other, "Vector");
    }
    ImGui::Indent();

    std::optional<size_t> drag_and_drop_index;

    std::string payload_type_id = absl::StrFormat(
        "%s_%p", kVectorFieldElementDragAndDropPayloadId, field);

    if (ImGui::GetDragDropPayload()) {
      drag_and_drop_index.emplace(
          *static_cast<size_t*>(ImGui::GetDragDropPayload()->Data));
    }

    // In kOverwrite, either the whole vector matches the base or none of it
    // does.
    bool does_full_vector_match_base = false;
    if (other && merge_strategy == proto::RepeatedMergeStrategy::kOverwrite) {
      does_full_vector_match_base = other->size() == field->size();
      if (does_full_vector_match_base) {
        for (size_t i = 0; i < field->size(); ++i) {
          does_full_vector_match_base =
              EditorFieldControl::CompareField((*field)[i], (*other)[i]);
          if (!does_full_vector_match_base) break;
        }
      }
    }

    if (does_full_vector_match_base) {
      editor::PushBaseIsfElementStyle();
    }

    for (size_t i = 0; i < field->size(); ++i) {
      T* element = &(*field)[i];
      T* base_element =
          merge_strategy != proto::RepeatedMergeStrategy::kOverwrite && other &&
                  other->size() > i
              ? &(*other)[i]
              : nullptr;

      // In kPerElement and kConcat, the element can't be re-arranged if it
      // comes from the base.
      bool can_rearrange_element = true;
      if ((merge_strategy == proto::RepeatedMergeStrategy::kPerElement ||
           merge_strategy == proto::RepeatedMergeStrategy::kConcat) &&
          other && i < other->size()) {
        can_rearrange_element = false;
      }
      // If the merge strategy is kConcat, the element inherited from the base
      // can't be edited.
      bool can_edit = can_rearrange_element ||
                      merge_strategy != proto::RepeatedMergeStrategy::kConcat;

      // Drag-and-drop target for inserting before.
      ImGui::Dummy(kVectorFieldElementDragAndDropTargetSize);
      // Check to only enable drag targets for elements that are not the source
      // or the element right after the source.
      if (can_rearrange_element && drag_and_drop_index.has_value() &&
          *drag_and_drop_index != i && *drag_and_drop_index + 1 != i &&
          ImGui::BeginDragDropTarget()) {
        if (ImGui::AcceptDragDropPayload(payload_type_id.c_str())) {
          std::size_t element_index =
              *static_cast<size_t*>(ImGui::GetDragDropPayload()->Data);
          T& source = (*field)[element_index];
          field->insert(field->begin() + i, source);
          if (i < element_index) {
            element_index += 1;
          }
          field->erase(field->begin() + element_index);
          drag_and_drop_index.reset();
          updated_ = true;
        }

        if constexpr (internal::kIsImpressProtoMessage<field_type, T>) {
          // Allow drag-and-drop from the Asset Library and inserting.
          T maybe_insert;
          bool should_insert = AcceptDragAndDropPayloadForField(&maybe_insert);
          if (should_insert) {
            field->insert(field->begin() + i, maybe_insert);
            updated_ = true;
          }
        }

        ImGui::EndDragDropTarget();
      }

      ImGui::Text("%s", absl::StrFormat("Element %d", i).c_str());
      if (base_element) {
        updated_ |= EditorFieldControl::RevertToBasePopup(
            *element, *base_element, "VectorElement");
      }

      if constexpr (internal::kIsImpressProtoMessage<field_type, T>) {
        // Allow drag-and-drop from the Asset Library to replace an element.
        updated_ |= CreateDragAndDropTargetForField(element);
      }

      if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
        ImGui::SetDragDropPayload(payload_type_id.c_str(), &i, sizeof(size_t));
        drag_and_drop_index.emplace(i);
        ImGui::Text("%s[%zu]", proto::GetFieldName<Proto>(field_id).data(), i);
        ImGui::EndDragDropSource();
      }

      if (can_rearrange_element) {
        ImGui::SameLine();
        if (ImGui::Button(
                absl::StrFormat("X##VectorFieldRemoveElement%p", element)
                    .c_str())) {
          field->erase(field->begin() + i);
          i -= 1;
          updated_ = true;
          continue;
        }
      }

      ImGui::Indent();

      EditorControlFlags editor_control_flags;
      if (can_edit) {
        editor_control_flags = EditorControlFlags::kIsEditable;
      } else {
        editor_control_flags = EditorControlFlags::kNone;
      }
      Visit<field_type, T>(field_index, field_id, element, base_element,
                           editor_control_flags);
      ImGui::Unindent();

      // Drag-and-drop target for inserting after the last element.
      if (i + 1 == field->size()) {
        ImGui::Dummy(kVectorFieldElementDragAndDropTargetSize);
        if (can_rearrange_element && drag_and_drop_index.has_value() &&
            *drag_and_drop_index != i && ImGui::BeginDragDropTarget()) {
          if (ImGui::AcceptDragDropPayload(payload_type_id.c_str())) {
            std::size_t element_index =
                *static_cast<size_t*>(ImGui::GetDragDropPayload()->Data);
            T& source = (*field)[element_index];
            field->insert(field->begin() + i + 1, source);
            if (i + 1 < element_index) {
              element_index += 1;
            }
            field->erase(field->begin() + element_index);
            drag_and_drop_index.reset();
            updated_ = true;
          }

          // Note: intentionally not adding Asset Library support here as it
          // is redundant with dragging onto the "Add Element" button below.

          ImGui::EndDragDropTarget();
        }
      }
    }

    if (does_full_vector_match_base) {
      editor::PopBaseIsfElementStyle();
    }

    if (ImGui::Button(
            absl::StrFormat("Add Element##VectorFieldAddElement%p", field)
                .c_str())) {
      field->push_back(T());
      updated_ = true;
    }

    if constexpr (internal::kIsImpressProtoMessage<field_type, T>) {
      // Allow drag-and-drop from the Asset Library onto the Add Element button.
      T maybe_insert;
      if (CreateDragAndDropTargetForField(&maybe_insert)) {
        field->push_back(maybe_insert);
        updated_ = true;
      }
    }

    ImGui::Unindent();

    if (IsFieldReadonly(field_id)) {
      ImGui::EndDisabled();
    }
    return ++field_index;
  }

  template <int field_type_key, int field_type_value, typename K, typename V>
  Cursor Visit(Cursor field_index, int field_id, std::map<K, V>* field,
               std::map<K, V>* other) {
    if (IsFieldDisabled(field_id)) {
      return ++field_index;
    }

    if (IsFieldReadonly(field_id)) {
      ImGui::BeginDisabled();
    }

    ImGui::Text("%s", proto::GetFieldName<Proto>(field_id).data());
    if (other) {
      updated_ |= EditorFieldControl::RevertToBasePopup(*field, *other, "Map");
    }
    ImGui::Indent();

    for (auto iter = field->begin(); iter != field->end();) {
      K key = iter->first;
      V* value = &iter->second;
      V* base = nullptr;
      if (other) {
        auto other_iter = other->find(key);
        if (other_iter != other->end()) {
          base = &other_iter->second;
        }
      }

      Visit<field_type_key, K>(field_index, field_id, &key,
                               static_cast<K*>(nullptr),
                               EditorControlFlags::kNone);

      if (base) {
        updated_ |=
            EditorFieldControl::RevertToBasePopup(*value, *base, "MapElement");
      }

      ImGui::SameLine();
      ImGui::Text(": ");

      ImGui::Indent();

      Visit<field_type_value, V>(field_index, field_id, value, base,
                                 EditorControlFlags::kIsEditable);

      if constexpr (internal::kIsImpressProtoMessage<field_type_value, V>) {
        // Allow drag-and-drop from the Asset Library onto map values.
        if (CreateDragAndDropTargetForField(value)) {
          updated_ = true;
        }
      }

      bool did_remove_element = false;
      if (base == nullptr) {
        ImGui::SameLine();
        if (ImGui::Button(
                absl::StrFormat("X##MapFieldRemoveKey%p", &iter->first)
                    .c_str())) {
          iter = field->erase(iter);
          did_remove_element = true;
        }
      }

      if (!did_remove_element) {
        ++iter;
      }
      ImGui::Unindent();
    }

    ImGui::Text("Add key to map");

    if constexpr (internal::kIsImpressProtoMessage<field_type_key, K>) {
      // Allow drag-and-drop from the Asset Library into new keys.
      K maybe_insert;
      if (CreateDragAndDropTargetForField(&maybe_insert)) {
        (*field)[maybe_insert] = V();
        updated_ = true;
      }
    }

    static K editable_key;
    Visit<field_type_key, K>(field_index, field_id, &editable_key,
                             static_cast<K*>(nullptr),
                             EditorControlFlags::kIsEditable);
    if (ImGui::Button(absl::StrFormat("Add##MapAddKV%p", field).c_str())) {
      (*field)[editable_key] = V();
      editable_key = K();
    }

    ImGui::Unindent();
    if (IsFieldReadonly(field_id)) {
      ImGui::EndDisabled();
    }
    return ++field_index;
  }

  template <typename T, typename FieldType, FieldType... field_types>
  Cursor VisitVariant(
      Cursor field_index, T* field, T* other, absl::string_view field_name,
      std::integer_sequence<int, field_types...> variant_field_types,
      const std::vector<int>& variant_field_ids) {
    // First, determine if the field matches the base.
    bool does_match_base_type = other && other->index() == field->index();
    bool is_base_set = other && other->index() != 0;

    if (does_match_base_type) {
      editor::PushBaseIsfElementStyle();
    }

    ImGui::Text("%s:", field_name.data());
    // TODO: Support RevertToBasePopup for variants that contain a
    // unique_ptr. This is a bit tricky because the unique_ptr needs to be
    // deep copied, which we support via a copy assignment operator on the
    // generated Message type but not on the variant itself. Luckily, this is a
    // rare case.
    if constexpr (std::is_copy_assignable_v<T>) {
      if (other) {
        updated_ |=
            EditorFieldControl::RevertToBasePopup(*field, *other, "Variant");
      }
    }

    // Second, ensure all protobuf message types can be dragged onto the combo.
    // Monostate (unset) is index 0, so start at 1.
    ForConstexpr<1, std::variant_size_v<T>>(
        [this, field, &variant_field_types](auto i) mutable {
          constexpr int field_type = GetAt<i - 1>(variant_field_types);
          using FieldT = std::decay_t<std::variant_alternative_t<i, T>>;
          if constexpr (internal::kIsImpressProtoMessage<field_type, FieldT>) {
            // Allow drag-and-drop from the Asset Library into this oneof.
            FieldT maybe_set;
            bool should_set = CreateDragAndDropTargetForField(&maybe_set);
            if (should_set) {
              field->template emplace<i>(std::move(maybe_set));
              updated_ = true;
            }
          }
        });

    bool field_set = field->index() != 0;
    bool can_unset = field_set && !is_base_set;
    absl::string_view combo_text =
        !field_set
            ? "select type..."
            : proto::GetFieldName<Proto>(variant_field_ids[field->index() - 1]);
    ImGui::SameLine();
    if (ImGui::BeginCombo(
            GenerateUniqueImGuiLabel("combo", field, EditorControlFlags::kNone)
                .c_str(),
            combo_text.data(), ImGuiComboFlags_None)) {
      // Next, populate the combo box with all the types in the oneof.
      ForConstexpr<0, std::variant_size_v<T>>(
          [this, can_unset, field, &variant_field_ids](auto i) mutable {
            bool selected = field->index() == i;
            absl::string_view label_str;
            // Index 0 is std::monostate (unset), so skip if the field is unset.
            if (i == 0) {
              if (!can_unset) return;
              label_str = "unset";
            } else {
              // The variant field_ids don't include monostate, so decrement.
              int variant_field_id = variant_field_ids[i - 1];
              label_str = proto::GetFieldName<Proto>(variant_field_id);
            }

            ImGui::Selectable(
                GenerateUniqueImGuiLabel(label_str, field).c_str(), &selected);
            if (selected && field->index() != i) {
              using FieldT = typename std::remove_const<
                  std::decay_t<std::variant_alternative_t<i, T>>>::type;
              field->template emplace<i>(FieldT{});
              updated_ = true;
            }
          });
      ImGui::EndCombo();
    }

    if (does_match_base_type) {
      editor::PopBaseIsfElementStyle();
    }

    ImGui::Indent();
    // If the field is set, recurse into Visit on the set field type.
    if (field_set) {
      // Monostate (unset) is index 0, so start at 1.
      ForConstexpr<1, std::variant_size_v<T>>(
          [this, field_index, field, other, &variant_field_types,
           &variant_field_ids](auto i) mutable {
            // Only display the currently set field.
            if (field->index() != i) return;
            // The variant field_ids don't include monostate, so decrement.
            int variant_field_id = variant_field_ids[i - 1];

            using VariantAltT =
                typename std::variant_alternative_t<i, std::decay_t<T>>;
            VariantAltT* element = &std::get<i>(*field);
            VariantAltT* base = std::get_if<i>(other);

            Visit<GetAt<i - 1>(variant_field_types)>(
                field_index + i, variant_field_id, element, base,
                static_cast<EditorControlFlags>(
                    EditorControlFlags::kIsEditable |
                    EditorControlFlags::kResetUnsetValToBase));
          });
    }
    ImGui::Unindent();

    // Oneofs guaranteed to be contiguous.
    return field_index + std::variant_size_v<T>;
  }

  // TODO: Support standard (non-impress) protos.
  template <typename StandardProto>
  Cursor VisitStandardProto(Cursor field_index, int field_id,
                            StandardProto* proto, StandardProto* other) {
    return ++field_index;
  }

  Cursor Unknown(Cursor field_index) { return 0; }

  // Resets the bool that tracks whether any field was updated.
  void Reset() { updated_ = false; }

  // Returns true if any field was updated during this visit.
  // TODO: should we return the specific field that was changed?
  bool AnyFieldEdited() { return updated_; }

 private:
  template <typename FieldT>
  bool ShowControl(
      size_t field_index, int field_id, FieldT* val, FieldT* other,
      EditorControlFlags editor_control_flags = EditorControlFlags::kDefault) {
    absl::string_view field_name = proto::GetFieldName<Proto>(field_id);
    std::optional<EditorControlType> editor_control_type =
        GetEditorControlTypeForField(field_id);
    if (editor_control_type.has_value() &&
        !std::holds_alternative<EditorControlReadonly>(
            editor_control_type->type)) {
      return std::visit(
          [field_name, val, other, editor_control_flags](auto&& control) {
            // TODO: figure out why this is not working.
            // using T = std::decay_t<decltype(control)>;
            // if constexpr (std::is_same_v<T, absl::monostate>) {
            //   IMP_LOG(imp::FATAL) << "Invalid unset/empty EditorControlType on field "
            //   << field_name;
            // } else {
            absl::StatusOr<bool> result = EditorFieldControl::ShowControl(
                field_name, control, val, other, editor_control_flags);
            if (!result.ok()) {
              IMP_LOG(imp::FATAL) << result.status();
            }
            return *result;
            //}
          },
          editor_control_type->type);
    } else {
      return EditorFieldControl::ShowDefaultControl(field_name, val, other,
                                                    editor_control_flags);
    }
  }

  template <typename E>
  bool ShowEnumControl(
      size_t field_index, int field_id, E* val, E* base,
      EditorControlFlags editor_control_flags = EditorControlFlags::kDefault) {
    absl::string_view field_name = proto::GetFieldName<Proto>(field_id);
    return EditorFieldControl::ShowEnumControl(field_name, val, base,
                                               editor_control_flags);
  }

  std::optional<EditorControlType> GetEditorControlTypeForField(int field_id) {
    size_t field_index = proto::GetFieldIndex<Proto>(field_id);
    std::string editor_control_type_serialized;
    absl::Base64Unescape(state_.kFieldEditorControlTypes[field_index],
                         &editor_control_type_serialized);
    if (editor_control_type_serialized.empty()) {
      return std::nullopt;
    }
    EditorControlType editor_control_type;
    imp::proto::ParseMessage(editor_control_type_serialized,
                             &editor_control_type);
    return editor_control_type;
  }

  bool IsFieldDisabled(int field_id) {
    std::optional<EditorControlType> editor_control_type =
        GetEditorControlTypeForField(field_id);
    return editor_control_type.has_value() &&
           std::holds_alternative<EditorControlDisabled>(
               editor_control_type->type);
  }

  bool IsFieldReadonly(int field_id) {
    std::optional<EditorControlType> editor_control_type =
        GetEditorControlTypeForField(field_id);
    return editor_control_type.has_value() &&
           std::holds_alternative<EditorControlReadonly>(
               editor_control_type->type);
  }

  // Accepts a drag-and-drop payload from Asset Library matching the field type.
  template <typename T>
  bool AcceptDragAndDropPayloadForField(T* field) {
    // Add a drag-and-drop target for this message.
    std::string payload_type = GetPayloadTypeForMessage<T>();
    std::optional<std::string> payload = AcceptDragAndDropPayload(payload_type);
    if (!payload.has_value()) {
      return false;
    }

    // First, check if this message is an embedded resource.
    absl::StatusOr<resources::Resource> proto_resource =
        resources::ResourceManager::LoadEmbeddedResource(*payload);
    if (proto_resource.ok()) {
      // Make sure field is reset before we deserialize into it.
      *field = T();
      if (!proto::ParseMessage(proto_resource->GetData().StringView(), field)) {
        IMP_LOG(imp::FATAL) << "Parsing proto message of type " << payload_type
                   << " failed! This is a coding error.";
      }
      return true;
    } else {
      // Try deserializing directly.
      T deserialized;
      if (proto::ParseMessage(*payload, &deserialized)) {
        *field = std::move(deserialized);
        return true;
      }
    }
    return false;
  }

  // Creates a drag-and-drop target and accepts a payload from Asset Library
  // matching the field type.
  template <typename T>
  bool CreateDragAndDropTargetForField(T* field) {
    bool updated = false;
    if (ImGui::BeginDragDropTarget()) {
      updated = AcceptDragAndDropPayloadForField(field);
      ImGui::EndDragDropTarget();
    }
    return updated;
  }

  template <typename T>
  void CreateDragAndDropSourceForField(const T* field) {
    if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
      std::string payload_type = GetPayloadTypeForMessage<T>();
      std::string payload;
      proto::SerializeTo(field, &payload);
      ImGui::SetDragDropPayload(
          absl::StrCat(kProtoDragAndDropScheme, payload_type).c_str(),
          payload.data(), payload.size());
      ImGui::Text("apply");
      ImGui::EndDragDropSource();
    }
  }

  Proto& state_;
  bool updated_ = false;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_PROTO_VISITOR_H_
