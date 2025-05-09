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

#ifndef THIRD_PARTY_IMPRESS_PROTOTYPES_SAMPLES_EDITOR_EDITOR_FIELD_CONTROL_H_
#define THIRD_PARTY_IMPRESS_PROTOTYPES_SAMPLES_EDITOR_EDITOR_FIELD_CONTROL_H_

#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "dear_imgui/misc/cpp/imgui_stdlib.h"
#include "core/common/bit_flag.h"
#include "core/editor/editor_style.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/layout/helpers.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/ui/drag_and_drop_node.h"
#include "core/geometry/shapes/box.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/imp_editor.proto.imp.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_differ.h"
#include "core/scene_handles/scene_handle_interface.h"

namespace imp {

// Forward declare NodeSceneHandle to avoid a circular dependency.
// TODO: Add support for GltfNodeSceneHandle in the editor.
class NodeSceneHandle;
template <typename T>
class ComponentSceneHandle;

// Collection of helper static methods to create ImGui UI to edit fields
// specified from an Impress proto field with an option specified here:
// (broken link)
class EditorFieldControl {
 public:
  // List all handled "primitive" types of fields so we can determine if a field
  // type needs to be recursed, i.e. it is a nested message.
  template <typename T>
  struct is_handled_type : std::false_type {};
  template <>
  struct is_handled_type<float2> : std::true_type {};
  template <>
  struct is_handled_type<float3> : std::true_type {};
  template <>
  struct is_handled_type<float4> : std::true_type {};
  template <>
  struct is_handled_type<Box> : std::true_type {};
  template <>
  struct is_handled_type<uint32_t> : std::true_type {};
  template <>
  struct is_handled_type<int32_t> : std::true_type {};
  template <>
  struct is_handled_type<float> : std::true_type {};
  template <>
  struct is_handled_type<double> : std::true_type {};
  template <>
  struct is_handled_type<std::string> : std::true_type {};
  template <>
  struct is_handled_type<bool> : std::true_type {};

  template <>
  struct is_handled_type<NodeSceneHandle> : std::true_type {};
  template <typename T>
  struct is_handled_type<ComponentSceneHandle<T>> : std::true_type {};

  template <typename ControlT, typename FieldT>
  static absl::StatusOr<bool> ShowControl(
      const std::string& name, ControlT& control, FieldT* val, FieldT* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "Cannot use editor control type %s with field of type %s",
        type_traits::kTypeName<ControlT>, type_traits::kTypeName<FieldT>));
  }

  // Helper for comparing two fields within a component state.
  //
  // If the fields are of a proto message type then the fields are compared
  // using proto::ProtoDiffer::CompareField.
  //
  // If the fields are of a primitive type then the fields are compared using
  // the == operator.
  template <typename T>
  static bool CompareField(T& val, T& base) {
    constexpr bool kHasFields = proto::HasFields<T>::value;

    bool result = false;

    if constexpr (kHasFields) {
      if (proto::CheckIfFieldsMatch(val, base) ==
          proto::ProtoDiffer::Result::kFoundAllFieldsMatch) {
        result = true;
      }
    } else {
      if (val == base) {
        result = true;
      }
    }

    return result;
  }

  template <typename T>
  static std::string ElementPopupLabel(T& val,
                                       absl::string_view extra_label = "") {
    std::string label_name = absl::StrCat("ElementPopup", extra_label);
    std::string label = editor::GenerateUniqueImGuiLabel(
        label_name, &val, editor::EditorControlFlags::kNone);
    return label;
  }

  // Helper to show a popup menu to revert a field to its base value.
  template <typename T>
  static bool RevertToBasePopup(T& val, T& base,
                                absl::string_view extra_label = "") {
    bool result = false;

    std::string label = ElementPopupLabel(val, extra_label);
    if (ImGui::BeginPopupContextItem(label.c_str())) {
      if (ImGui::MenuItem("Revert To Base", nullptr, &result, true)) {
        val = base;
      }
      ImGui::EndPopup();
    }

    return result;
  }

  enum class EditingElementMode {
    // Indicates that the element being edited will be displayed normally.
    kNormal,
    // Indicates that the element being edited will be displayed with a
    // different style to indicate that it matches the base value.
    kMatchesBase
  };

  // Call just before editing an element within a component state to handle if
  // the element matches the base value.
  //
  // If the element matches the base value then the element will be displayed
  // with a different style to indicate that it matches the base value.
  template <typename T>
  static EditingElementMode BeginEditingElement(
      T* val, T* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    EditingElementMode result = EditingElementMode::kNormal;

    if (base) {
      // A field in a proto cannot be overridden to be unset because unset and
      // default cannot be distinguished in the encoded format. Since this case
      // can't be serialized out, detect it and revert to the base value.
      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kResetUnsetValToBase)) {
        T unset{};
        if (CompareField(*val, unset)) {
          result = EditingElementMode::kMatchesBase;

          if (!CompareField(*val, *base)) {
            // TODO: Indicate to the user that the field was
            // reverted to the base value and why.
            *val = *base;
          }
        }
      }

      if (result != EditingElementMode::kMatchesBase) {
        if (CompareField(*val, *base)) {
          result = EditingElementMode::kMatchesBase;
        }
      }
    }

    if (result == EditingElementMode::kMatchesBase) {
      editor::PushBaseIsfElementStyle();
    }

    return result;
  }

  // Called just after editing an element within a component state to handle if
  // the element matches the base value.
  //
  // Reverts back to the normal UI style if needed, and shows a popup menu to
  // revert the element to the base value.
  //
  // Returns true if val was edited
  template <typename T>
  static bool EndEditingElement(EditingElementMode mode, T* val, T* base) {
    if (mode == EditingElementMode::kMatchesBase) {
      editor::PopBaseIsfElementStyle();
    } else if (base) {
      return RevertToBasePopup(*val, *base);
    }

    return false;
  }

  template <typename ScalarT>
  static bool ShowDefaultControlForScalar(
      ImGuiDataType data_type, const std::string& name, ScalarT* val,
      ScalarT* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      std::string type_specifier;
      if constexpr (std::is_floating_point_v<ScalarT>) {
        type_specifier = "%f";
      } else {
        type_specifier = "%d";
      }

      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kDisplayLabel)) {
        std::string format = absl::StrCat(type_specifier, ": %s");
        ImGui::Text(format.c_str(), *val, name.c_str());
      } else {
        ImGui::Text(type_specifier.c_str(), *val);
      }
      return false;
    }

    EditingElementMode mode =
        BeginEditingElement(val, base, editor_control_flags);

    std::string label =
        editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags);

    bool result =
        ImGui::InputScalar(label.c_str(), data_type, val, nullptr, nullptr,
                           nullptr, ImGuiInputTextFlags_CharsScientific);

    result |= EndEditingElement(mode, val, base);

    return result;
  }

  // Default control for math vector fields like float2, float3, float4.
  template <typename VecT>
  static bool ShowDefaultVectorControl(
      const std::string& name, VecT* val, VecT* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      std::string text = "(";
      for (int i = 0; i < VecT::SIZE; ++i) {
        text = absl::StrCat(text, absl::StrFormat("%lf", (*val)[i]));
        if (i < VecT::SIZE - 1) {
          absl::StrAppend(&text, ", ");
        }
      }
      absl::StrAppend(&text, ")");
      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kDisplayLabel)) {
        absl::StrAppend(&text, ": ", name);
      }
      ImGui::Text(text.c_str());
      return false;
    }

    ImGui::PushID(val);

    bool result = false;

    ImGui::PushMultiItemsWidths(VecT::SIZE, ImGui::CalcItemWidth());
    for (int i = 0; i < VecT::SIZE; ++i) {
      float* val_element = &(*val)[i];
      float* base_element = base ? &(*base)[i] : nullptr;

      ImGui::PushID(i);

      EditingElementMode mode =
          BeginEditingElement(val_element, base_element, editor_control_flags);

      result |= ImGui::InputFloat("", val_element);
      ImGui::PopItemWidth();

      result |= EndEditingElement(mode, val_element, base_element);

      ImGui::PopID();

      ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x);
    }

    if (CheckBit(editor_control_flags,
                 editor::EditorControlFlags::kDisplayLabel)) {
      std::string label =
          editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags);
      ImGui::Text(name.c_str());
    }

    ImGui::PopID();

    return result;
  }

  template <typename FieldT>
  static absl::StatusOr<bool> ShowControl(
      const std::string& name, EditorControlDisabled& control, FieldT* val,
      FieldT* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return false;
  }

  // Full specialization for float slider.
  static absl::StatusOr<bool> ShowControl(
      const std::string& name, EditorControlSliderFloat& control, float* val,
      float* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kDisplayLabel)) {
        ImGui::Text("%lf: %s", *val, name.c_str());
      } else {
        ImGui::Text("%lf", *val);
      }
      return false;
    }

    EditingElementMode mode =
        BeginEditingElement(val, base, editor_control_flags);

    bool result = ImGui::SliderFloat(
        editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags)
            .c_str(),
        val, control.min, control.max,
        absl::StrCat("%.", control.precision, "f").c_str(),
        control.logarithmic ? ImGuiSliderFlags_Logarithmic
                            : ImGuiSliderFlags_None);

    result |= EndEditingElement(mode, val, base);

    return result;
  }

  // Full specialization for int slider.
  static absl::StatusOr<bool> ShowControl(
      const std::string& name, EditorControlSliderInt& control, int32_t* val,
      int32_t* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kDisplayLabel)) {
        ImGui::Text("%d: %s", *val, name.c_str());
      } else {
        ImGui::Text("%d", *val);
      }
      return false;
    }

    EditingElementMode mode =
        BeginEditingElement(val, base, editor_control_flags);

    bool result = ImGui::SliderInt(
        editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags)
            .c_str(),
        val, control.min, control.max);

    result |= EndEditingElement(mode, val, base);

    return result;
  }

  // Full specialization for float3 colors.
  static absl::StatusOr<bool> ShowControl(
      const std::string& name, EditorControlColor3& control, float3* val,
      float3* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kDisplayLabel)) {
        ImGui::Text("(%lf, %lf, %lf): %s", val->r, val->g, val->b,
                    name.c_str());
      } else {
        ImGui::Text("(%lf, %lf, %lf)", val->r, val->g, val->b);
      }
      return false;
    }

    editor::EditorControlFlags vector_control_flags = editor_control_flags;
    vector_control_flags = static_cast<editor::EditorControlFlags>(
        SetBitFromBool(vector_control_flags,
                       editor::EditorControlFlags::kDisplayLabel, false));
    bool result =
        ShowDefaultVectorControl(name, val, base, vector_control_flags);

    EditingElementMode mode =
        BeginEditingElement(val, base, editor_control_flags);

    result |= ImGui::ColorEdit3(
        editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags)
            .c_str(),
        val->v,
        ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoOptions |
            ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueBar);

    result |= EndEditingElement(mode, val, base);

    return result;
  }

  // Full specialization for float4 colors.
  static absl::StatusOr<bool> ShowControl(
      const std::string& name, EditorControlColor4& control, float4* val,
      float4* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kDisplayLabel)) {
        ImGui::Text("(%lf, %lf, %lf, %lf): %s", val->r, val->g, val->b, val->a,
                    name.c_str());
      } else {
        ImGui::Text("(%lf, %lf, %lf, %lf)", val->r, val->g, val->b, val->a);
      }
      return false;
    }

    editor::EditorControlFlags vector_control_flags = editor_control_flags;
    vector_control_flags = static_cast<editor::EditorControlFlags>(
        SetBitFromBool(vector_control_flags,
                       editor::EditorControlFlags::kDisplayLabel, false));
    bool result =
        ShowDefaultVectorControl(name, val, base, vector_control_flags);

    EditingElementMode mode =
        BeginEditingElement(val, base, editor_control_flags);

    result |= ImGui::ColorEdit4(
        editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags)
            .c_str(),
        val->v,
        ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoOptions |
            ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueBar |
            ImGuiColorEditFlags_AlphaBar | ImGuiColorEditFlags_AlphaPreview);

    result |= EndEditingElement(mode, val, base);

    return result;
  }

  // Specialization for string fields that accept material assets.
  static absl::StatusOr<bool> ShowControl(
      const std::string& name, EditorControlMaterial& control, std::string* val,
      std::string* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    bool edited = ShowDefaultControl(name, val, base, editor_control_flags);
    if (ImGui::BeginDragDropTarget()) {
      std::optional<std::string> payload =
          AcceptDragAndDropPayload(editor::DragAndDropType::kMaterial);
      if (payload.has_value()) {
        *val = *payload;
        return true;
      }
      ImGui::EndDragDropTarget();
    }
    return edited;
  }

  // Specialization for string fields that accept texture assets.
  static absl::StatusOr<bool> ShowControl(
      const std::string& name, EditorControlTexture& control, std::string* val,
      std::string* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    bool edited = ShowDefaultControl(name, val, base, editor_control_flags);
    if (ImGui::BeginDragDropTarget()) {
      std::optional<std::string> payload =
          AcceptDragAndDropPayload(editor::DragAndDropType::kTexture);
      if (payload.has_value()) {
        *val = *payload;
        return true;
      }
      ImGui::EndDragDropTarget();
    }
    return edited;
  }

  // General handler for unsupported types.
  static bool ShowDefaultControl(
      const std::string& name, void* val, void* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return false;
  }

  // Default control for float2 fields.
  static bool ShowDefaultControl(
      const std::string& name, float2* val, float2* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return ShowDefaultVectorControl(name, val, base, editor_control_flags);
  }

  // Default control for float3 fields.
  static bool ShowDefaultControl(
      const std::string& name, float3* val, float3* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return ShowDefaultVectorControl(name, val, base, editor_control_flags);
  }

  // Default control for float4 fields.
  static bool ShowDefaultControl(
      const std::string& name, float4* val, float4* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return ShowDefaultVectorControl(name, val, base, editor_control_flags);
  }

  // Default control for box fields.
  static bool ShowDefaultControl(
      const std::string& name, Box* val, Box* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    ImGui::Text(name.c_str());
    ImGui::Indent();

    bool result = ShowDefaultControl("center", &val->center,
                                     base ? &base->center : nullptr,
                                     editor_control_flags);
    result |= ShowDefaultControl("half extent", &val->halfExtent,
                                 base ? &base->halfExtent : nullptr,
                                 editor_control_flags);

    ImGui::Unindent();

    return result;
  }

  // Default control for uint32 fields.
  static bool ShowDefaultControl(
      const std::string& name, uint32_t* val, uint32_t* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return ShowDefaultControlForScalar(ImGuiDataType_U32, name, val, base,
                                       editor_control_flags);
  }

  // Default control for int32 fields.
  static bool ShowDefaultControl(
      const std::string& name, int32_t* val, int32_t* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return ShowDefaultControlForScalar(ImGuiDataType_S32, name, val, base,
                                       editor_control_flags);
  }

  // Default control for float fields.
  static bool ShowDefaultControl(
      const std::string& name, float* val, float* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return ShowDefaultControlForScalar(ImGuiDataType_Float, name, val, base,
                                       editor_control_flags);
  }

  // Default control for double fields.
  static bool ShowDefaultControl(
      const std::string& name, double* val, double* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    return ShowDefaultControlForScalar(ImGuiDataType_Double, name, val, base,
                                       editor_control_flags);
  }

  // Default control for string fields.
  static bool ShowDefaultControl(
      const std::string& name, std::string* val, std::string* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kDisplayLabel)) {
        ImGui::Text("%s: %s", val->c_str(), name.c_str());
      } else {
        ImGui::Text("%s", val->c_str());
      }
      return false;
    }

    EditingElementMode mode =
        BeginEditingElement(val, base, editor_control_flags);

    std::string label =
        editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags);
    bool result = ImGui::InputText(label.c_str(), val);

    result |= EndEditingElement(mode, val, base);

    return result;
  }

  // Default control for bool fields.
  static bool ShowDefaultControl(
      const std::string& name, bool* val, bool* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      if (CheckBit(editor_control_flags,
                   editor::EditorControlFlags::kDisplayLabel)) {
        ImGui::Text("%s: %s", *val ? "true" : "false", name.c_str());
      } else {
        ImGui::Text("%s", *val ? "true" : "false");
      }
      return false;
    }

    EditingElementMode mode =
        BeginEditingElement(val, base, editor_control_flags);

    std::string label =
        editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags);
    bool result = ImGui::Checkbox(label.c_str(), val);

    result |= EndEditingElement(mode, val, base);

    return result;
  }

  template <typename E>
  static bool ShowEnumControl(const std::string& name, E* val, E* base,
                              editor::EditorControlFlags editor_control_flags =
                                  editor::EditorControlFlags::kDefault) {
    if (!CheckBit(editor_control_flags,
                  editor::EditorControlFlags::kIsEditable)) {
      for (E e : proto::EnumMetaData<E>::kValues) {
        if (*val == e) {
          if (CheckBit(editor_control_flags,
                       editor::EditorControlFlags::kDisplayLabel)) {
            ImGui::Text("%s: %s", proto::EnumMetaData<E>::GetName(e).data(),
                        name.c_str());
          } else {
            ImGui::Text("%s", proto::EnumMetaData<E>::GetName(e).data());
          }
          break;
        }
      }
      return false;
    }

    EditingElementMode mode =
        BeginEditingElement(val, base, editor_control_flags);

    bool updated = false;

    if (ImGui::BeginCombo(
            editor::GenerateUniqueImGuiLabel(name, val, editor_control_flags)
                .c_str(),
            proto::EnumMetaData<E>::GetName(*val).data())) {
      for (E e : proto::EnumMetaData<E>::kValues) {
        const bool is_selected = (*val == e);
        if (ImGui::Selectable(proto::EnumMetaData<E>::GetName(e).data(),
                              is_selected) &&
            *val != e) {
          *val = e;
          updated = true;
        }
        if (is_selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }

    updated |= EndEditingElement(mode, val, base);

    return updated;
  }

  static bool ShowDefaultControl(
      const std::string& name, SceneHandleInterface* val,
      SceneHandleInterface* base,
      editor::EditorControlFlags editor_control_flags =
          editor::EditorControlFlags::kDefault) {
    bool updated = false;

    EditingElementMode mode = EditingElementMode::kNormal;
    if (base && val->GetIdentifier() == base->GetIdentifier()) {
      editor::PushBaseIsfElementStyle();
      mode = EditingElementMode::kMatchesBase;
    }

    ImVec2 padding = ImGui::GetStyle().FramePadding;

    NodeHandle scene_node = val->GetSceneNode();

    std::string label;
    if (!scene_node) {
      label = absl::StrFormat("<Unassigned %s>", val->GetTypeName());
    } else {
      if (!scene_node->GetName().empty()) {
        label = std::string(scene_node->GetName());
      } else {
        label = "<Unnamed>";
      }
    }

    ImVec2 text_size = ImGui::CalcTextSize(label.c_str());
    text_size.x += padding.x * 2.0f;
    text_size.y += padding.y * 2.0f;

    ImVec2 cursor = ImGui::GetCursorPos();
    ImGui::Dummy(text_size);
    ImVec2 final_cursor = ImGui::GetCursorPos();

    ImVec2 min = ImGui::GetItemRectMin();
    ImVec2 max = ImGui::GetItemRectMax();
    ImVec4 fill_color = mode == EditingElementMode::kMatchesBase
                            ? editor::kGrey900
                            : editor::kBlue900;
    ImVec4 border_color = mode == EditingElementMode::kMatchesBase
                              ? editor::kGrey700
                              : editor::kBlue700;

    if (val->GetSceneNode()) {
      ImGui::GetWindowDrawList()->AddRectFilled(
          min, max, ImGui::GetColorU32(editor::WithAlpha(fill_color, 0.6f)),
          3.0f);
    }

    ImGui::GetWindowDrawList()->AddRect(
        min, max, ImGui::GetColorU32(editor::WithAlpha(border_color, 0.8f)),
        3.0f);

    cursor.x += padding.x;
    cursor.y += padding.y;
    ImGui::SetCursorPos(cursor);
    ImGui::Text(label.c_str());
    ImGui::SetCursorPos(final_cursor);

    // Clear the scene handle. It can't be cleared if the base is set, because
    // you can't override to an unset value in proto encoding.
    bool base_has_value = base && base->GetIdentifier().index() != 0;
    if (val->GetSceneNode() && !base_has_value) {
      std::string label = ElementPopupLabel(*val);
      if (ImGui::BeginPopupContextItem(label.c_str())) {
        if (ImGui::MenuItem("Clear", nullptr, false)) {
          val->AssignSceneHandleForNode({});
          updated = true;
        }
        ImGui::EndPopup();
      }
    }

    if (val->CanAssignSceneHandleForNode(editor::GetDragAndDropPayloadNode()) &&
        ImGui::BeginDragDropTarget()) {
      NodeHandle node = editor::AcceptDragAndDropPayloadNode();
      if (node) {
        val->AssignSceneHandleForNode(node);
        updated = true;
      }
      ImGui::EndDragDropTarget();
    }

    ImGui::SameLine(0, ImGui::GetStyle().ItemInnerSpacing.x + padding.x);

    ImGui::Text(name.c_str());

    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + padding.y);

    // TODO: Add Support for the "Revert To Base" popup option.
    // This is tricky to do because the base hasn't actually been mapped to the
    // real node handle.
    if (mode == EditingElementMode::kMatchesBase) {
      editor::PopBaseIsfElementStyle();
    }

    return updated;
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_PROTOTYPES_SAMPLES_EDITOR_EDITOR_FIELD_CONTROL_H_
