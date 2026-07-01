#ifndef XR_EXT_INTERACTION_PROFILE_BATTERY_STATE_DISPLAY_H_
#define XR_EXT_INTERACTION_PROFILE_BATTERY_STATE_DISPLAY_H_ 1

/*
** Copyright 2017-2026 The Khronos Group Inc.
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

/*
** This header is generated from the Khronos OpenXR XML API Registry.
**
*/


#ifdef __cplusplus
extern "C" {
#endif


#ifndef XR_EXT_interaction_profile_battery_state_display

// XR_EXT_interaction_profile_battery_state_display is a preprocessor guard. Do not pass it to API calls.
#define XR_EXT_interaction_profile_battery_state_display 1
#define XR_TYPE_BATTERY_STATE_DISPLAY_EXT ((XrStructureType) 1000836000U)

#define XR_EXT_interaction_profile_battery_state_display_SPEC_VERSION 1
#define XR_EXT_INTERACTION_PROFILE_BATTERY_STATE_DISPLAY_EXTENSION_NAME "XR_EXT_interaction_profile_battery_state_display"
typedef XrFlags64 XrBatteryStateDisplayStateFlagsEXT;

// Flag bits for XrBatteryStateDisplayStateFlagsEXT
// Indicates validity of slink:XrBatteryStateDisplayEXT::pname:batteryLevel
static const XrBatteryStateDisplayStateFlagsEXT XR_BATTERY_STATE_DISPLAY_STATE_VALID_BIT_EXT = 0x00000001;
// Indicates that the device associated with the interaction profile is charging.
static const XrBatteryStateDisplayStateFlagsEXT XR_BATTERY_STATE_DISPLAY_STATE_CHARGING_BIT_EXT = 0x00000002;
// Indicates that the device associated with the interaction profile is plugged in to a power source.
static const XrBatteryStateDisplayStateFlagsEXT XR_BATTERY_STATE_DISPLAY_STATE_PLUGGED_IN_BIT_EXT = 0x00000004;
// Indicates that the device associated with the interaction profile does not have a battery power source.
static const XrBatteryStateDisplayStateFlagsEXT XR_BATTERY_STATE_DISPLAY_STATE_NO_BATTERY_BIT_EXT = 0x00000008;

typedef struct XrBatteryStateDisplayEXT {
    XrStructureType                       type;
    void* XR_MAY_ALIAS                    next;
    XrBatteryStateDisplayStateFlagsEXT    stateFlags;
    float                                 batteryLevel;
} XrBatteryStateDisplayEXT;


// Reflection macros
#define XR_LIST_BITS_XrBatteryStateDisplayStateFlagsEXT(_) \
    _(XR_BATTERY_STATE_DISPLAY_STATE_VALID_BIT_EXT, 0x00000001) \
    _(XR_BATTERY_STATE_DISPLAY_STATE_CHARGING_BIT_EXT, 0x00000002) \
    _(XR_BATTERY_STATE_DISPLAY_STATE_PLUGGED_IN_BIT_EXT, 0x00000004) \
    _(XR_BATTERY_STATE_DISPLAY_STATE_NO_BATTERY_BIT_EXT, 0x00000008)

#define XR_LIST_STRUCT_XrBatteryStateDisplayEXT(_) \
    _(type) \
    _(next) \
    _(stateFlags) \
    _(batteryLevel)

#endif /* XR_EXT_interaction_profile_battery_state_display */

#ifdef __cplusplus
}
#endif

#endif
