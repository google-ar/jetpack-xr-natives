#ifndef XR_ANDROIDX_SPATIAL_INTERACTION_H_
#define XR_ANDROIDX_SPATIAL_INTERACTION_H_ 1

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


#ifndef XR_ANDROIDX_spatial_interaction

// XR_ANDROIDX_spatial_interaction is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX_spatial_interaction 1
XR_DEFINE_HANDLE(XrHandSurfaceTouchTrackerANDROIDX)
#define XR_ANDROIDX_spatial_interaction_SPEC_VERSION 2
#define XR_ANDROIDX_SPATIAL_INTERACTION_EXTENSION_NAME "XR_ANDROIDX_spatial_interaction"
#define XR_TYPE_HAND_SURFACE_TOUCH_TRACKER_CREATE_INFO_ANDROIDX ((XrStructureType) 1000705000U)
#define XR_TYPE_HAND_SURFACE_TOUCHES_ANDROIDX ((XrStructureType) 1000705001U)
#define XR_TYPE_HAND_SURFACE_TOUCH_LOCATE_INFO_ANDROIDX ((XrStructureType) 1000705002U)
// XrHandSurfaceTouchTrackerANDROIDX
#define XR_OBJECT_TYPE_HAND_SURFACE_TOUCH_TRACKER_ANDROIDX ((XrObjectType) 1000705000U)
#define XR_HAND_SURFACE_TOUCH_POINT_COUNT_ANDROIDX 6
#define XR_HAND_SURFACE_TOUCH_MAX_COUNT_ANDROIDX 12

typedef enum XrHandSurfaceTouchPointANDROIDX {
    XR_HAND_SURFACE_TOUCH_POINT_THUMB_TIP_ANDROIDX = 0,
    XR_HAND_SURFACE_TOUCH_POINT_INDEX_TIP_ANDROIDX = 1,
    XR_HAND_SURFACE_TOUCH_POINT_MIDDLE_TIP_ANDROIDX = 2,
    XR_HAND_SURFACE_TOUCH_POINT_RING_TIP_ANDROIDX = 3,
    XR_HAND_SURFACE_TOUCH_POINT_LITTLE_TIP_ANDROIDX = 4,
    XR_HAND_SURFACE_TOUCH_POINT_PALM_ANDROIDX = 5,
    XR_HAND_SURFACE_TOUCH_POINT_MAX_ENUM_ANDROIDX = 0x7FFFFFFF
} XrHandSurfaceTouchPointANDROIDX;
typedef struct XrHandSurfaceTouchTrackerCreateInfoANDROIDX {
    XrStructureType    type;
    const void *       next;
} XrHandSurfaceTouchTrackerCreateInfoANDROIDX;

typedef struct XrHandSurfaceTouchStateANDROIDX {
    XrBool32      isTouching;
    XrVector3f    contactPosition;
} XrHandSurfaceTouchStateANDROIDX;

typedef struct XrHandSurfaceTouchesANDROIDX {
    XrStructureType                     type;
    void *                              next;
    XrPosef                             surfacePose;
    uint32_t                            touchStateCapacityInput;
    uint32_t                            touchStateCountOutput;
    XrHandSurfaceTouchStateANDROIDX*    touchStates;
} XrHandSurfaceTouchesANDROIDX;

typedef struct XrHandSurfaceTouchLocateInfoANDROIDX {
    XrStructureType    type;
    const void *       next;
    XrSpace            baseSpace;
    XrTime             time;
} XrHandSurfaceTouchLocateInfoANDROIDX;

typedef XrResult (XRAPI_PTR *PFN_xrCreateHandSurfaceTouchTrackerANDROIDX)(XrSession session, const XrHandSurfaceTouchTrackerCreateInfoANDROIDX* createInfo, XrHandSurfaceTouchTrackerANDROIDX* tracker);
typedef XrResult (XRAPI_PTR *PFN_xrDestroyHandSurfaceTouchTrackerANDROIDX)(XrHandSurfaceTouchTrackerANDROIDX tracker);
typedef XrResult (XRAPI_PTR *PFN_xrLocateHandSurfaceTouchesANDROIDX)(XrHandSurfaceTouchTrackerANDROIDX tracker, const XrHandSurfaceTouchLocateInfoANDROIDX* locateInfo, XrHandSurfaceTouchesANDROIDX* touches);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrCreateHandSurfaceTouchTrackerANDROIDX(
    XrSession                                   session,
    const XrHandSurfaceTouchTrackerCreateInfoANDROIDX* createInfo,
    XrHandSurfaceTouchTrackerANDROIDX*          tracker);

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyHandSurfaceTouchTrackerANDROIDX(
    XrHandSurfaceTouchTrackerANDROIDX           tracker);

XRAPI_ATTR XrResult XRAPI_CALL xrLocateHandSurfaceTouchesANDROIDX(
    XrHandSurfaceTouchTrackerANDROIDX           tracker,
    const XrHandSurfaceTouchLocateInfoANDROIDX* locateInfo,
    XrHandSurfaceTouchesANDROIDX*               touches);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROIDX_spatial_interaction */

#ifdef __cplusplus
}
#endif

#endif
