#ifndef XR_ANDROIDX1_ENUMERATE_SYSTEM_EXTENSION_PROPERTIES_H_
#define XR_ANDROIDX1_ENUMERATE_SYSTEM_EXTENSION_PROPERTIES_H_ 1

/*
** Copyright 2017-2025 The Khronos Group Inc.
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


#ifndef XR_ANDROIDX1_enumerate_system_extension_properties

// XR_ANDROIDX1_enumerate_system_extension_properties is a preprocessor guard. Do not pass it to API calls.
#define XR_ANDROIDX1_enumerate_system_extension_properties 1
#define XR_ANDROIDX1_enumerate_system_extension_properties_SPEC_VERSION 1
#define XR_ANDROIDX1_ENUMERATE_SYSTEM_EXTENSION_PROPERTIES_EXTENSION_NAME "XR_ANDROIDX1_enumerate_system_extension_properties"
#define XR_TYPE_SYSTEM_EXTENSION_PROPERTIES_ANDROIDX1 ((XrStructureType) 1000724000U)
#define XR_TYPE_EVENT_DATA_SYSTEM_PROPERTIES_CHANGED_ANDROIDX1 ((XrStructureType) 1000724001U)
typedef struct XrSystemExtensionPropertiesANDROIDX1 {
    XrStructureType          type;
    void* XR_MAY_ALIAS       next;
    XrExtensionProperties    properties;
    XrBool32                 isSupported;
} XrSystemExtensionPropertiesANDROIDX1;

typedef struct XrEventDataSystemPropertiesChangedANDROIDX1 {
    XrStructureType             type;
    const void* XR_MAY_ALIAS    next;
} XrEventDataSystemPropertiesChangedANDROIDX1;

typedef XrResult (XRAPI_PTR *PFN_xrEnumerateSystemExtensionsANDROIDX1)(XrInstance instance, XrSystemId systemId, uint32_t propertyCapacityInput, uint32_t* propertyCountOutput, XrSystemExtensionPropertiesANDROIDX1* properties);

#ifndef XR_NO_PROTOTYPES
#ifdef XR_EXTENSION_PROTOTYPES
XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateSystemExtensionsANDROIDX1(
    XrInstance                                  instance,
    XrSystemId                                  systemId,
    uint32_t                                    propertyCapacityInput,
    uint32_t*                                   propertyCountOutput,
    XrSystemExtensionPropertiesANDROIDX1*       properties);
#endif /* XR_EXTENSION_PROTOTYPES */
#endif /* !XR_NO_PROTOTYPES */
#endif /* XR_ANDROIDX1_enumerate_system_extension_properties */

#ifdef __cplusplus
}
#endif

#endif
