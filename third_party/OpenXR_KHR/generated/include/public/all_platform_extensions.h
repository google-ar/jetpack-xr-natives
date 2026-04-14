#ifndef OPENXR_PLATFORM_EXTENSIONS_HELPER_PUBLIC_H_
#define OPENXR_PLATFORM_EXTENSIONS_HELPER_PUBLIC_H_

// IWYU pragma: begin_exports

/* These include helps graduating extensions into openxr.h */
#include <openxr/openxr.h>


// IWYU pragma: end_exports

// These extensions are experimental and MUST be included explicitly by code
// that needs them to avoid breaks on updates.
// Do not rely on this header to include them.
#ifdef DO_NOT_USE_INCLUDE_LATEST_EXPERIMENTAL_EXTENSIONS
#endif  // DO_NOT_USE_INCLUDE_LATEST_EXPERIMENTAL_EXTENSIONS

// This macro extends the base XR_LIST_EXTENSIONS macro with the extensions in
// this header.
#define XR_LIST_PUBLIC_PLATFORM_EXTENSIONS(_) \

#endif  // OPENXR_PLATFORM_EXTENSIONS_HELPER_PUBLIC_H_
