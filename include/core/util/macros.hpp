//
// Created by bwiethof on 3/20/23.
//

#pragma once

#if ((defined(_MSVC_LANG) && _MSVC_LANG >= 201703L) || __cplusplus >= 201703L)
#define NO_DISCARD [[nodiscard ]]
#else

#ifdef _WIN32
#define NO_DISCARD _Check_return_
#else
#define NO_DISCARD __attribute__((warn_unused_result))
#endif

#endif
