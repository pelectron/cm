/**
 *  Copyright 2024-2025 Pelé Constam
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef CM_ASSERT_HPP
#define CM_ASSERT_HPP

#include "cortex/common.hpp"

#include <cassert>
#include <type_traits>

#define CM_ASSERT_RETURN_FALSE(expr) \
  if (not static_cast<bool>(expr)) { \
    return false;                    \
  }

#define CM_STRINGIFY1(x) #x
#define CM_STRINGIFY(x) CM_STRINGIFY1(x)

/**
 * Assert that the expression passed in evaluates to true.
 * if expr evaluates to false, this macro will:
 * - call AssertHandler with the error message at runtime
 * - fall back to the standard assert in a constexpr context
 */
#define CM_ASSERT(expr)                                  \
  {                                                      \
    if (std::is_constant_evaluated()) {                  \
      if (not static_cast<bool>((expr)))                 \
        assert(expr);                                    \
    } else {                                             \
      if (not static_cast<bool>(expr)) {                 \
        AssertHandler("Assertion failed: " CM_STRINGIFY( \
            expr) "\n\tfile: '" __FILE__ "'"             \
                  "\n\tline: " CM_STRINGIFY(__LINE__));  \
      }                                                  \
    }                                                    \
  }

/**
 * Same effect as CM_ASSERT(expr), except that message passed to
 * AssertHandler can be specified.
 */
#define CM_ASSERT_MESSAGE(expr, message)                                       \
  if (std::is_constant_evaluated()) {                                          \
    if (not static_cast<bool>((expr)))                                         \
      assert(expr);                                                            \
  } else {                                                                     \
    if (not static_cast<bool>((expr)))                                         \
      cm::AssertHandler("Assertion failed: " CM_STRINGIFY(                     \
          expr) "\n\tfile: '" __FILE__ "'"                                     \
                "\n\tline: " CM_STRINGIFY(__LINE__) "\n\t message: " message); \
  }

namespace cm {
void AssertHandler(const char* const message);
}
#endif
