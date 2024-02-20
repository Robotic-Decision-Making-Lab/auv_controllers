// Copyright 2024, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef POSITION_CONTROLLERS__VISIBILITY_CONTROL_H_
#define POSITION_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define POSITION_CONTROLLERS_EXPORT __attribute__((dllexport))
#define POSITION_CONTROLLERS_IMPORT __attribute__((dllimport))
#else
#define POSITION_CONTROLLERS_EXPORT __declspec(dllexport)
#define POSITION_CONTROLLERS_IMPORT __declspec(dllimport)
#endif
#ifdef POSITION_CONTROLLERS_BUILDING_DLL
#define POSITION_CONTROLLERS_PUBLIC POSITION_CONTROLLERS_EXPORT
#else
#define POSITION_CONTROLLERS_PUBLIC POSITION_CONTROLLERS_IMPORT
#endif
#define POSITION_CONTROLLERS_PUBLIC_TYPE POSITION_CONTROLLERS_PUBLIC
#define POSITION_CONTROLLERS_LOCAL
#else
#define POSITION_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#define POSITION_CONTROLLERS_IMPORT
#if __GNUC__ >= 4
#define POSITION_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define POSITION_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#else
#define POSITION_CONTROLLERS_PUBLIC
#define POSITION_CONTROLLERS_LOCAL
#endif
#define POSITION_CONTROLLERS_PUBLIC_TYPE
#endif

#endif  // POSITION_CONTROLLERS__VISIBILITY_CONTROL_H_
