########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Dual MIT/GPLv2
# 
# The contents of this file are subject to the MIT license as set out below.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# Alternatively, the contents of this file may be used under the terms of
# the GNU General Public License Version 2 ("GPL") in which case the provisions
# of GPL are applicable instead of those above.
# 
# If you wish to allow use of your version of this file only under the terms of
# GPL, and not to allow others to use your version of this file under the terms
# of the MIT license, indicate your decision by deleting the provisions above
# and replace them with the notice and other provisions required by GPL as set
# out in the file called "GPL-COPYING" included in this distribution. If you do
# not delete the provisions above, a recipient may use your version of this file
# under the terms of either the MIT license or GPL.
# 
# This License is also included in this distribution in the file called
# "MIT-COPYING".
# 
# EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
# PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
### ###########################################################################

SYS_CFLAGS := \
 -fno-short-enums \
 -funwind-tables \
 -ffunction-sections \
 -fdata-sections \
 -D__linux__

# Always include the NDK compatibility directory, because it allows us to
# compile in inline versions of simple functions to eliminate dependencies,
# and we can also constrain the available APIs.

SYS_INCLUDES := \
 -isystem android/ndk

ifneq ($(TARGET_PLATFORM),)

# Support for building with the Android NDK >= r13b.
# The NDK provides only the most basic includes and libraries.

SYS_INCLUDES += \
 -isystem $(NDK_PLATFORMS_ROOT)/$(TARGET_PLATFORM)/arch-$(TARGET_ARCH)/usr/include

else # !TARGET_PLATFORM

# These libraries are not coming from the NDK now, so we need to include them
# from the ANDROID_ROOT source tree.

SYS_INCLUDES += \
 -isystem $(ANDROID_ROOT)/bionic/libc/include \
 -isystem $(ANDROID_ROOT)/bionic/libc/kernel/android/uapi \
 -isystem $(ANDROID_ROOT)/bionic/libm/include \
 -isystem $(ANDROID_ROOT)/external/zlib/src \
 -isystem $(ANDROID_ROOT)/libnativehelper/include/nativehelper

# Obsolete include paths

SYS_INCLUDES += \
 -isystem $(ANDROID_ROOT)/bionic/libc/kernel/arch-$(TARGET_ARCH) \
 -isystem $(ANDROID_ROOT)/bionic/libc/kernel/common \
 -isystem $(ANDROID_ROOT)/bionic/libc/kernel/uapi \
 -isystem $(ANDROID_ROOT)/bionic/libthread_db/include \
 -isystem $(ANDROID_ROOT)/external/jpeg \
 -isystem $(ANDROID_ROOT)/frameworks/base/include \
 -isystem $(ANDROID_ROOT)/system/core/include/sync \
 -isystem $(ANDROID_ROOT)/system/core/libsync

endif # !TARGET_PLATFORM

# These components aren't in the NDK. They may be added to the VDK later.
# For now, always rely on the ANDROID_ROOT source tree.

SYS_INCLUDES += \
 -isystem $(ANDROID_ROOT)/external/libdrm \
 -isystem $(ANDROID_ROOT)/external/libdrm/include/drm \
 -isystem $(ANDROID_ROOT)/external/libjpeg-turbo \
 -isystem $(ANDROID_ROOT)/external/libpng \
 -isystem $(ANDROID_ROOT)/external/libunwind/include \
 -isystem $(ANDROID_ROOT)/frameworks/compile/libbcc/bcinfo/include \
 -isystem $(ANDROID_ROOT)/frameworks/compile/libbcc/include \
 -isystem $(ANDROID_ROOT)/frameworks/compile/slang \
 -isystem $(ANDROID_ROOT)/frameworks/native/include \
 -isystem $(ANDROID_ROOT)/frameworks/native/libs/arect/include \
 -isystem $(ANDROID_ROOT)/frameworks/native/libs/nativewindow/include \
 -isystem $(ANDROID_ROOT)/frameworks/native/vulkan/include \
 -isystem $(ANDROID_ROOT)/frameworks/rs \
 -isystem $(ANDROID_ROOT)/frameworks/rs/cpp \
 -isystem $(ANDROID_ROOT)/frameworks/rs/driver \
 -isystem $(ANDROID_ROOT)/hardware/libhardware/include \
 -isystem $(ANDROID_ROOT)/libnativehelper/include \
 -isystem $(ANDROID_ROOT)/system/core/adf/libadf/include \
 -isystem $(ANDROID_ROOT)/system/core/adf/libadfhwc/include \
 -isystem $(ANDROID_ROOT)/system/core/base/include \
 -isystem $(ANDROID_ROOT)/system/core/include \
 -isystem $(ANDROID_ROOT)/system/core/libion/include \
 -isystem $(ANDROID_ROOT)/system/core/libsync/include \
 -isystem $(ANDROID_ROOT)/system/media/camera/include

# Handle SSL specially as we do not want to contaminate one with the other
# by including both paths
ifneq ($(wildcard $(ANDROID_ROOT)/external/boringssl/src/include),)
SYS_INCLUDES += \
 -isystem $(ANDROID_ROOT)/external/boringssl/src/include
else
SYS_INCLUDES += \
 -isystem $(ANDROID_ROOT)/external/openssl/include
endif

OPTIM ?= -O2

# Android enables build-id sections to allow mapping binaries to debug
# information for symbol resolution
SYS_LDFLAGS += -Wl,--build-id=md5 -Wl,--gc-sections
