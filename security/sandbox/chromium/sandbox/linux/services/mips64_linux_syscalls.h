// Copyright (c) 2015 WangQing. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Generated from the Linux kernel's calls.S.
#ifndef SANDBOX_LINUX_SERVICES_MIPS64_LINUX_SYSCALLS_H_
#define SANDBOX_LINUX_SERVICES_MIPS64_LINUX_SYSCALLS_H_

#include <asm/sgidefs.h>

#if !defined(__mips__) || !defined(_ABI64)
#error "Including header on wrong architecture"
#endif

// __NR_Linux is defined in <asm/unistd.h>.
#include <asm/unistd.h>

#endif  // SANDBOX_LINUX_SERVICES_MIPS64_LINUX_SYSCALLS_H_

