/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file navigator_dataman_test.h
 *
 * Shared dataman/work-queue lifecycle for navigator unit tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <gtest/gtest.h>

#include <parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

extern "C" int dataman_main(int argc, char *argv[]);

class NavigatorDatamanRuntime
{
public:
	NavigatorDatamanRuntime()
	{
		param_control_autosave(false);
		px4::WorkQueueManagerStart();

		char name[] = "dataman";
		char start[] = "start";
		char ram[] = "-r";
		char *argv[] = {name, start, ram};
		dataman_main(3, argv);
	}

	~NavigatorDatamanRuntime()
	{
		param_control_autosave(true);

		char name[] = "dataman";
		char stop[] = "stop";
		char *argv[] = {name, stop};
		dataman_main(2, argv);

		px4::WorkQueueManagerStop();
	}
};

static inline NavigatorDatamanRuntime &navigatorDatamanRuntime()
{
	static NavigatorDatamanRuntime runtime{};
	return runtime;
}

class NavigatorDatamanTestBase : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		(void)navigatorDatamanRuntime();
	}

	static void TearDownTestSuite() {}
};
