/****************************************************************************
 *
 *   Copyright (c) 2014-2018 PX4 Development Team. All rights reserved.
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
 * @file landing_target_estimator_params.c
 * Landing target estimator algorithm parameters.
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 * @author Mohammed Kabir <kabir@uasys.io>
 *
 */

/**
 * Landing target estimator module enable
 *
 * @boolean
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_INT32(LTEST_EN, 1);

/**
 * Landing target estimator module enable orientation estimation
 *
 * @boolean
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_INT32(LTEST_YAW_EN, 0);

/**
 * Landing target estimator module enable position estimation
 *
 * @boolean
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_INT32(LTEST_POS_EN, 1);

/**
 * Integer bitmask controlling data fusion and aiding methods.
 *
 * Set bits in the following positions to enable:
 * 0 : Set to true to use the target's GPS position data if available. (+1)
 * 1 : Set to true to use the relative GPS velocity data if available. (If the target is moving, a target velocity estimate is required) (+2)
 * 2 : Set to true to use the target relative position from vision-based data if available (+4)
 * 3 : Set to true to use the target relative position from irlock data if available (+8)
 * 4 : Set to true to use the target relative position from uwb data if available (+16)
 * 5 : Set to true to use the mission landing point. Ignored if target GPS position enabled. (+32)
 *
 * @group Landing target Estimator
 * @min 0
 * @max 63
 * @bit 0 target GPS position
 * @bit 1 relative GPS velocity
 * @bit 2 vision relative position
 * @bit 3 irlock relative position
 * @bit 4 uwb relative position
 * @bit 5 mission landing position
 *
 * @reboot_required true
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_INT32(LTEST_AID_MASK, 46);

/**
 * Landing target mode
 *
 * Configure the mode of the landing target. Depending on the mode, the state of the estimator (Kalman filter) varies. For static targets, the landing target observations can be used to aid position estimation.
 *
 * Mode Static: The landing target is static, the state of the Kalman filter is: [relative position, relative velocity, bias]. If the observations have a low variance,they can be used to aid position estimation.
 * Mode Moving: The landing target may be moving around, the state of the Kalman filter is: [relative position, relative velocity, bias, target's acceleration]
 * Mode Moving Augmented State: The landing target may be moving around, the state of the Kalman filter is: [relative position, drone velocity, bias, target's acceleration, target's velocity]. The state is augmented to estimate both the drone's velocity and the target's velocity (when comapred to the mocing mode where the relative velocity is estimated).
 *
 * @min 0
 * @max 2
 * @group Landing target Estimator
 * @value 0 Static
 * @value 1 Moving
 * @value 2 Moving Aug. State
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_INT32(LTEST_MODE, 1);

/**
 * Landing target model
 *
 * Configure the Kalman Filter model used to predict the state of the filter.
 *
 * Mode decoupled: One KF per direction: x,y,z decoupled
 * Mode coupled: One KF for all directions: [x,y,z] coupled
 *
 * @min 0
 * @max 2
 * @group Landing target Estimator
 * @value 0 Decoupled
 * @value 1 Coupled
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_INT32(LTEST_MODEL, 1);


/**
 * Landing Target Timeout
 *
 * Time after which the landing target is considered lost without any new measurements.
 *
 * @unit s
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.5
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_BTOUT, 3.0f);

/**
 * Drone acceleration uncertainty
 *
 * Variance of drone's acceleration used for landing target position prediction.
 * Higher values results in tighter following of the measurements and more lenient outlier rejection
 *
 * @unit (m/s^2)^2
 * @min 0.01
 * @decimal 2
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_ACC_D_UNC, 1.0f);

/**
 * Target acceleration uncertainty
 *
 * Variance of target acceleration (in NED frame) used for landing target position prediction.
 * Higher values results in tighter following of the measurements and more lenient outlier rejection
 *
 * @unit (m/s^2)^2
 * @min 0.01
 * @decimal 2
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_ACC_T_UNC, 1.0f);

/**
 * Bias uncertainty
 *
 * Variance of GPS bias used for landing target position prediction.
 * Higher values results in tighter following of the measurements and more lenient outlier rejection
 *
 * @unit m^2
 * @min 0.01
 * @decimal 2
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_BIAS_UNC, 0.05f);

/**
 * Bias limit
 *
 * Maximal bias between drone GPS and landing target GPS.
 *
 * @unit m^2
 * @min 0.01
 * @decimal 2
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_BIAS_LIM, 1.f);

/**
 * Landing target measurement uncertainty for Irlock and uwb sensors
 *
 * Variance of the landing target measurement from the driver.
 * Higher values result in less aggressive following of the measurement and a smoother output as well as fewer rejected measurements.
 *
 * @unit tan(rad)^2
 * @decimal 4
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_MEAS_UNC, 0.05f);

/**
 * Initial landing target and drone relative position uncertainty
 *
 * Initial variance of the relative landing target position in x,y,z direction
 *
 * @unit m^2
 * @min 0.001
 * @decimal 3
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_POS_UNC_IN, 0.5f);

/**
 * Initial landing target and drone relative velocity uncertainty
 *
 * Initial variance of the relative landing target velocity in x,y,z directions
 *
 * @unit (m/s)^2
 * @min 0.001
 * @decimal 3
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_VEL_UNC_IN, 0.5f);

/**
 * Initial GPS bias uncertainty
 *
 * Initial variance of the bias between the GPS on the target and the GPS on the drone
 *
 * @unit m^2
 * @min 0.001
 * @decimal 3
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_BIA_UNC_IN, 1.0f);

/**
 * Initial Orientation uncertainty
 *
 * Initial variance of the orientation (yaw) of the landing target
 *
 * @unit m^2
 * @min 0.001
 * @decimal 3
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_YAW_UNC_IN, 1.0f);

/**
 * Initial landing target absolute acceleration uncertainty
 *
 * Initial variance of the relative landing target acceleration in x,y,z directions
 *
 * @unit (m/s^2)^2
 * @min 0.001
 * @decimal 3
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_ACC_UNC_IN, 0.1f);

/**
 * Measurement noise for gps horizontal velocity.
 *
 * minimum allowed observation noise for gps velocity fusion (m/sec)
 *
 * @min 0.01
 * @max 5.0
 * @unit m/s
 * @decimal 2
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTE_GPS_V_NOISE, 0.3f);

/**
 * Measurement noise for gps position.
 *
 * minimum allowed observation noise for gps position fusion (m)
 *
 * @min 0.01
 * @max 10.0
 * @unit m
 * @decimal 2
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTE_GPS_P_NOISE, 0.5f);

/**
 * Whether to set the external vision observation noise from the parameter or from vision message
 *
 * If set to true the observation noise is set from the parameters directly, if set to false the measurement noise is taken from the vision message and the parameter are used as a lower bound.
 *
 * @boolean
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_INT32(LTE_EV_NOISE_MD, 0);

/**
 * Measurement noise for vision angle observations used to lower bound or replace the uncertainty included in the message
 *
 * @min 0.05
 * @unit rad
 * @decimal 2
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTE_EVA_NOISE, 0.05f);

/**
 * Measurement noise for vision position observations used to lower bound or replace the uncertainty included in the message.
 *
 * If used to replace the uncertainty in the message, the measurement noise is lineraly scaled with the altitude i.e. unc = LTE_EVP_NOISE^2 * max(dist_bottom, 1)
 *
 * @min 0.01
 * @unit m
 * @decimal 2
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTE_EVP_NOISE, 0.1f);

/**
 * Scale factor for sensor measurements in sensor x axis
 *
 * Landing target x measurements are scaled by this factor before being used
 *
 * @min 0.01
 * @decimal 3
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_SCALE_X, 1.0f);

/**
 * Scale factor for sensor measurements in sensor y axis
 *
 * Landing target y measurements are scaled by this factor before being used
 *
 * @min 0.01
 * @decimal 3
 *
 * @group Landing target Estimator
 */
PARAM_DEFINE_FLOAT(LTEST_SCALE_Y, 1.0f);


/**
 * Rotation of IRLOCK sensor relative to airframe
 *
 * Default orientation of Yaw 90°
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 *
 * @min -1
 * @max 40
 * @reboot_required true
 * @group Landing Target Estimator
 */
PARAM_DEFINE_INT32(LTEST_SENS_ROT, 2);

/**
 * X Position of IRLOCK in body frame (forward)
 *
 * @reboot_required true
 * @unit m
 * @decimal 3
 * @group Landing Target Estimator
 *
 */
PARAM_DEFINE_FLOAT(LTEST_SENS_POS_X, 0.0f);

/**
 * Y Position of IRLOCK in body frame (right)
 *
 * @reboot_required true
 * @unit m
 * @decimal 3
 * @group Landing Target Estimator
 *
 */
PARAM_DEFINE_FLOAT(LTEST_SENS_POS_Y, 0.0f);

/**
 * Z Position of IRLOCK in body frame (downward)
 *
 * @reboot_required true
 * @unit m
 * @decimal 3
 * @group Landing Target Estimator
 *
 */
PARAM_DEFINE_FLOAT(LTEST_SENS_POS_Z, 0.0f);
