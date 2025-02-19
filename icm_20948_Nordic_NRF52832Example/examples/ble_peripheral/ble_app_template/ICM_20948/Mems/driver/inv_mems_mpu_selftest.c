/*
* ________________________________________________________________________________________________________
* Copyright � 2014-2015 InvenSense Inc. Portions Copyright � 2014-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively �Software�) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/
#ifndef MEMS_20609

#include "inv_mems_mpu_selftest.h"

#if (MEMS_CHIP == HW_ICM20648)
#include "inv_mems_defines.h"
#include "inv_mems_base_driver.h"
#include "inv_mems_drv_hook.h"
#include "inv_mems_transport.h"
#if defined MEMS_SECONDARY_DEVICE
    #include "inv_mems_slave_compass.h"
#endif

////
extern int a_average[3];
extern int g_average[3];

/* full scale and LPF setting */
#define SELFTEST_GYRO_FS            ((0 << 3) | 1)
#define SELFTEST_ACCEL_FS           ((7 << 3) | 1)

/* register settings */
#define SELFTEST_GYRO_SMPLRT_DIV        10
#define SELFTEST_GYRO_AVGCFG        	3
#define SELFTEST_ACCEL_SMPLRT_DIV       10
#define SELFTEST_ACCEL_DEC3_CFG     	2

/* wait time in ms between 2 data collection */
#define WAIT_TIME_BTW_2_SAMPLESREAD     10
/* wait time in ms after sensor self-test enabling for oscillations to stabilize */
#define DEF_ST_STABLE_TIME              20 //ms
/* number of times self test reading should be done until abord */
#define DEF_ST_TRY_TIMES                2
/* number of samples to be read to be averaged */
#define DEF_ST_SAMPLES                  200

#define LOWER_BOUND_CHECK(value) ((value)>>1) // value * 0.5
#define UPPER_BOUND_CHECK(value) ((value) + ((value)>>1) ) // value * 1.5

struct recover_regs
{
    // Bank#0
    uint8_t fifo_cfg;			// REG_FIFO_CFG
    uint8_t user_ctrl;			// REG_USER_CTRL
    uint8_t lp_config;			// REG_LP_CONFIG
    uint8_t int_enable;			// REG_INT_ENABLE
    uint8_t int_enable_1;		// REG_INT_ENABLE_1
    uint8_t int_enable_2;       // REG_INT_ENABLE_2
    uint8_t fifo_en;			// REG_FIFO_EN
    uint8_t fifo_en_2;			// REG_FIFO_EN_2
    uint8_t fifo_rst;			// REG_FIFO_RST
    uint8_t pwr_mgmt_1;			// REG_PWR_MGMT_1
    uint8_t pwr_mgmt_2;			// REG_PWR_MGMT_2

    // Bank#2
    uint8_t gyro_smplrt_div;	// REG_GYRO_SMPLRT_DIV
    uint8_t gyro_config_1;		// REG_GYRO_CONFIG_1
    uint8_t gyro_config_2;		// REG_GYRO_CONFIG_2
    uint8_t accel_smplrt_div_1;	// REG_ACCEL_SMPLRT_DIV_1
    uint8_t accel_smplrt_div_2;	// REG_ACCEL_SMPLRT_DIV_2
    uint8_t accel_config;		// REG_ACCEL_CONFIG
    uint8_t accel_config_2;		// REG_ACCEL_CONFIG_2
};
static struct recover_regs saved_regs;

static uint8_t gyro_st_data[3] = {0};
static uint8_t accel_st_data[3] = {0};

// Table for list of results for factory self-test value equation
// st_otp = 2620/2^FS * 1.01^(st_value - 1)
// for gyro and accel FS = 0 so 2620 * 1.01^(st_value - 1)
// st_value = 1 => 2620
// st_value = 2 => 2620 * 1.01 = 2646
// etc../
static const uint16_t sSelfTestEquation[256] =
{
    2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
    2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
    3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
    3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
    3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
    3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
    4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
    4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
    4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
    5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
    5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
    6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
    6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
    7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
    7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
    8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
    9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
    10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
    10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
    11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
    12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
    13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
    15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
    16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
    17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
    19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
    20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
    22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
    24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
    26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
    28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
    30903, 31212, 31524, 31839, 32157, 32479, 32804
};

static inv_error_t inv_save_setting()
{
    inv_error_t result = 0;

    result |= inv_read_mems_reg_core(REG_FIFO_CFG, 1, &saved_regs.fifo_cfg);

    result |= inv_read_mems_reg_core(REG_USER_CTRL, 1, &saved_regs.user_ctrl);

    result |= inv_read_mems_reg_core(REG_LP_CONFIG, 1, &saved_regs.lp_config);

    result |= inv_read_mems_reg_core(REG_INT_ENABLE, 1, &saved_regs.int_enable);

    result |= inv_read_mems_reg_core(REG_INT_ENABLE_1, 1, &saved_regs.int_enable_1);

    result |= inv_read_mems_reg_core(REG_INT_ENABLE_2, 1, &saved_regs.int_enable_2);

    result |= inv_read_mems_reg_core(REG_FIFO_EN, 1, &saved_regs.fifo_en);

    result |= inv_read_mems_reg_core(REG_FIFO_EN_2, 1, &saved_regs.fifo_en_2);

    result |= inv_read_mems_reg_core(REG_FIFO_RST, 1, &saved_regs.fifo_rst);

    result |= inv_read_mems_reg_core(REG_GYRO_SMPLRT_DIV, 1, &saved_regs.gyro_smplrt_div);

    result |= inv_read_mems_reg_core(REG_GYRO_CONFIG_1, 1, &saved_regs.gyro_config_1);

    result |= inv_read_mems_reg_core(REG_GYRO_CONFIG_2, 1, &saved_regs.gyro_config_2);

    result |= inv_read_mems_reg_core(REG_ACCEL_SMPLRT_DIV_1, 1, &saved_regs.accel_smplrt_div_1);

    result |= inv_read_mems_reg_core(REG_ACCEL_SMPLRT_DIV_2, 1, &saved_regs.accel_smplrt_div_2);

    result |= inv_read_mems_reg_core(REG_ACCEL_CONFIG, 1, &saved_regs.accel_config);

    result |= inv_read_mems_reg_core(REG_ACCEL_CONFIG_2, 1, &saved_regs.accel_config_2);

    return result;
}

static inv_error_t inv_recover_setting()
{
    inv_error_t result = 0;

    // Wake up
    result |= inv_write_single_mems_reg_core(REG_PWR_MGMT_1, BIT_CLK_PLL);

    // Stop sensors
    result |= inv_write_single_mems_reg_core(REG_PWR_MGMT_2,
              BIT_PWR_PRESSURE_STBY | BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY);

    // Restore sensor configurations
    result |= inv_write_single_mems_reg_core(REG_GYRO_SMPLRT_DIV, saved_regs.gyro_smplrt_div);

    result |= inv_write_single_mems_reg_core(REG_GYRO_CONFIG_1, saved_regs.gyro_config_1);

    result |= inv_write_single_mems_reg_core(REG_GYRO_CONFIG_2, saved_regs.gyro_config_2);

    result |= inv_write_single_mems_reg_core(REG_ACCEL_SMPLRT_DIV_1, saved_regs.accel_smplrt_div_1);

    result |= inv_write_single_mems_reg_core(REG_ACCEL_SMPLRT_DIV_2, saved_regs.accel_smplrt_div_2);

    result |= inv_write_single_mems_reg_core(REG_ACCEL_CONFIG, saved_regs.accel_config);

    result |= inv_write_single_mems_reg_core(REG_ACCEL_CONFIG_2, saved_regs.accel_config_2);

    // Restore FIFO configurations
    result |= inv_write_single_mems_reg_core(REG_FIFO_CFG, saved_regs.fifo_cfg);

    result |= inv_write_single_mems_reg_core(REG_LP_CONFIG, saved_regs.lp_config);

    result |= inv_write_single_mems_reg_core(REG_INT_ENABLE, saved_regs.int_enable);

    result |= inv_write_single_mems_reg_core(REG_INT_ENABLE_1, saved_regs.int_enable_1);

    result |= inv_write_single_mems_reg_core(REG_FIFO_EN, saved_regs.fifo_en);

    result |= inv_write_single_mems_reg_core(REG_FIFO_EN_2, saved_regs.fifo_en_2);

    result |= inv_write_single_mems_reg_core(REG_FIFO_RST, MAX_5_BIT_VALUE);

    result |= inv_write_single_mems_reg_core(REG_FIFO_RST, saved_regs.fifo_rst);

    // Reset DMP
    result |= inv_write_single_mems_reg_core(REG_USER_CTRL,
              (saved_regs.user_ctrl & (~BIT_FIFO_EN)) | BIT_DMP_RST);
    inv_sleep(DMP_RESET_TIME);

    result |= inv_set_dmp_address();
    #if (MEMS_CHIP != HW_ICM20609)
    result |= inv_set_secondary();
    #endif
    #if defined MEMS_SECONDARY_DEVICE
    result |= inv_setup_compass_akm();
    #endif
    // Restore PWR_MGMT_1/PWR_MGMT_2
    result |= inv_write_single_mems_reg_core(REG_PWR_MGMT_2, saved_regs.pwr_mgmt_2);
    result |= inv_write_single_mems_reg_core(REG_PWR_MGMT_1, saved_regs.pwr_mgmt_1);

    return result;
}

/**
*  @brief check accel or gyro self test
*  @param[in] sensorType type of sensor to be tested
*  @param[in] selfTestValuesReadFromReg self test written in register at production time.
*  @param[in] meanNormalTestValues average value of normal test.
*  @param[in] meanSelfTestValues   average value of self test
*  @return zero as success. A non-zero return value indicates failure in self test.
*/
static int inv_check_accelgyro_self_test(enum INV_SENSORS sensorType, uint8_t * selfTestValuesReadFromReg, int *meanNormalTestValues, int *meanSelfTestValues)
{
    int ret_val;
    int lIsStOtpReadZero = 0;
    int l_st_otp_read[3], lDiffNormalStValues[3], i;

    ret_val = 0;

    // Calculate factory Self-Test value (ST_OTP) based on the following equation:
    // The factory Self-Test value (ST_OTP) is calculated from the ST_Code (the SELF_TEST values read)
    // using the following equation, where �FS� is the full scale value code:
    // st_otp = 2620/2^FS * 1.01^(st_value - 1)
    // the result of the equation is in sSelfTestEquation array
    for (i = 0; i < 3; i++)
    {
        if (selfTestValuesReadFromReg[i] != 0)
        {
            l_st_otp_read[i] = sSelfTestEquation[selfTestValuesReadFromReg[i] - 1];
        }
        else
        {
            l_st_otp_read[i] = 0;
            lIsStOtpReadZero = 1;
        }
    }

    // Calculate the Self-Test response as follows:
    // - GXST = GX_ST_OS - GX_OS
    // - GYST = GY_ST_OS - GY_OS
    // - GZST = GZ_ST_OS - GZ_OS
    // - AXST = AX_ST_OS - AX_OS
    // - AYST = AY_ST_OS - AY_OS
    // - AZST = AZ_ST_OS - AZ_OS
    for (i = 0; i < 3; i++)
    {
        lDiffNormalStValues[i] = meanSelfTestValues[i] - meanNormalTestValues[i];

        // Ensure the factory Self-Test values ST_OTP are not 0
        if (!lIsStOtpReadZero)
        {
            // Compare the current Self-Test response (GXST, GYST, GZST, AXST, AYST and AZST) to the factory Self-Test values (ST_OTP)
            // and report Self-Test is passing if all the following criteria are fulfilled:
            // (GXST / GXST_OTP)  > 0.5
            if (lDiffNormalStValues[i] < LOWER_BOUND_CHECK(l_st_otp_read[i]) )
                ret_val = 1;

            if (sensorType != INV_SENSOR_GYRO)

                // (AXST / AXST_OTP)  < 1.5
                if (lDiffNormalStValues[i] > UPPER_BOUND_CHECK(l_st_otp_read[i]) )
                    ret_val = 1;
        }
        else
            ret_val = 1;
    }

    return ret_val;
}

static inv_error_t inv_setup_selftest()
{
    inv_error_t result = 0;

    // Save PWR_MGMT_1 and PWR_MGMT_2
    result |= inv_read_mems_reg_core(REG_PWR_MGMT_1, 1, &saved_regs.pwr_mgmt_1);
    result |= inv_read_mems_reg_core(REG_PWR_MGMT_2, 1, &saved_regs.pwr_mgmt_2);

    // Wake up
    result |= inv_write_single_mems_reg_core(REG_PWR_MGMT_1, BIT_CLK_PLL);

    // Save the current settings
    result |= inv_save_setting();

    // Stop sensors
    result |= inv_write_single_mems_reg_core(REG_PWR_MGMT_2,
              BIT_PWR_PRESSURE_STBY | BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY);

    // Set cycle mode
    result |= inv_write_single_mems_reg_core(REG_LP_CONFIG,
              BIT_I2C_MST_CYCLE | BIT_ACCEL_CYCLE | BIT_GYRO_CYCLE);

    // Configure FSR and DLPF for gyro
    result |= inv_write_single_mems_reg_core(REG_GYRO_SMPLRT_DIV, SELFTEST_GYRO_SMPLRT_DIV);

    result |= inv_write_single_mems_reg_core(REG_GYRO_CONFIG_1, SELFTEST_GYRO_FS);

    result |= inv_write_single_mems_reg_core(REG_GYRO_CONFIG_2, SELFTEST_GYRO_AVGCFG);

    // Configure FSR and DLPF for accel
    result |= inv_write_single_mems_reg_core(REG_ACCEL_SMPLRT_DIV_1, 0);

    result |= inv_write_single_mems_reg_core(REG_ACCEL_SMPLRT_DIV_2, SELFTEST_ACCEL_SMPLRT_DIV);

    result |= inv_write_single_mems_reg_core(REG_ACCEL_CONFIG, SELFTEST_ACCEL_FS);

    result |= inv_write_single_mems_reg_core(REG_ACCEL_CONFIG_2, SELFTEST_ACCEL_DEC3_CFG);

    // Read selftest values
    // Retrieve factory Self-Test code (ST_Code) from SELF_TEST registers  (User Bank 1):
    result |= inv_read_mems_reg_core(REG_SELF_TEST1, 1, &gyro_st_data[0]);

    result |= inv_read_mems_reg_core(REG_SELF_TEST2, 1, &gyro_st_data[1]);

    result |= inv_read_mems_reg_core(REG_SELF_TEST3, 1, &gyro_st_data[2]);

    result |= inv_read_mems_reg_core(REG_SELF_TEST4, 1, &accel_st_data[0]);

    result |= inv_read_mems_reg_core(REG_SELF_TEST5, 1, &accel_st_data[1]);

    result |= inv_read_mems_reg_core(REG_SELF_TEST6, 1, &accel_st_data[2]);

    // Restart sensors
    result |= inv_write_single_mems_reg_core(REG_PWR_MGMT_2, BIT_PWR_PRESSURE_STBY);
    inv_sleep(GYRO_ENGINE_UP_TIME);

    return result;
}

static int inv_selftest_read_samples(enum INV_SENSORS type, int *sum_result, int *s)
{
    uint8_t w;
    int16_t vals[3];
    uint8_t d[BYTES_PER_SENSOR];
    int j;

    // Average 200 readings and save the averaged values as GX_OS, GY_OS, GZ_OS, AX_OS, AY_OS and AZ_OS.
    // - GX_OS = Average (GYRO_XOUT_H | GYRO_XOUT_L)
    // - GY_OS = Average (GYRO_YOUT_H | GYRO_YOUT_L)
    // - GZ_OS = Average (GYRO_ZOUT_H | GYRO_ZOUT_L)
    // - AX_OS = Average (ACCEL_XOUT_H | ACCEL_XOUT_L)
    // - AY_OS = Average (ACCEL_YOUT_H | ACCEL_YOUT_L)
    // - AZ_OS = Average (ACCEL_ZOUT_H | ACCEL_ZOUT_L)

    if (INV_SENSOR_GYRO == type)
        w = REG_GYRO_XOUT_H_SH;
    else
        w = REG_ACCEL_XOUT_H_SH;

    while (*s < DEF_ST_SAMPLES)
    {

        if(inv_read_mems_reg_core(w, BYTES_PER_SENSOR, d))
            return -1;

        for (j = 0; j < THREE_AXES; j++)
        {
            vals[j] = (d[(2 * j)] << 8) | (d[(2 * j) + 1] & 0xff);
            sum_result[j] += vals[j];
        }

        (*s)++;

        inv_sleep(WAIT_TIME_BTW_2_SAMPLESREAD);
    }

    return 0;
}

/*
*  inv_do_test_accelgyro() - do the actual test of self testing
*/
static int inv_do_test_accelgyro(enum INV_SENSORS sensorType, int *meanValue, int *stMeanValue)
{
    int result, i, j;
    int lNbSamples = 0;

    // initialize output to be 0
    for (i = 0; i < THREE_AXES; i++)
    {
        meanValue[i] = 0;
        stMeanValue[i] = 0;
    }

    // read the accel/gyro output
    // the output values are 16 bits wide and in 2�s complement
    // Average 200 readings and save the averaged values
    result = inv_selftest_read_samples(sensorType, meanValue, &lNbSamples);

    if (result)
        return result;

    for (j = 0; j < THREE_AXES; j++)
    {
        meanValue[j] /= lNbSamples;
    }

    // Set Self-Test Bit
    if (sensorType == INV_SENSOR_GYRO)
    {
        // Enable gyroscope Self-Test by setting register User Bank 2, Register Address 02 (02h) Bit [5:3] to b111
        result = inv_write_single_mems_reg_core(REG_GYRO_CONFIG_2, BIT_GYRO_CTEN | SELFTEST_GYRO_AVGCFG);
    }
    else
    {
        result = inv_write_single_mems_reg_core(REG_ACCEL_CONFIG_2, BIT_ACCEL_CTEN | SELFTEST_ACCEL_DEC3_CFG);
    }

    if (result)
        return result;

    // Wait 20ms for oscillations to stabilize.
    inv_sleep(DEF_ST_STABLE_TIME);

    // Read the accel/gyro output and average 200 readings
    // These readings are in units of LSBs
    lNbSamples = 0;
    result = inv_selftest_read_samples(sensorType, stMeanValue, &lNbSamples);

    if (result)
        return result;

    for (j = 0; j < THREE_AXES; j++)
    {
        stMeanValue[j] /= lNbSamples;
    }

    return 0;
}




int inv_mems_run_selftest(void)
{
    int result;
    int gyro_bias_st[THREE_AXES], gyro_bias_regular[THREE_AXES];
    int accel_bias_st[THREE_AXES], accel_bias_regular[THREE_AXES];
    int test_times;
    volatile char accel_result, gyro_result, compass_result;
    int i = 0;

    accel_result = 0;
    gyro_result = 0;
    compass_result = 0;

    // save original state of the chip, initialize registers, configure sensors and read ST values
    result = inv_setup_selftest();

    if (result)
        goto test_fail;

    // perform self test for gyro
    test_times = DEF_ST_TRY_TIMES;

    while (test_times > 0)
    {
        result = inv_do_test_accelgyro(INV_SENSOR_GYRO, gyro_bias_regular, gyro_bias_st);

        if (result)
            test_times--;
        else
            break;
    }

    if (result)
        goto test_fail;

    // perform self test for accel
    test_times = DEF_ST_TRY_TIMES;

    while (test_times > 0)
    {
        result = inv_do_test_accelgyro(INV_SENSOR_ACCEL, accel_bias_regular, accel_bias_st);

        if (result)
            test_times--;
        else
            break;
    }

    if (result)
        goto test_fail;

    for (i = 0; i < 3; i++)
    {
        g_average[i] = gyro_bias_regular[i];// / DEF_ST_PRECISION;
        a_average[i] = accel_bias_regular[i];// / DEF_ST_PRECISION;
    }

    // check values read at various steps
    accel_result = !inv_check_accelgyro_self_test(INV_SENSOR_ACCEL, accel_st_data, accel_bias_regular, accel_bias_st);
    gyro_result = !inv_check_accelgyro_self_test(INV_SENSOR_GYRO, gyro_st_data, gyro_bias_regular, gyro_bias_st);
    #if defined MEMS_SECONDARY_DEVICE
    //compass_result = !inv_check_akm_self_test();
    // compass_result = 1;
    #endif

test_fail:
    // restore original state of the chips
    inv_recover_setting();

    return    0x04 |
              (accel_result   << 1) |
              gyro_result;
}
#endif
/**
* @}
*/

#endif

