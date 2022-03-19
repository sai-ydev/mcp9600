/*
 * mcp9600.h
 * Based on SeeedStudio's drivers
 *  Created on: Mar 18, 2022
 *      Author: yaman
 */

#ifndef INC_MCP9600_H_
#define INC_MCP9600_H_

#include "stm32l0xx_hal.h"
#include "stdio.h"

typedef int            s32;
typedef long unsigned int   u32;
typedef short          s16;
typedef unsigned short u16;
typedef char           s8;
typedef unsigned char  u8;

typedef enum {
    NO_ERROR = 0,
    ERROR_PARAM = -1,
    ERROR_COMM = -2,
    ERROR_OTHERS = -128,
} err_t;

#define CHECK_RESULT(a,b)   do{if(a!=b)  {    \
            printf("%s\r\n", __FILE__);    \
            printf("%d\r\n", __LINE__);   \
            printf(" error code = %d\r\n", a);  \
            return a;   \
        }}while(0)

#define HOT_JUNCTION_REG_ADDR               0X0
#define JUNCTION_TEMP_DELTA_REG_ADDR        0X1
#define COLD_JUNCTION_TEMP_REG_ADDR         0X2
#define RAW_ADC_DATA_REG_ADDR               0X3
#define STAT_REG_ADDR                       0X4
#define THERM_SENS_CFG_REG_ADDR             0X5
#define DEVICE_CFG_REG_ADDR                 0X6


#define ALERT1_CFG_REG_ADDR                 0X8
#define ALERT2_CFG_REG_ADDR                 0X9
#define ALERT3_CFG_REG_ADDR                 0XA
#define ALERT4_CFG_REG_ADDR                 0XB
#define ALERT1_HYS_REG_ADDR                 0XC
#define ALERT2_HYS_REG_ADDR                 0XD
#define ALERT3_HYS_REG_ADDR                 0XE
#define ALERT4_HYS_REG_ADDR                 0XF

#define TEMP_ALERT1_LIMIT_REG_ADDR          0X10
#define TEMP_ALERT2_LIMIT_REG_ADDR          0X11
#define TEMP_ALERT3_LIMIT_REG_ADDR          0X12
#define TEMP_ALERT4_LIMIT_REG_ADDR          0X13

#define VERSION_ID_REG_ADDR                 0x20


#define DEFAULT_IIC_ADDR  					(0X60 << 1)

#define RESOLUTION_0_5_DEGREE               0
#define RESOLUTION_0_25_DEGREE              0X01
#define RESOLUTION_0_125_DEGREE             0X02
#define RESOLUTION_0_0625_DEGREE            0X03

#define THER_TYPE_K                         0X0<<4
#define THER_TYPE_J                         0X1<<4
#define THER_TYPE_T                         0X2<<4
#define THER_TYPE_N                         0X3<<4
#define THER_TYPE_S                         0X4<<4
#define THER_TYPE_E                         0X5<<4
#define THER_TYPE_B                         0X6<<4
#define THER_TYPE_R                         0X7<<4


#define ALERT_NUN_1                         1
#define ALERT_NUN_2                         2
#define ALERT_NUN_3                         3
#define ALERT_NUN_4                         4


#define FILT_OFF                            0
#define FILT_MIN                            1
#define FILT_MID                            4
#define FILT_MAX                            7


#define ENABLE   true
#define DISABLE  false


#define COLD_JUNC_RESOLUTION_0_625          0<<7
#define COLD_JUNC_RESOLUTION_0_25           1<<7

#define ADC_18BIT_RESOLUTION                0<<5
#define ADC_16BIT_RESOLUTION                1<<5
#define ADC_14BIT_RESOLUTION                2<<5
#define ADC_12BIT_RESOLUTION                3<<5

#define BURST_1_SAMPLE                      0<<2
#define BURST_2_SAMPLE                      1<<2
#define BURST_4_SAMPLE                      2<<2
#define BURST_8_SAMPLE                      3<<2
#define BURST_16_SAMPLE                     4<<2
#define BURST_32_SAMPLE                     5<<2
#define BURST_64_SAMPLE                     6<<2
#define BURST_128_SAMPLE                    7<<2

#define NORMAL_OPERATION                    0
#define SHUTDOWN_MODE                       1
#define BURST_MODE                          2

#define ACTIVE_LOW                          0<<2
#define ACTIVE_HIGH                         1<<2

#define INT_MODE                            1<<1
#define COMPARE_MODE                        0<<1

#define UPDATE_FLAG                         1<<6

class MCP9600_IIC_OPRTS {
  public:

	void IIC_begin(I2C_HandleTypeDef *hi2c){
		i2c = hi2c;
	}
    err_t IIC_write_byte(u8 reg, u8 byte);
    err_t IIC_read_byte(u8 reg, u8* byte);
    void set_iic_addr(u8 IIC_ADDR);
    err_t IIC_read_16bit(u8 start_reg, u16* value);
    err_t IIC_write_16bit(u8 reg, u16 value);
    err_t IIC_read_bytes(u8 start_reg, u8* data, u32 data_len);
  private:
    I2C_HandleTypeDef *i2c;
    u8 _IIC_ADDR;

};


class MCP9600: public MCP9600_IIC_OPRTS {
  public:
    MCP9600(u8 IIC_ADDR = DEFAULT_IIC_ADDR);
    ~MCP9600() {};
    err_t init(u8 therm_type, I2C_HandleTypeDef *i2c);
    err_t read_version(u16* ver);

    err_t read_hot_junc(float* value);
    err_t read_junc_temp_delta(float* value);
    err_t read_cold_junc(float* value);
    err_t read_ADC_data(u8* data, u32 data_len);
    err_t read_status(u8* byte);

    err_t set_therm_cfg(u8 set_byte);
    err_t read_therm_cfg(u8* byte);
    err_t set_therm_type(u8 set_byte);
    err_t set_filt_coefficients(u8 set_byte);

    err_t set_dev_cfg(u8 set_byte);
    err_t read_dev_cfg(u8* byte);
    err_t set_sensor_mode(u8 set_byte);
    err_t set_burst_mode_samp(u8 set_byte);
    err_t set_ADC_meas_resolution(u8 set_byte);
    err_t set_cold_junc_resolution(u8 set_byte);

    err_t set_alert_limit(u8 alert_num, u16 value);
    err_t set_alert_hys(u8 alert_num, u16 value);


    err_t set_alert_cfg(u8 alert_num, u8 set_byte);
    err_t read_alert_cfg(u8 alert_num, u8* byte);

    err_t clear_int_flag(u8 alert_num);
    err_t set_alert_for_TH_or_TC(u8 alert_num, u8 set_byte);
    err_t set_alert_limit_direction(u8 alert_num, u8 set_byte);
    err_t set_alert_bit(u8 alert_num, u8 set_byte);
    err_t set_alert_mode_bit(u8 alert_num, u8 set_byte);
    err_t set_alert_enable(u8 alert_num, u8 set_byte);

    u16 covert_temp_to_reg_form(float temp);

    err_t check_data_update(bool* stat);
    err_t read_INT_stat(u8* stat);
  private:

};
#endif /* INC_MCP9600_H_ */
