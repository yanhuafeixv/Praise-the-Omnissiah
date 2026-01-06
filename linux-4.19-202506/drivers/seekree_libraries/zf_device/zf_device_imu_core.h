#ifndef _zf_device_imu_core_h_
#define _zf_device_imu_core_h_

#include "../zf_common/zf_common_typedef.h"

#include <linux/spinlock.h>

enum imu_type {
    NO_FIND_DEVICE = 0,
    ZF_DEVICE_IMU660RA,
    ZF_DEVICE_IMU660RB,
	ZF_DEVICE_IMU660RC,
	ZF_DEVICE_IMU963RA,
};

struct imu_dev_struct 
{
	uint8 type;							// 设备类型,查看enum imu_type

	int16 (*imu_get_acc ) (struct imu_dev_struct *dev, int axis);
	int16 (*imu_get_gyro) (struct imu_dev_struct *dev, int axis);
	int16 (*imu_get_mag ) (struct imu_dev_struct *dev, int axis);

	struct spi_device *spi;				// spi设备
	struct regmap *regmap;				// 寄存器访问接口
	struct regmap_config reg_cfg;		// 寄存器访问接口

	spinlock_t read_raw_lock;
};

typedef enum
{
    DEV_NO_FIND  = 0x00,
    DEV_IMU660RA = 0x10,
    DEV_IMU660RB = 0x11,
    DEV_IMU963RA = 0x12,
}imu_dev_enum;

int imu_device_init(struct imu_dev_struct *dev);
void imu_type_print(struct imu_dev_struct *dev);

#endif