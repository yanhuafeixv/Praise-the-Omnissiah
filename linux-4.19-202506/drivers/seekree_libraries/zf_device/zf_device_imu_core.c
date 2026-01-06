#include <linux/spi/spi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/unaligned/be_byteshift.h>

#include "zf_device_imu_core.h"
#include "zf_device_imu660ra.h"
#include "zf_device_imu660rb.h"
#include "zf_device_imu963ra.h"


void imu_type_print(struct imu_dev_struct *dev)
{
    switch (dev->type)
    {
        case ZF_DEVICE_IMU660RA:
        {
            dev_info(&dev->spi->dev, "IMU660RA init OK\n");
            break;
        }

        case ZF_DEVICE_IMU660RB:
        {
            dev_info(&dev->spi->dev, "IMU660RB init OK\n");
            break;
        }

        case ZF_DEVICE_IMU660RC:
        {
            dev_info(&dev->spi->dev, "IMU660RC init OK\n");
            break;
        }

        case ZF_DEVICE_IMU963RA:
        {
            dev_info(&dev->spi->dev, "IMU963RA init OK\n");
            break;
        }

        default:
        {
            dev_info(&dev->spi->dev, "NO FIND IMU DEV\n");
            break;
        }
            
    }

}

int imu_device_init(struct imu_dev_struct *dev)
{
    uint8 ret = 0;

    // if(dev->type == NO_FIND_DEVICE)
    // {
        do
        {
            // IMU963里面的六轴和IMU660RB是同一个芯片，所以要先初始化IMU963RA
            if(imu963ra_init(dev) == 0)
            {
                dev->type = ZF_DEVICE_IMU963RA;
                dev->imu_get_acc = imu963ra_get_acc;
                dev->imu_get_gyro = imu963ra_get_gyro;
                dev->imu_get_mag = imu963ra_get_mag;
                break;
            }

            if (imu660rb_init(dev) == 0)
            {
                dev->type = ZF_DEVICE_IMU660RB;
                dev->imu_get_acc = imu660rb_get_acc;
                dev->imu_get_gyro = imu660rb_get_gyro;
                break;
            }

            if (imu660ra_init(dev) == 0)
            {
                dev->type = ZF_DEVICE_IMU660RA;
                dev->imu_get_acc = imu660ra_get_acc;
                dev->imu_get_gyro = imu660ra_get_gyro;
                break;
            }

            dev->type = NO_FIND_DEVICE;
            dev->imu_get_acc = NULL;
            dev->imu_get_gyro = NULL;
            dev->imu_get_mag = NULL;
            ret = -1;

        } while (0);
    // }
    
    return ret;
}



MODULE_LICENSE("GPL");
MODULE_AUTHOR("seekfree_bigW");
