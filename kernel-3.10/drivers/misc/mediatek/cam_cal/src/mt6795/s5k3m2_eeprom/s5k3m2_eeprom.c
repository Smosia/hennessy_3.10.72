#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG

#ifdef CAM_CAL_DEBUG
#include <linux/xlog.h>
#define PFX "CAM_CAL"
#define CAM_CALDB(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG, PFX, "[%s] " fmt, __FUNCTION__, ##arg)
#else
#define CAM_CALDB(x,...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock);

#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define OTP_POWER_SOURCE 9
#define OTP_DEVICE_ID    0xB0

static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x6C>>1) };

static struct i2c_client * g_pstI2Cclient = NULL;

static dev_t g_CAM_CALdevno = MKDEV(226, 0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;

static u8 *I2CDMABuf_va = NULL;
static volatile u64 I2CDMABuf_pa = NULL;

int iWriteCAM_CAL(u16 a_u2Addr, u32 a_u4Bytes, u8 puDataInBytes)
{
    int i4RetValue = 0;
    u32 u4Index = 0;
    int retry = 3;
    char puSendCmd[3] = {(char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF), puDataInBytes};
    do {
        CAM_CALDB("Write 0x%x=0x%x \n", a_u2Addr, puDataInBytes);
        i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, 3);
        if (i4RetValue != 3) {
            CAM_CALDB("I2C send failed!!\n");
        }
        else {
            break;
        }
        mdelay(10);
    } while ((retry--) > 0);

    return 0;
}

int iCAM_i2c_Read(struct i2c_client *client, char *writebuf,
            int writelen, char *readbuf, int readlen)
{
    int ret,i;

    if(writelen != 0)
    {
        for(i = 0 ; i < writelen; i++)
            I2CDMABuf_va[i] = writebuf[i];

        client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;

        if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
            dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%llx\n", __func__,ret,I2CDMABuf_pa);

        client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
    }

    if(readlen!=0)
    {
        client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;

        ret = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, readlen);

        for(i = 0; i < readlen; i++)
            readbuf[i] = I2CDMABuf_va[i];

        client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
    }
    return ret;
}

int iReadCAM_CAL(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};

    i4RetValue = iCAM_i2c_Read(g_pstI2Cclient, puReadCmd, 2, (char *)a_puBuff, ui4_length);
    return 0;
}

static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;

   CAM_CALDB("begin\n" );

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;

   CAM_CALDB("u4CurrentOffset is %d \n",u4CurrentOffset);

   do
   {
       CAM_CALDB("Write 0x%x=0x%x \n",u4CurrentOffset, pBuff[0]);
       i4RetValue = iWriteCAM_CAL((u16)u4CurrentOffset, 1, pBuff[0]);
       if (i4RetValue != 0)
       {
            CAM_CALDB("I2C iWriteData failed!! \n");
            return -1;
       }
       u4IncOffset ++;
       i4ResidueDataLength--;
       u4CurrentOffset = ui4_offset + u4IncOffset;
       pBuff = pinputdata + u4IncOffset;
   }while (i4ResidueDataLength > 0);
   CAM_CALDB("iWriteData done\n" );

   return 0;
}

static int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32  u4IncOffset = 0;
   u32  u4CurrentOffset;
   u8 * pBuff;

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;
   do
   {
       i4RetValue = iReadCAM_CAL((u16)u4CurrentOffset, 1, pBuff);
       CAM_CALDB("Read 0x%x=0x%x \n", u4CurrentOffset, pBuff[0]);
       if (i4RetValue != 0)
       {
            CAM_CALDB("I2C iReadData failed!! \n");
            return -1;
       }
       u4IncOffset++;
       i4ResidueDataLength--;
       u4CurrentOffset = ui4_offset + u4IncOffset;
       pBuff = pinputdata + u4IncOffset;
   }while (i4ResidueDataLength > 0);

   return 0;
}

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
    return err;
}

static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long s5k3m2otp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
    CAM_CALDB("%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {
        case COMPAT_CAM_CALIOC_G_READ:
        {
            data32 = compat_ptr(arg);
            data = compat_alloc_user_space(sizeof(*data));
            if (data == NULL)
                return -EFAULT;

            err = compat_get_cal_info_struct(data32, data);
            if (err)
                return err;

            ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
            err = compat_put_cal_info_struct(data32, data);

            if(err != 0)
                CAM_CALDB("compat_put_acdk_sensor_getinfo_struct failed\n");
            return ret;
        }
        default:
            return -ENOIOCTLCMD;
    }
}
#endif

static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {
                kfree(pBuff);
                CAM_CALDB("ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB("init Working buffer address 0x%p  command is 0x%8x\n", pWorkingBuff, (u32)a_u4Command);

    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            CAM_CALDB("Write CMD \n");

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("Read CMD \n");
            if(ptempbuf->u4Offset == 4)
                ptempbuf->u4Offset = 1;
            CAM_CALDB("offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("length %d \n", ptempbuf->u4Length);
            CAM_CALDB("Before read Working buffer address 0x%p \n", pWorkingBuff);
            i4RetValue = iReadData((u16)(ptempbuf->u4Offset), ptempbuf->u4Length, pWorkingBuff);

            CAM_CALDB("After read Working buffer data  0x%4x \n", *pWorkingBuff);

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        default:
            CAM_CALDB("No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        CAM_CALDB("to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("to user  Working buffer address 0x%p \n", pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}

static u32 g_u4Opened = 0;
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
        CAM_CALDB("Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

    if(TRUE != hwPowerOn(OTP_POWER_SOURCE, VOL_2800, CAM_CAL_DRVNAME))
    {
        CAM_CALDB("Fail to enable analog gain\n");
        return -EIO;
    }

    return 0;
}

static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);
    if(TRUE != hwPowerDown(OTP_POWER_SOURCE, CAM_CAL_DRVNAME))
    {
        CAM_CALDB("Fail to disable analog gain\n");
        return -EIO;
    }
    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    .compat_ioctl = s5k3m2otp_Ioctl_Compat,
    .unlocked_ioctl = CAM_CAL_Ioctl
};

inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("Allocate device no failed\n");
        return -EAGAIN;
    }

    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);
        CAM_CALDB("Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("Attatch file operation failed\n");
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    int i4RetValue;
    CAM_CALDB("Attach I2C \n");
    I2CDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, 512, &I2CDMABuf_pa, GFP_KERNEL);

    if (!I2CDMABuf_va) {
        dev_dbg(&client->dev,"%s Allocate DMA I2C Buffer failed!\n",__func__);
        return -EIO;
    }

    spin_lock(&g_CAM_CALLock);
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = OTP_DEVICE_ID>>1;
    g_pstI2Cclient->timing = 400;
    spin_unlock(&g_CAM_CALLock);

    CAM_CALDB("g_pstI2Cclient->addr = 0x%x \n",g_pstI2Cclient->addr);

    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("register char device failed!\n");
        return i4RetValue;
    }

    CAM_CALDB("Attached!! \n");
    return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    if (I2CDMABuf_va) {
        dma_free_coherent(NULL, 512, I2CDMABuf_va, I2CDMABuf_pa);
        I2CDMABuf_va = NULL;
        I2CDMABuf_pa = 0;
    }
    return 0;
}

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};

static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,
    .remove = CAM_CAL_i2c_remove,
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id
};

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

static struct platform_driver g_stCAM_CAL_Driver = {
    .probe = CAM_CAL_probe,
    .remove = CAM_CAL_remove,
    .driver = {
        .name = CAM_CAL_DRVNAME,
        .owner = THIS_MODULE
    }
};

static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0
};

static int __init CAM_CAL_i2C_init(void)
{
    i2c_register_board_info(0, &kd_cam_cal_dev, 1);
    if (platform_driver_register(&g_stCAM_CAL_Driver))
        return -ENODEV;
    if (platform_device_register(&g_stCAM_CAL_Device))
        return -ENODEV;
    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
    platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");