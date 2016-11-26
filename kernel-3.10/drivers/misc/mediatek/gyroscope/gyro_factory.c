#include "gyro_factory.h"

static int gyro_factory_open(struct inode *inode, struct file *file)
{
    file->private_data = gyro_context_obj;

    if (file->private_data == NULL)
    {
        GYRO_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}

static int gyro_factory_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static int gyro_set_cali(int data[GYRO_AXES_NUM])
{
	struct gyro_context *cxt = gyro_context_obj;
	GYRO_LOG(" factory gyro cali %d,%d,%d \n",data[GYRO_AXIS_X],data[GYRO_AXIS_Y],data[GYRO_AXIS_Z] );
	GYRO_LOG(" original gyro  cali %d,%d,%d \n",cxt->cali_sw[GYRO_AXIS_X],cxt->cali_sw[GYRO_AXIS_Y],cxt->cali_sw[GYRO_AXIS_Z]);
	cxt->cali_sw[GYRO_AXIS_X] += data[GYRO_AXIS_X];
    cxt->cali_sw[GYRO_AXIS_Y] += data[GYRO_AXIS_Y];
    cxt->cali_sw[GYRO_AXIS_Z] += data[GYRO_AXIS_Z];
	GYRO_LOG(" GYRO new cali %d,%d,%d \n",cxt->cali_sw[GYRO_AXIS_X],cxt->cali_sw[GYRO_AXIS_Y],cxt->cali_sw[GYRO_AXIS_Z]);

	return 0;
}

static int gyro_clear_cali(void)
{
	struct gyro_context *cxt = gyro_context_obj;
	cxt->cali_sw[GYRO_AXIS_X] = 0;
    cxt->cali_sw[GYRO_AXIS_Y] = 0;
    cxt->cali_sw[GYRO_AXIS_Z] = 0;
	GYRO_LOG(" GYRO after clear cali %d,%d,%d \n",cxt->cali_sw[GYRO_AXIS_X],cxt->cali_sw[GYRO_AXIS_Y],cxt->cali_sw[GYRO_AXIS_Z] );
	return 0;
}

static long gyro_factory_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *data;
    long err = 0;
	struct gyro_context *cxt = gyro_context_obj;
	int x,y,z,status;
	char strbuf[256];
    int cali[3] = {0};
    SENSOR_DATA sensor_data = {0};
	int smtRes;
	
    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if (err)
    {
        GYRO_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch (cmd)
    {
    case GYROSCOPE_IOCTL_INIT:
		if(cxt->gyro_ctl.enable_nodata!= NULL){
			err = cxt->gyro_ctl.enable_nodata(1);
			if(err < 0)
			{
				GYRO_LOG("GYROSCOPE_IOCTL_READ_SENSORDATA read data fail!\n");
				break;
			}
			GYRO_LOG("GYROSCOPE_IOCTL_INIT\n");

		}else{
			GYRO_LOG("GYROSCOPE_IOCTL_READ_SENSORDATA ");
		}
	    break;
    case GYROSCOPE_IOCTL_SMT_DATA:
		data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;    
        }
		smtRes = 1;
        err = copy_to_user(data, &smtRes,  sizeof(smtRes));
        if (err)
        {
            err = -EINVAL;
            GYRO_ERR("copy gyro data to user failed!\n");
        }

        break;    
    case GYROSCOPE_IOCTL_READ_SENSORDATA:
		data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;    
        }
		if(cxt->gyro_data.get_data != NULL){
			err = cxt->gyro_data.get_data(&x, &y, &z, &status);
			if(err < 0)
			{
				GYRO_LOG("GYROSCOPE_IOCTL_READ_SENSORDATA read data fail!\n");
				break;
			}
			x+=cxt->cali_sw[0];
			y+=cxt->cali_sw[1];
			z+=cxt->cali_sw[2];
			sprintf(strbuf, "%x %x %x", x, y, z);
			GYRO_LOG("GYROSCOPE_IOCTL_READ_SENSORDATA read data : (%d, %d, %d)!\n", x, y, z);
			GYRO_LOG("GYROSCOPE_IOCTL_READ_SENSORDATA read strbuf : (%s)!\n", strbuf);
			
	        if (copy_to_user(data, strbuf, strlen(strbuf)+1))
	        {
	            err = -EFAULT;
	            break;    
	        }
		}else{
			GYRO_LOG("GYROSCOPE_IOCTL_READ_SENSORDATA ");
		}
        break;
		
    case GYROSCOPE_IOCTL_READ_SENSORDATA_RAW:
		data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;    
        }
		if(cxt->gyro_data.get_raw_data != NULL){
			err = cxt->gyro_data.get_raw_data(&x, &y, &z);
			if(err < 0)
			{
				GYRO_LOG("GSENSOR_IOCTL_READ_RAW_DATA read data fail!\n");
				break;
			}
			x+=cxt->cali_sw[0];
			y+=cxt->cali_sw[1];
			z+=cxt->cali_sw[2];
			sprintf(strbuf, "%x %x %x", x, y, z);
			GYRO_LOG("GSENSOR_IOCTL_READ_RAW_DATA read data : (%d, %d, %d)!\n", x, y, z);
	        if (copy_to_user(data, strbuf, strlen(strbuf)+1))
	        {
	            err = -EFAULT;
	            break;    
	        }
		}else{
			GYRO_LOG("GSENSOR_IOCTL_READ_RAW_DATA FAIL!\n ");
		}
        break;   
    case GYROSCOPE_IOCTL_SET_CALI:
        data = (void __user*)arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;    
        }
        if (copy_from_user(&sensor_data, data, sizeof(sensor_data)))
        {
            err = -EFAULT;
            break;    
        }
        cali[0] = sensor_data.x ;
        cali[1] = sensor_data.y ;
        cali[2] = sensor_data.z ;
		GYRO_LOG("GYROSCOPE_IOCTL_SET_CALI data : (%d, %d, %d)!\n", cali[0], cali[1], cali[2]);
        gyro_set_cali(cali);
/*
		if(cxt->gyro_ctl.gyro_calibration != NULL)
		{
			err = cxt->gyro_ctl.gyro_calibration(SETCALI, cali);
			if(err < 0)
			{
				GYRO_LOG("GYROSCOPE_IOCTL_SET_CALI fail!\n");
				break;
			}
		}     
		*/
        break;

    case GYROSCOPE_IOCTL_CLR_CALI:
		/*
        if(cxt->gyro_ctl.gyro_calibration != NULL)
		{
			err = cxt->gyro_ctl.gyro_calibration(CLRCALI, cali);
			if(err < 0)
			{
				GYRO_LOG("GYROSCOPE_IOCTL_CLR_CALI fail!\n");
				break;
			}
		}   
		*/
		gyro_clear_cali();
        break;

    case GYROSCOPE_IOCTL_GET_CALI:
        data = (void __user*)arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;    
        }
		/*
        if(cxt->gyro_ctl.gyro_calibration != NULL)
		{
			err = cxt->gyro_ctl.gyro_calibration(GETCALI, cali);
			if(err < 0)
			{
				GYRO_LOG("GYROSCOPE_IOCTL_GET_CALI fail!\n");
				break;
			}
		}
		*/
		GYRO_LOG("GYROSCOPE_IOCTL_GET_CALI data : (%d, %d, %d)!\n", cxt->cali_sw[0] , cxt->cali_sw[1], cxt->cali_sw[2]);
        sensor_data.x = cxt->cali_sw[0]; ; 
        sensor_data.y = cxt->cali_sw[1]; ;
        sensor_data.z = cxt->cali_sw[2]; ;
        if (copy_to_user(data, &sensor_data, sizeof(sensor_data)))
        {
            err = -EFAULT;
            break;
        }
        break;
		
    default:
        GYRO_LOG("unknown IOCTL: 0x%08x\n", cmd);
        err = -ENOIOCTLCMD;
        break;

    }
    return err;
}


static struct file_operations gyro_factory_fops = {
    .open = gyro_factory_open,
    .release = gyro_factory_release,
    .unlocked_ioctl = gyro_factory_unlocked_ioctl,
};

static struct miscdevice gyro_factory_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gyroscope",
    .fops = &gyro_factory_fops,
};

int gyro_factory_device_init()
{
	int error = 0;
	struct gyro_context *cxt = gyro_context_obj;

	if (!cxt->gyro_ctl.is_use_common_factory) {
		GYRO_LOG("Node of '/dev/gyroscope' has already existed!\n");
		return -1;
	}
    if ((error = misc_register(&gyro_factory_device)))
    {
        GYRO_LOG("gyro_factory_device register failed\n");
		error = -1;
    }	
	return error;
}




