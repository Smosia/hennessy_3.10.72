#include "accel_factory.h"
static int acc_factory_open(struct inode *inode, struct file *file)
{
    file->private_data = acc_context_obj;

    if (file->private_data == NULL)
    {
        ACC_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}

static int acc_factory_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
static int acc_set_cali(int data[ACC_AXES_NUM])
{
	struct acc_context *cxt = acc_context_obj;
	ACC_LOG(" factory cali %d,%d,%d \n",data[ACC_AXIS_X],data[ACC_AXIS_Y],data[ACC_AXIS_Z] );
	ACC_LOG(" original cali %d,%d,%d \n",cxt->cali_sw[ACC_AXIS_X],cxt->cali_sw[ACC_AXIS_Y],cxt->cali_sw[ACC_AXIS_Z]);
	cxt->cali_sw[ACC_AXIS_X] += data[ACC_AXIS_X];
    cxt->cali_sw[ACC_AXIS_Y] += data[ACC_AXIS_Y];
    cxt->cali_sw[ACC_AXIS_Z] += data[ACC_AXIS_Z];
	ACC_LOG(" new cali %d,%d,%d \n",cxt->cali_sw[ACC_AXIS_X],cxt->cali_sw[ACC_AXIS_Y],cxt->cali_sw[ACC_AXIS_Z]);

	return 0;
}

static int acc_clear_cali(void)
{
	struct acc_context *cxt = acc_context_obj;
	cxt->cali_sw[ACC_AXIS_X] = 0;
    cxt->cali_sw[ACC_AXIS_Y] = 0;
    cxt->cali_sw[ACC_AXIS_Z] = 0;
	ACC_LOG(" after clear cali %d,%d,%d \n",cxt->cali_sw[ACC_AXIS_X],cxt->cali_sw[ACC_AXIS_Y],cxt->cali_sw[ACC_AXIS_Z] );
	return 0;
}

static long acc_factory_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *data;
    int err = 0;
	struct acc_context *cxt = acc_context_obj;
	int x,y,z;
	char strbuf[256];
    int cali[3] = {0};
    SENSOR_DATA sensor_data = {0};
	
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
        ACC_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch (cmd)
    {
    case GSENSOR_IOCTL_INIT:
	    break;
    case GSENSOR_IOCTL_READ_CHIPINFO:
        break;    
    case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;    
        }
		if(cxt->acc_ctl.enable_nodata != NULL){
			err = cxt->acc_ctl.enable_nodata(1);
        	if(err)
        	{ 
           		err = cxt->acc_ctl.enable_nodata(1);
		   		if(err)
		  		 {
		   			err = cxt->acc_ctl.enable_nodata(1);
					if(err)
					ACC_ERR("acc ioctl enable err 3 timers = %d\n", err);
		   		}
         	}
			ACC_LOG("acc ioctl real enable  \n" );
		}
		
		if(cxt->acc_data.get_data != NULL){
			//err = cxt->acc_data.get_data(&x, &y, &z, &status);
			err = cxt->acc_data.get_raw_data(&x, &y, &z);
			if(err < 0)
			{
				ACC_ERR("GSENSOR_IOCTL_READ_SENSORDATA read data fail!\n");
				break;
			}
			x+=cxt->cali_sw[0];
			y+=cxt->cali_sw[1];
			z+=cxt->cali_sw[2];
			sprintf(strbuf, "%x %x %x", x, y, z);
			ACC_LOG("GSENSOR_IOCTL_READ_SENSORDATA read data : (%d, %d, %d)!\n", x, y, z);
			ACC_LOG("GSENSOR_IOCTL_READ_SENSORDATA read strbuf : (%s)!\n", strbuf);
			
	        if (copy_to_user(data, strbuf, strlen(strbuf)+1))
	        {
	            err = -EFAULT;
	            break;    
	        }
		}else{
			ACC_ERR("GSENSOR_IOCTL_READ_SENSORDATA ");
		}
        break;
		
    case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *) arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;    
        }
		if(cxt->acc_data.get_raw_data != NULL){
			err = cxt->acc_data.get_raw_data(&x, &y, &z);
			if(err < 0)
			{
				ACC_ERR("GSENSOR_IOCTL_READ_RAW_DATA read data fail!\n");
				break;
			}
			x+=cxt->cali_sw[0];
			y+=cxt->cali_sw[1];
			z+=cxt->cali_sw[2];
			sprintf(strbuf, "%x %x %x", x, y, z);
			ACC_LOG("GSENSOR_IOCTL_READ_RAW_DATA read data : (%d, %d, %d)!\n", x, y, z);
	        if (copy_to_user(data, strbuf, strlen(strbuf)+1))
	        {
	            err = -EFAULT;
	            break;    
	        }
		}else{
			ACC_ERR("GSENSOR_IOCTL_READ_SENSORDATA ");
		}
        break;   
    case GSENSOR_IOCTL_SET_CALI:
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
		ACC_LOG("GSENSOR_IOCTL_SET_CALI data : (%d, %d, %d)!\n", cali[0], cali[1], cali[2]);

		acc_set_cali(cali);
		/*if(cxt->acc_ctl.acc_calibration != NULL)
		{
			err = cxt->acc_ctl.acc_calibration(SETCALI, cali);
			if(err < 0)
			{
				ACC_ERR("GSENSOR_IOCTL_SET_CALI fail!\n");
				break;
			}
		}
		*/
        break;

    case GSENSOR_IOCTL_CLR_CALI:
		/*
        if(cxt->acc_ctl.acc_calibration != NULL)
		{
			err = cxt->acc_ctl.acc_calibration(CLRCALI, cali);
			if(err < 0)
			{
				ACC_ERR("GSENSOR_IOCTL_CLR_CALI fail!\n");
				break;
			}
		} 
		*/
		acc_clear_cali();
        break;

    case GSENSOR_IOCTL_GET_CALI:
        data = (void __user*)arg;
        if (data == NULL)
        {
            err = -EINVAL;
            break;    
        }
		/*
        if(cxt->acc_ctl.acc_calibration != NULL)
		{
			err = cxt->acc_ctl.acc_calibration(GETCALI, cali);
			if(err < 0)
			{
				ACC_ERR("GSENSOR_IOCTL_GET_CALI fail!\n");
				break;
			}
		}
		*/
		ACC_LOG("GSENSOR_IOCTL_GET_CALI data : (%d, %d, %d)!\n", cxt->cali_sw[0], cxt->cali_sw[1], cxt->cali_sw[2]);
        sensor_data.x = cxt->cali_sw[0]; 
        sensor_data.y = cxt->cali_sw[1];
        sensor_data.z = cxt->cali_sw[2];
        if (copy_to_user(data, &sensor_data, sizeof(sensor_data)))
        {
            err = -EFAULT;
            break;
        }
        break;
		
    default:
        ACC_ERR("unknown IOCTL: 0x%08x\n", cmd);
        err = -ENOIOCTLCMD;
        break;

    }
    return err;
}


static struct file_operations acc_factory_fops = {
    .open = acc_factory_open,
    .release = acc_factory_release,
    .unlocked_ioctl = acc_factory_unlocked_ioctl,
};

static struct miscdevice acc_factory_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gsensor",
    .fops = &acc_factory_fops,
};

int acc_factory_device_init()
{
	int error = 0;
	struct acc_context *cxt = acc_context_obj;
	if (!cxt->acc_ctl.is_use_common_factory) {
		ACC_LOG("Node of '/dev/gsensor' has already existed!\n");
		return -1;
	}
    if ((error = misc_register(&acc_factory_device)))
    {
        ACC_LOG("acc_factory_device register failed\n");
		error = -1;
    }	
	return error;
}



