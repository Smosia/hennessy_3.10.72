
#include "gravity.h"

static struct grav_context *grav_context_obj = NULL;


static struct grav_init_info* gravitysensor_init_list[MAX_CHOOSE_GRAV_NUM]= {0}; //modified
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_EARLYSUSPEND)
static void grav_early_suspend(struct early_suspend *h);
static void grav_late_resume(struct early_suspend *h);
#endif

static void grav_work_func(struct work_struct *work)
{
	struct grav_context *cxt = NULL;
	int x,y,z,status;
	int64_t  nt;
	struct timespec time; 
	int err;

	GRAV_FUN();
	
	cxt  = grav_context_obj;
	
	if(NULL == cxt->grav_data.get_data)
	{
		GRAV_LOG("grav driver not register data path\n");
	}
	
	time.tv_sec = time.tv_nsec = 0;    
	time = get_monotonic_coarse(); 
	nt = time.tv_sec*1000000000LL+time.tv_nsec;
	
	err = cxt->grav_data.get_data(&x,&y,&z,&status);

	if(err)
	{
		GRAV_ERR("get grav data fails!!\n" );
		goto grav_loop;
	}
	else
	{
		{	
			if( 0 == x && 0==y 
						&& 0 == z)
			{
				    goto grav_loop;
			}

			cxt->drv_data.grav_data.values[0] = x+cxt->cali_sw[0];
			cxt->drv_data.grav_data.values[1] = y+cxt->cali_sw[1];
			cxt->drv_data.grav_data.values[2] = z+cxt->cali_sw[2];
			cxt->drv_data.grav_data.status = status;
			cxt->drv_data.grav_data.time = nt;
					
		}			
	 }
    
	if(true ==  cxt->is_first_data_after_enable)
	{
		cxt->is_first_data_after_enable = false;
	    if(GRAV_INVALID_VALUE == cxt->drv_data.grav_data.values[0] ||
		   	     GRAV_INVALID_VALUE == cxt->drv_data.grav_data.values[1] ||
		   	     GRAV_INVALID_VALUE == cxt->drv_data.grav_data.values[2])
	    {
	        GRAV_LOG(" read invalid data \n");
	       	goto grav_loop;
			
	    }
	}

	grav_data_report(cxt->drv_data.grav_data.values[0],
		cxt->drv_data.grav_data.values[1],cxt->drv_data.grav_data.values[2],
		cxt->drv_data.grav_data.status);

	grav_loop:
	if(true == cxt->is_polling_run)
	{
		  mod_timer(&cxt->timer, jiffies + atomic_read(&cxt->delay)/(1000/HZ)); 
	}
}

static void grav_poll(unsigned long data)
{
	struct grav_context *obj = (struct grav_context *)data;
	if(obj != NULL)
	{
		schedule_work(&obj->report);
	}
}

static struct grav_context *grav_context_alloc_object(void)
{
	
	struct grav_context *obj = kzalloc(sizeof(*obj), GFP_KERNEL); 
    GRAV_LOG("grav_context_alloc_object++++\n");
	if(!obj)
	{
		GRAV_ERR("Alloc gravity object error!\n");
		return NULL;
	}	
	atomic_set(&obj->delay, 200); /*5Hz*/// set work queue delay time 200ms
	atomic_set(&obj->wake, 0);
	INIT_WORK(&obj->report, grav_work_func);
	init_timer(&obj->timer);
	obj->timer.expires	= jiffies + atomic_read(&obj->delay)/(1000/HZ);
	obj->timer.function	= grav_poll;
	obj->timer.data		= (unsigned long)obj;
	obj->is_first_data_after_enable = false;
	obj->is_polling_run = false;
	mutex_init(&obj->grav_op_mutex);
	obj->is_batch_enable = false;//for batch mode init
	obj->cali_sw[GRAV_AXIS_X]=0;
	obj->cali_sw[GRAV_AXIS_Y]=0;
	obj->cali_sw[GRAV_AXIS_Z]=0;
	GRAV_LOG("grav_context_alloc_object----\n");
	return obj;
}

static int grav_real_enable(int enable)
{
  int err =0;
  struct grav_context *cxt = NULL;

  GRAV_FUN();
 
  cxt = grav_context_obj;
  if(1==enable)
  {
     
     if(true==cxt->is_active_data || true ==cxt->is_active_nodata)
     {
        err = cxt->grav_ctl.enable_nodata(1);
        if(err)
        { 
           err = cxt->grav_ctl.enable_nodata(1);
		   if(err)
		   {
		   		err = cxt->grav_ctl.enable_nodata(1);
				if(err)
					GRAV_ERR("grav enable(%d) err 3 timers = %d\n", enable, err);
		   }
        }
		GRAV_LOG("grav real enable  \n" );
     }
	 
  }
  if(0==enable)
  {
     if(false==cxt->is_active_data && false ==cxt->is_active_nodata)
     {
        err = cxt->grav_ctl.enable_nodata(0);
        if(err)
        { 
          GRAV_ERR("grav enable(%d) err = %d\n", enable, err);
        }
		GRAV_LOG("grav real disable  \n" );
     }
	 
  }

  return err;
}
static int grav_enable_data(int enable)
{
    struct grav_context *cxt = NULL;

	GRAV_FUN();

	//int err =0;
	cxt = grav_context_obj;
	if(NULL  == cxt->grav_ctl.open_report_data)
	{
	  GRAV_ERR("no grav control path\n");
	  return -1;
	}
	
    if(1 == enable)
    {
       GRAV_LOG("GRAV enable data\n");
	   cxt->is_active_data =true;
       cxt->is_first_data_after_enable = true;
	   cxt->grav_ctl.open_report_data(1);
	grav_real_enable(enable);
	   if(false == cxt->is_polling_run && cxt->is_batch_enable == false)
	   {
	      if(false == cxt->grav_ctl.is_report_input_direct)
	      {
	      	mod_timer(&cxt->timer, jiffies + atomic_read(&cxt->delay)/(1000/HZ));
		  	cxt->is_polling_run = true;
	      }
	   }
    }
	if(0 == enable)
	{
	   GRAV_LOG("GRAV disable \n");
	   
	   cxt->is_active_data =false;
	   cxt->grav_ctl.open_report_data(0);
	   if(true == cxt->is_polling_run)
	   {
	      if(false == cxt->grav_ctl.is_report_input_direct)
	      {
	      	cxt->is_polling_run = false;
	      	del_timer_sync(&cxt->timer);
	      	cancel_work_sync(&cxt->report);
			cxt->drv_data.grav_data.values[0] = GRAV_INVALID_VALUE;
	   		cxt->drv_data.grav_data.values[1] = GRAV_INVALID_VALUE;
	   		cxt->drv_data.grav_data.values[2] = GRAV_INVALID_VALUE;
	      }
	   }
	grav_real_enable(enable);	   
	}
	return 0;
}



int grav_enable_nodata(int enable)
{
    struct grav_context *cxt = NULL;

	//int err =0;
	cxt = grav_context_obj;
	if(NULL  == cxt->grav_ctl.enable_nodata)
	{
	  GRAV_ERR("grav_enable_nodata:grav ctl path is NULL\n");
	  return -1;
	}
	
    if(1 == enable)
    {
       cxt->is_active_nodata = true;
    }
	
	if(0 == enable)
	{
	  cxt->is_active_nodata = false;
	}
	grav_real_enable(enable);
	return 0;
}


static ssize_t grav_show_enable_nodata(struct device* dev, 
                                 struct device_attribute *attr, char *buf) 
{
    int len = 0;

	GRAV_LOG(" not support now\n");
	return len;
}

static ssize_t grav_store_enable_nodata(struct device* dev, struct device_attribute *attr,
                                  const char *buf, size_t count)
{
	struct grav_context *cxt = NULL;
	//int err =0;

	GRAV_LOG("grav_store_enable nodata buf=%s\n",buf);	
	mutex_lock(&grav_context_obj->grav_op_mutex);
	
	cxt = grav_context_obj;
	if(NULL == cxt->grav_ctl.enable_nodata)
	{
		GRAV_LOG("grav_ctl enable nodata NULL\n");
		mutex_unlock(&grav_context_obj->grav_op_mutex);
	 	return count;
	}
	if (!strncmp(buf, "1", 1))
	{
		//cxt->grav_ctl.enable_nodata(1);
		grav_enable_nodata(1);
	}
	else if (!strncmp(buf, "0", 1))
	{
       //cxt->grav_ctl.enable_nodata(0);
       grav_enable_nodata(0);
    }
	else
	{
	  GRAV_ERR(" grav_store enable nodata cmd error !!\n");
	}
	mutex_unlock(&grav_context_obj->grav_op_mutex);
	return count;
}

static ssize_t grav_store_active(struct device* dev, struct device_attribute *attr,
                                  const char *buf, size_t count)
{
	struct grav_context *cxt = NULL;

	GRAV_LOG("grav_store_active buf=%s\n",buf);
	mutex_lock(&grav_context_obj->grav_op_mutex);
	cxt = grav_context_obj;
	if(NULL == cxt->grav_ctl.open_report_data)
	{
		GRAV_LOG("grav_ctl enable NULL\n");
		mutex_unlock(&grav_context_obj->grav_op_mutex);
	 	return count;
	}
    if (!strncmp(buf, "1", 1)) 
	{
      // cxt->grav_ctl.enable(1);
      grav_enable_data(1);
       
    } 
	else if (!strncmp(buf, "0", 1))
	{
        
       //cxt->grav_ctl.enable(0);
       grav_enable_data(0);
    }
	else
	{
	  GRAV_ERR(" grav_store_active error !!\n");
	}
	mutex_unlock(&grav_context_obj->grav_op_mutex);
	GRAV_LOG(" grav_store_active done\n");
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t grav_show_active(struct device* dev, 
                                 struct device_attribute *attr, char *buf) 
{
	struct grav_context *cxt = NULL;
	int div = 0;
	cxt = grav_context_obj;
	div = cxt->grav_data.vender_div;

	GRAV_LOG("grav vender_div value: %d\n", div);
	return snprintf(buf, PAGE_SIZE, "%d\n", div); 
}

static ssize_t grav_store_delay(struct device* dev, struct device_attribute *attr,
                                  const char *buf, size_t count)
{
   // struct grav_context *devobj = (struct grav_context*)dev_get_drvdata(dev);
    int delay = 0;
	int mdelay=0;
	struct grav_context *cxt = NULL;

	mutex_lock(&grav_context_obj->grav_op_mutex);
	//int err =0;
	cxt = grav_context_obj;
	if(NULL == cxt->grav_ctl.set_delay)
	{
		GRAV_LOG("grav_ctl set_delay NULL\n");
		mutex_unlock(&grav_context_obj->grav_op_mutex);
	 	return count;
	}

    if (1 != sscanf(buf, "%d", &delay)) {
        GRAV_ERR("invalid format!!\n");
		mutex_unlock(&grav_context_obj->grav_op_mutex);
        return count;
    }

    if(false == cxt->grav_ctl.is_report_input_direct)
    {
    	mdelay = (int)delay/1000/1000;
    	atomic_set(&grav_context_obj->delay, mdelay);
    }
    cxt->grav_ctl.set_delay(delay);
	GRAV_LOG(" grav_delay %d ns\n",delay);
	mutex_unlock(&grav_context_obj->grav_op_mutex);
    return count;
}

static ssize_t grav_show_delay(struct device* dev, 
                                 struct device_attribute *attr, char *buf) 
{
    int len = 0;
	GRAV_LOG(" not support now\n");
	return len;
}

static ssize_t grav_show_sensordevnum(struct device* dev, 
                                 struct device_attribute *attr, char *buf) 
{
	struct grav_context *cxt = NULL;
	const char *devname = NULL;
	cxt = grav_context_obj;
	devname = dev_name(&cxt->idev->dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", devname+5); 
}


static ssize_t grav_store_batch(struct device* dev, struct device_attribute *attr,
                                  const char *buf, size_t count)
{

	struct grav_context *cxt = NULL;

	//int err =0;
	GRAV_LOG("grav_store_batch buf=%s\n",buf);
	mutex_lock(&grav_context_obj->grav_op_mutex);
	cxt = grav_context_obj;
	if(cxt->grav_ctl.is_support_batch){
	    	if (!strncmp(buf, "1", 1)) 
		{
	    		cxt->is_batch_enable = true;
			// MTK problem fix - start
			if(cxt->is_active_data && cxt->is_polling_run )
			{
				  cxt->is_polling_run = false;
				  del_timer_sync(&cxt->timer);
				  cancel_work_sync(&cxt->report);
			}
			// MTK problem fix - end
	    	} 
		else if (!strncmp(buf, "0", 1))
		{
			cxt->is_batch_enable = false;
			// MTK problem fix - start
			if(cxt->is_active_data){
				grav_enable_data(true);				
			}
			// MTK problem fix - end
	    	}
		else
		{
			GRAV_ERR(" grav_store_batch error !!\n");
		}
	}else{
		GRAV_LOG(" grav_store_batch mot supported\n");
	}
	mutex_unlock(&grav_context_obj->grav_op_mutex);
	GRAV_LOG(" grav_store_batch done: %d\n", cxt->is_batch_enable);
    	return count;

}

static ssize_t grav_show_batch(struct device* dev, 
                                 struct device_attribute *attr, char *buf) 
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0); 
}

static ssize_t grav_store_flush(struct device* dev, struct device_attribute *attr,
                                  const char *buf, size_t count)
{
	//mutex_lock(&grav_context_obj->grav_op_mutex);
   // struct grav_context *devobj = (struct grav_context*)dev_get_drvdata(dev);
	//do read FIFO data function and report data immediately
	//mutex_unlock(&grav_context_obj->grav_op_mutex);
    return count;
}

static ssize_t grav_show_flush(struct device* dev, 
                                 struct device_attribute *attr, char *buf) 
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0); 
}

static int gravitysensor_remove(struct platform_device *pdev)
{
	GRAV_LOG("gravitysensor_remove\n");
	return 0;
}

static int gravitysensor_probe(struct platform_device *pdev) 
{
	GRAV_LOG("gravitysensor_probe\n");
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gravitysensor_of_match[] = {
	{ .compatible = "mediatek,gravitysensor", },
	{},
};
#endif

static struct platform_driver gravitysensor_driver = {
	.probe      = gravitysensor_probe,
	.remove     = gravitysensor_remove,    
	.driver     = 
	{
		.name  = "gravitysensor",
        #ifdef CONFIG_OF
		.of_match_table = gravitysensor_of_match,
		#endif
	}
};

static int grav_real_driver_init(void) 
{
    int i =0;
	int err=0;
	GRAV_LOG(" grav_real_driver_init +\n");
	for(i = 0; i < MAX_CHOOSE_GRAV_NUM; i++)
	{
	  GRAV_LOG(" i=%d\n",i);
	  if(0 != gravitysensor_init_list[i])
	  {
	    GRAV_LOG(" grav try to init driver %s\n", gravitysensor_init_list[i]->name);
	    err = gravitysensor_init_list[i]->init();
		if(0 == err)
		{
		   GRAV_LOG(" grav real driver %s probe ok\n", gravitysensor_init_list[i]->name);
		   break;
		}
	  }
	}

	if(i == MAX_CHOOSE_GRAV_NUM)
	{
	   GRAV_LOG(" grav_real_driver_init fail\n");
	   err=-1;
	}
	return err;
}

static int grav_misc_init(struct grav_context *cxt)
{

    int err=0;
    cxt->mdev.minor = MISC_DYNAMIC_MINOR;
	cxt->mdev.name  = GRAV_MISC_DEV_NAME;
	if((err = misc_register(&cxt->mdev)))
	{
		GRAV_ERR("unable to register grav misc device!!\n");
	}
	//dev_set_drvdata(cxt->mdev.this_device, cxt);
	return err;
}

static void grav_input_destroy(struct grav_context *cxt)
{
	struct input_dev *dev = cxt->idev;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int grav_input_init(struct grav_context *cxt)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = GRAV_INPUTDEV_NAME;

	input_set_capability(dev, EV_ABS, EVENT_TYPE_GRAV_X);
	input_set_capability(dev, EV_ABS, EVENT_TYPE_GRAV_Y);
	input_set_capability(dev, EV_ABS, EVENT_TYPE_GRAV_Z);
	input_set_capability(dev, EV_REL, EVENT_TYPE_GRAV_STATUS);
	
	input_set_abs_params(dev, EVENT_TYPE_GRAV_X, GRAV_VALUE_MIN, GRAV_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, EVENT_TYPE_GRAV_Y, GRAV_VALUE_MIN, GRAV_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, EVENT_TYPE_GRAV_Z, GRAV_VALUE_MIN, GRAV_VALUE_MAX, 0, 0);
	input_set_drvdata(dev, cxt);

	input_set_events_per_packet(dev, 32); //test

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	cxt->idev= dev;

	return 0;
}

DEVICE_ATTR(gravenablenodata,     S_IWUSR | S_IRUGO, grav_show_enable_nodata, grav_store_enable_nodata);
DEVICE_ATTR(gravactive,     S_IWUSR | S_IRUGO, grav_show_active, grav_store_active);
DEVICE_ATTR(gravdelay,      S_IWUSR | S_IRUGO, grav_show_delay,  grav_store_delay);
DEVICE_ATTR(gravbatch,      		S_IWUSR | S_IRUGO, grav_show_batch,  grav_store_batch);
DEVICE_ATTR(gravflush,      		S_IWUSR | S_IRUGO, grav_show_flush,  grav_store_flush);
DEVICE_ATTR(gravdevnum,      		S_IWUSR | S_IRUGO, grav_show_sensordevnum,  NULL);

static struct attribute *grav_attributes[] = {
	&dev_attr_gravenablenodata.attr,
	&dev_attr_gravactive.attr,
	&dev_attr_gravdelay.attr,
	&dev_attr_gravbatch.attr,
	&dev_attr_gravflush.attr,
	&dev_attr_gravdevnum.attr,
	NULL
};

static struct attribute_group grav_attribute_group = {
	.attrs = grav_attributes
};

int grav_register_data_path(struct grav_data_path *data)
{
	struct grav_context *cxt = NULL;
	//int err =0;
	cxt = grav_context_obj;
	cxt->grav_data.get_data = data->get_data;
	cxt->grav_data.get_raw_data = data->get_raw_data;
	cxt->grav_data.vender_div = data->vender_div;
	GRAV_LOG("grav register data path vender_div: %d\n", cxt->grav_data.vender_div);
	if(NULL == cxt->grav_data.get_data)
	{
		GRAV_LOG("grav register data path fail \n");
	 	return -1;
	}
	return 0;
}

int grav_register_control_path(struct grav_control_path *ctl)
{
	struct grav_context *cxt = NULL;
	int err =0;
	cxt = grav_context_obj;
	cxt->grav_ctl.set_delay = ctl->set_delay;
	cxt->grav_ctl.open_report_data= ctl->open_report_data;
	cxt->grav_ctl.enable_nodata = ctl->enable_nodata;
	cxt->grav_ctl.is_support_batch = ctl->is_support_batch;
	cxt->grav_ctl.is_report_input_direct= ctl->is_report_input_direct;
	cxt->grav_ctl.grav_calibration = ctl->grav_calibration;
	
	if(NULL==cxt->grav_ctl.set_delay || NULL==cxt->grav_ctl.open_report_data
		|| NULL==cxt->grav_ctl.enable_nodata)
	{
		GRAV_LOG("grav register control path fail \n");
	 	return -1;
	}

	//add misc dev for sensor hal control cmd
	err = grav_misc_init(grav_context_obj);
	if(err)
	{
	   GRAV_ERR("unable to register grav misc device!!\n");
	   return -2;
	}
	err = sysfs_create_group(&grav_context_obj->mdev.this_device->kobj,
			&grav_attribute_group);
	if (err < 0)
	{
	   GRAV_ERR("unable to create grav attribute file\n");
	   return -3;
	}
	
	kobject_uevent(&grav_context_obj->mdev.this_device->kobj, KOBJ_ADD);
	
	return 0;	
}

int grav_data_report(int x, int y, int z,int status)
{
    struct grav_context *cxt = NULL;
	int err =0;
	cxt = grav_context_obj;

	GRAV_LOG("grav_data_report! %d, %d, %d, %d\n",x,y,z,status);

  	input_report_abs(cxt->idev, EVENT_TYPE_GRAV_X, x);
	input_report_abs(cxt->idev, EVENT_TYPE_GRAV_Y, y);
	input_report_abs(cxt->idev, EVENT_TYPE_GRAV_Z, z);
	input_report_rel(cxt->idev, EVENT_TYPE_GRAV_STATUS, status);
	input_sync(cxt->idev); 
	return err;
}

static int grav_probe(struct platform_device *pdev) 
{

	int err;
	GRAV_LOG("+++++++++++++gravity_probe!!\n");

	grav_context_obj = grav_context_alloc_object();
	if (!grav_context_obj)
	{
		err = -ENOMEM;
		GRAV_ERR("unable to allocate devobj!\n");
		goto exit_alloc_data_failed;
	}

	//init real gravityeration driver
    err = grav_real_driver_init();
	if(err)
	{
		GRAV_ERR("grav real driver init fail\n");
		goto real_driver_init_fail;
	}

	//init input dev
	err = grav_input_init(grav_context_obj);
	if(err)
	{
		GRAV_ERR("unable to register grav input device!\n");
		goto exit_alloc_input_dev_failed;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_EARLYSUSPEND)
       atomic_set(&(grav_context_obj->early_suspend), 0);
	grav_context_obj->early_drv.level    = 1;//EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1,
	grav_context_obj->early_drv.suspend  = grav_early_suspend,
	grav_context_obj->early_drv.resume   = grav_late_resume,    
	register_early_suspend(&grav_context_obj->early_drv);
#endif
  
	GRAV_LOG("----gravity_probe OK !!\n");
	return 0;

	//exit_hwmsen_create_attr_failed:
	//exit_misc_register_failed:    

	//exit_err_sysfs:
	
	if (err)
	{
	   GRAV_ERR("sysfs node creation error \n");
	   grav_input_destroy(grav_context_obj);
	}
	
	real_driver_init_fail:
	exit_alloc_input_dev_failed:    
	kfree(grav_context_obj);
	
	exit_alloc_data_failed:
	

	GRAV_LOG("----gravity_probe fail !!!\n");
	return err;
}



static int grav_remove(struct platform_device *pdev)
{
	int err=0;

	GRAV_FUN(f);
	input_unregister_device(grav_context_obj->idev);        
	sysfs_remove_group(&grav_context_obj->idev->dev.kobj,
				&grav_attribute_group);
	
	if((err = misc_deregister(&grav_context_obj->mdev)))
	{
		GRAV_ERR("misc_deregister fail: %d\n", err);
	}
	kfree(grav_context_obj);

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_EARLYSUSPEND)
static void grav_early_suspend(struct early_suspend *h) 
{
   atomic_set(&(grav_context_obj->early_suspend), 1);
   GRAV_LOG(" grav_early_suspend ok------->hwm_obj->early_suspend=%d \n",atomic_read(&(grav_context_obj->early_suspend)));
   return ;
}

/*----------------------------------------------------------------------------*/

static void grav_late_resume(struct early_suspend *h)
{
   atomic_set(&(grav_context_obj->early_suspend), 0);
   GRAV_LOG(" grav_late_resume ok------->hwm_obj->early_suspend=%d \n",atomic_read(&(grav_context_obj->early_suspend)));
   return ;
}
#endif

static int grav_suspend(struct platform_device *dev, pm_message_t state) 
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int grav_resume(struct platform_device *dev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id m_grav_pl_of_match[] = {
	{ .compatible = "mediatek,m_grav_pl", },
	{},
};
#endif

  static struct platform_driver grav_driver =
  {
	  .probe	  = grav_probe,
	  .remove	  = grav_remove,    
	  .suspend	  = grav_suspend,
	  .resume	  = grav_resume,
	  .driver	  = 
	  {
		  .name = GRAV_PL_DEV_NAME,
		#ifdef CONFIG_OF
		.of_match_table = m_grav_pl_of_match,
		#endif
	  }
  };

  int grav_driver_add(struct grav_init_info* obj) 
{
    	int err=0;
	int i =0;
	
	GRAV_FUN();

	for(i =0; i < MAX_CHOOSE_GRAV_NUM; i++ )
	{
		if ((i == 0) && (NULL == gravitysensor_init_list[0])) {
			GRAV_LOG("register gensor driver for the first time\n");
			if(platform_driver_register(&gravitysensor_driver))
			{
				GRAV_ERR("failed to register gensor driver already exist\n");
			}
		}
		
	    if(NULL == gravitysensor_init_list[i])
	    {
	      obj->platform_diver_addr = &gravitysensor_driver;
	      gravitysensor_init_list[i] = obj;
		  break;
	    }
	}
	if (i >= MAX_CHOOSE_GRAV_NUM)
	{
	   GRAV_ERR("GRAV driver add err \n");
	   err=-1;
	}
	return err;
}
EXPORT_SYMBOL_GPL(grav_driver_add);

static int __init grav_init(void) 
{
	GRAV_FUN();

	if(platform_driver_register(&grav_driver))
	{
		GRAV_ERR("failed to register grav driver\n");
		return -ENODEV;
	}
	
	return 0;
}
static void __exit grav_exit(void)
{
	platform_driver_unregister(&grav_driver); 
	platform_driver_unregister(&gravitysensor_driver);      
}

late_initcall(grav_init);
//module_init(grav_init);
//module_exit(grav_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GRAVITYSENSOR device driver");
MODULE_AUTHOR("Mediatek");

