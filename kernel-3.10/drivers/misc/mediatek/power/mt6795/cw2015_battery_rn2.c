#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <mach/mt_gpio.h>
#include <linux/delay.h>

#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/board.h>

#define BAT_CHANGE_ALGORITHM

#ifdef BAT_CHANGE_ALGORITHM
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#define FILE_PATH "/data/lastsoc"
#define CPSOC  94
#define NORMAL_CYCLE 10
#endif

#include <linux/module.h>

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/ioctl.h>
//#include <linux/dev_info.h>
#include <linux/string.h>

#include <cust_charging.h>
#include <mach/charging.h>

/***************************************
 *      define 
 ***************************************/
#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        //ATHD = 0%

#define BATTERY_UP_MAX_CHANGE                   420         // the max time allow battery change quantity
#define BATTERY_DOWN_CHANGE                     60          // the max time allow battery change quantity
#define BATTERY_DOWN_MIN_CHANGE_RUN             30          // the min time allow battery change quantity when run
#define BATTERY_DOWN_MIN_CHANGE_SLEEP           1800        // the min time allow battery change quantity when run 30min
#define BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE   1800

#define SIZE_BATINFO            64

#define USB_CHARGER_MODE        1
#define AC_CHARGER_MODE         2

#define FG_CW2015_TAG                  "[FG_CW2015]"
#define FG_CW2015_FUN(f)               printk(KERN_ERR FG_CW2015_TAG"%s\n", __FUNCTION__)
#define FG_CW2015_ERR(fmt, args...)    printk(KERN_ERR FG_CW2015_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define FG_CW2015_LOG(fmt, args...)    printk(KERN_ERR FG_CW2015_TAG fmt, ##args)

#define I2C_BUSNUM          4
#define CW_I2C_SPEED        100000          // default i2c speed set 100khz
#define CW2015_DEV_NAME     "CW2015"

static u8 config_info[SIZE_BATINFO] = {
    0x17, 0xF3, 0x63, 0x6A, 0x6A, 0x68, 0x68, 0x65, 0x63, 0x60, 
    0x5B, 0x59, 0x65, 0x5B, 0x46, 0x41, 0x36, 0x31, 0x28, 0x27, 
    0x31, 0x35, 0x43, 0x51, 0x1C, 0x3B, 0x0B, 0x85, 0x22, 0x42, 
    0x5B, 0x82, 0x99, 0x92, 0x98, 0x96, 0x3D, 0x1A, 0x66, 0x45, 
    0x0B, 0x29, 0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82, 0x8C, 
    0x92, 0x96, 0x54, 0xC2, 0xBA, 0xCB, 0x2F, 0x7D, 0x72, 0xA5, 
    0xB5, 0xC1, 0xA5, 0x49
};
static u8 config_info_des[SIZE_BATINFO] = {
    0x17, 0xF9, 0x6D, 0x6D, 0x6B, 0x67, 0x65, 0x64, 0x58, 0x6D, 
    0x6D, 0x48, 0x57, 0x5D, 0x4A, 0x43, 0x37, 0x31, 0x2B, 0x20, 
    0x24, 0x35, 0x44, 0x55, 0x20, 0x37, 0x0B, 0x85, 0x2A, 0x4A, 
    0x56, 0x68, 0x74, 0x6B, 0x6D, 0x6E, 0x3C, 0x1A, 0x5C, 0x45, 
    0x0B, 0x30, 0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82, 0x8C, 
    0x92, 0x96, 0x64, 0xB4, 0xDB, 0xCB, 0x2F, 0x7D, 0x72, 0xA5, 
    0xB5, 0xC1, 0xA5, 0x42
};

struct cw_bat_platform_data {
    int is_dc_charge;
    int is_usb_charge;

    int bat_low_pin;
    int bat_low_level;
    int chg_ok_pin;
    int chg_ok_level;
    u8* cw_bat_config_info;
};

static struct cw_bat_platform_data cw_bat_platdata = {    
    .bat_low_pin    = 0,
    .bat_low_level  = 0,   
    .chg_ok_pin   = 0,
    .chg_ok_level = 0,   
    .is_usb_charge = 0,
    
    .cw_bat_config_info = config_info,
};

struct cw_battery {
    struct i2c_client *client;
    struct workqueue_struct *battery_workqueue;
    struct delayed_work battery_delay_work;
    struct delayed_work dc_wakeup_work;
    struct delayed_work bat_low_wakeup_work;
    struct cw_bat_platform_data *plat_data;
    
    struct power_supply rk_bat;
    struct power_supply rk_ac;
    struct power_supply rk_usb;
    
    long sleep_time_capacity_change;      // the sleep time from capacity change to present, it will set 0 when capacity change 
    long run_time_capacity_change;
    
    long sleep_time_charge_start;      // the sleep time from insert ac to present, it will set 0 when insert ac
    long run_time_charge_start;
    
    int dc_online;
    int usb_online;
    int charger_mode;
    int charger_init_mode;
    int capacity;
    int voltage;
    int status;
    int time_to_empty;
    int alt;
    
    int bat_change;
};

#ifdef BAT_CHANGE_ALGORITHM
struct cw_store{
    long bts;
    int OldSOC;
    int DetSOC;
    int AlRunFlag;
};
#endif

#ifdef BAT_CHANGE_ALGORITHM
static int PowerResetFlag = -1;
static int alg_run_flag = -1;
static int count_num = 0;
static int count_sp = 0;
#endif

int g_cw2015_capacity = 0;
int g_cw2015_vol = 0;

int battery_type_id = 1;

extern int FG_charging_type;
extern int FG_charging_status;
extern char* saved_command_line;

extern int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);

static void cw_get_battery_version()
{
    battery_type_id = simple_strtol(strstr(saved_command_line, "batversion=")+11, 0, 10);  //COS = 1, DES = 2     
    printk("[CW2015] [cw_get_battery_version] battery_type_id= %d\n", battery_type_id);
}

int get_capacity() 
{
    return g_cw2015_capacity;
}

int get_voltage() 
{
    return g_cw2015_vol;
}

#ifdef BAT_CHANGE_ALGORITHM
//????
static unsigned int cw_convertData(struct cw_battery *cw_bat,unsigned int ts)
{
    unsigned int i = ts%4096,n = ts/4096;
    unsigned int ret = 65536;
    
    if(i>=1700){i-=1700;ret=(ret*3)/4;}else{}
    if(i>=1700){i-=1700;ret=(ret*3)/4;}else{}
    if(i>=789){i-=789;ret=(ret*7)/8;}else{}
    if(i>=381){i-=381;ret=(ret*15)/16;}else{}
    if(i>=188){i-=188;ret=(ret*31)/32;}else{}
    if(i>=188){i-=188;ret=(ret*31)/32;}else{}
        if(i>=93){i-=93;ret=(ret*61)/64;}else{}
    if(i>=46){i-=46;ret=(ret*127)/128;}else{}
    if(i>=23){i-=23;ret=(ret*255)/256;}else{}
    if(i>=11){i-=11;ret=(ret*511)/512;}else{}
    if(i>=6){i-=6;ret=(ret*1023)/1024;}else{}
    if(i>=3){i-=3;ret=(ret*2047)/2048;}else{}
    if(i>=3){i-=3;ret=(ret*2047)/2048;}else{}
    
    return ret>>n;
}

//???
static int AlgNeed(struct cw_battery *cw_bat, int dSOC, int SOCT0)
{
    
    int dSOC1 = dSOC * 100;
    int SOCT01 = SOCT0 * 100;
    if(SOCT01 < 2400)
    {
        if ((dSOC1>-100)&&(dSOC1<((2400+SOCT01)/3)))
            return 1;
    }
    else if(SOCT01 < 4800)
    {
        if ((dSOC1>-100)&&(dSOC1<((7200-SOCT01)/3)))
            return 1;
    }
    else
    {
        if ((dSOC1>-100)&&(dSOC1<800))
            return 1;
    }
    
    return -1;
}

//???
static int cw_algorithm(struct cw_battery *cw_bat,int real_capacity)
{
    //struct timespec ts;
    struct file *file = NULL;
    struct cw_store st;
    struct inode *inode;
    mm_segment_t old_fs;
    
    unsigned int file_size = 0;
    
    int fileresult;
    int vmSOC;
    unsigned int utemp,utemp1;
    long timeNow;
    
    count_num = count_num + 1;
    if(count_num<count_sp)
    {
        return real_capacity;
    }
    else
    {
        count_num = 0;
    }
    timeNow = get_seconds();
    vmSOC = real_capacity;
    if(file == NULL)
        file = filp_open(FILE_PATH,O_RDWR|O_CREAT,0644);
    if(IS_ERR(file))
    {
        FG_CW2015_ERR(" error occured while opening file %s,exiting...\n",FILE_PATH);
        return real_capacity;
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    inode = file->f_dentry->d_inode;
    file_size = inode->i_size;
    if(file_size<sizeof(st))//(inode->i_size)<sizeof(st)
    {
        //st.bts = timeNow;
        //st.OldSOC = real_capacity;
        //st.DetSOC = 0;
        //st.AlRunFlag = -1;
        FG_CW2015_ERR("cw2015_file_test  file size error!\n");
        set_fs(old_fs);
        filp_close(file,NULL);
        file = NULL;
        return real_capacity;
    }
    else
    {
        file->f_pos = 0;
        vfs_read(file,(char*)&st,sizeof(st),&file->f_pos);
    }
   
    if(((st.bts)<0)||(st.OldSOC>100)||(st.OldSOC<0)||(st.DetSOC>16)||(st.DetSOC<-1))
    {
        FG_CW2015_ERR("cw2015_file_test  reading file error!\n");
        FG_CW2015_ERR("cw2015_file_test  st.bts = %ld st.OldSOC = %d st.DetSOC = %d st.AlRunFlag = %d  vmSOC = %d  2015SOC=%d\n",st.bts,st.OldSOC,st.DetSOC,st.AlRunFlag,vmSOC,real_capacity);
        
        set_fs(old_fs);
        filp_close(file,NULL);
        file = NULL;
        return real_capacity;
        //st.bts = timeNow;
        //st.OldSOC = real_capacity;
        //st.DetSOC = 0;
        //st.AlRunFlag = -1;
        //FG_CW2015_ERR("cw2015_file_test  reading file error!\n");
    }
    
  
    if(PowerResetFlag == 1)
    {
        PowerResetFlag = -1;
        st.DetSOC = st.OldSOC - real_capacity;
        
        if(AlgNeed(cw_bat ,st.DetSOC,st.OldSOC)==1)
            st.AlRunFlag = 1;
        else
            st.AlRunFlag = -1;
        
        if ((real_capacity<100)&&(real_capacity>CPSOC))
        {
            st.AlRunFlag = 1;
            vmSOC = 100;
            st.DetSOC = 100 - real_capacity;
        }
        //st.bts = timeNow;
        FG_CW2015_ERR("cw2015_file_test  PowerResetFlag == 1!\n");
        
    }
    else  if(((st.AlRunFlag) == 1)&&((st.DetSOC)!=0))
    {
        utemp1 = 32768/(st.DetSOC);
        if((st.bts)<timeNow)
            utemp = cw_convertData(cw_bat,(timeNow-st.bts));
        else
            utemp = cw_convertData(cw_bat,1);
        FG_CW2015_ERR("cw2015_file_test  convertdata = %d\n",utemp);
        if((st.DetSOC)<0)
            vmSOC = real_capacity-(int)((((unsigned int)((st.DetSOC)*(-1))*utemp)+utemp1)/65536);
        else
            vmSOC = real_capacity+(int)((((unsigned int)(st.DetSOC)*utemp)+utemp1)/65536);
        if (vmSOC == real_capacity)
        {
            st.AlRunFlag = -1;
            FG_CW2015_ERR("cw2015_file_test  algriothm end\n");
        }
    }
    else
    {
        count_sp = NORMAL_CYCLE;
        st.AlRunFlag = -1;
        st.bts = timeNow;
        FG_CW2015_ERR("cw2015_file_test  no algriothm\n");
    }
    //FG_CW2015_ERR("cw2015_file_test  sizeof(st) = %d filesize = %ld time = %d\n ",sizeof(st),(long)(inode->i_size),timeNow);
    //FG_CW2015_ERR("cw2015_file_test  st.bts = %ld st.OldSOC = %d st.DetSOC = %d st.AlRunFlag = %d  vmSOC = %d  2015SOC=%d\n",st.bts,st.OldSOC,st.DetSOC,st.AlRunFlag,vmSOC,real_capacity);
    //FG_CW2015_ERR("cw2015_file_test debugdata,\t%ld,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d,\t%d\n",timeNow,cw_bat->capacity,cw_bat->voltage,vmSOC,st.DetSOC,st.OldSOC,st.bts,st.AlRunFlag,real_capacity);
    alg_run_flag = st.AlRunFlag;
    if(vmSOC>100)
        vmSOC = 100;
    else if(vmSOC<0)
        vmSOC = 0;
    st.OldSOC = vmSOC;
    file->f_pos = 0;
    vfs_write(file,(char*)&st,sizeof(st),&file->f_pos);
    set_fs(old_fs);
    //FG_CW2015_ERR("cw2015_file_test rtc6 = %d  getSecond = %ld\n",ts.tv_sec,get_seconds());
    filp_close(file,NULL);
    file = NULL;
    
    return vmSOC; 
}
#endif

//ok
static int cw_read(struct i2c_client *client, u8 reg, u8 buf[])
{
    int ret = 0;
    ret = i2c_smbus_read_byte_data(client,reg);
    printk("cw_read buf = %d",ret);
    if (ret < 0)
        return ret;
    else
    {
        buf[0] = ret;
        ret = 0;
    }
    return ret;
}

//ok
static int cw_write(struct i2c_client *client, u8 reg, u8 const buf[])
{
    int ret = 0;
    ret =  i2c_smbus_write_byte_data(client,reg,buf[0]);
    return ret;
}

//ok
static int cw_read_word(struct i2c_client *client, u8 reg, u8 buf[])
{
    int ret = 0;
    unsigned int data = 0;
    data = i2c_smbus_read_word_data(client, reg);
    buf[0] = data & 0x00FF;
    buf[1] = (data & 0xFF00)>>8;
    return ret;
}

//ok
static int cw_get_vol(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val[2];
    u16 value16, value16_1, value16_2, value16_3;
    int voltage;

    FG_CW2015_LOG("cw_get_vol \n");
    
    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    value16 = (reg_val[0] << 8) + reg_val[1];
    
    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    value16_1 = (reg_val[0] << 8) + reg_val[1];
    
    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    value16_2 = (reg_val[0] << 8) + reg_val[1];
    
    if(value16 > value16_1)
    {
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }
    
    if(value16_1 > value16_2)
    {
        value16_3 =value16_1;
        value16_1 =value16_2;
        value16_2 =value16_3;
    }
    
    if(value16 > value16_1)
    {
        value16_3 =value16;
        value16 =value16_1;
        value16_1 =value16_3;
    }

    voltage = value16_1 * 312 / 1024;
    FG_CW2015_LOG("cw_get_vol 4444 voltage = %d\n",voltage);
    return voltage;
}

//ok
static int cw_update_config_info(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val;
    int i;
    u8 reset_val;
    
    FG_CW2015_LOG("func: %s-------\n", __func__);
    FG_CW2015_LOG("test cw_bat_config_info = 0x%x",cw_bat->plat_data->cw_bat_config_info[0]);
    
    /* make sure no in sleep mode */
    ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
    if (ret < 0)
        return ret;
    FG_CW2015_LOG("cw_update_config_info reg_val = 0x%x",reg_val);
    
    reset_val = reg_val;
    if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        FG_CW2015_ERR("Error, device in sleep mode, cannot update battery info\n");
        return -1;
    }
    
    /* update new battery info */
    for (i = 0; i < SIZE_BATINFO; i++) {
        ret = cw_write(cw_bat->client, REG_BATINFO + i, &(cw_bat->plat_data->cw_bat_config_info[i]));
        if (ret < 0) 
            return ret;
    }
    
    /* readback & check */
    for (i = 0; i < SIZE_BATINFO; i++) {
        ret = cw_read(cw_bat->client, REG_BATINFO + i, &reg_val);
        if (reg_val != cw_bat->plat_data->cw_bat_config_info[i])
            return -1;
    }
    
    /* set cw2015/cw2013 to use new battery info */
    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
        return ret;
    
    reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */
    reg_val &= 0x07;                /* clear ATHD */
    reg_val |= ATHD;                /* set ATHD */
    ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
        return ret;
    
    /* check 2015/cw2013 for ATHD & update_flag */ 
    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
        return ret;
    
    if (!(reg_val & CONFIG_UPDATE_FLG))
        FG_CW2015_LOG("update flag for new battery info have not set..\n");
        
    if ((reg_val & 0xf8) != ATHD) 
        FG_CW2015_LOG("the new ATHD have not set..\n");
    
    /* reset */
    reset_val &= ~(MODE_RESTART);
    reg_val = reset_val | MODE_RESTART;
    ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
    if (ret < 0)
        return ret;
    
    msleep(10);
    ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
    if (ret < 0)
        return ret;

    PowerResetFlag = 1;
    FG_CW2015_ERR("cw2015_file_test  set PowerResetFlag/n ");
    msleep(10);
    
    return 0;
}

//ok
static int cw_init(struct cw_battery *cw_bat)
{
    int ret;
    int i;
    u8 reg_val = MODE_SLEEP;
    
    cw_get_battery_version();
    switch (battery_type_id)
    {
        case 1:
            cw_bat->plat_data->cw_bat_config_info = config_info;
            break;
        case 2:
            cw_bat->plat_data->cw_bat_config_info = config_info_des;
            break;
        default:
            FG_CW2015_LOG("Battery type ID not match\n");
            cw_bat->plat_data->cw_bat_config_info = config_info;
    }

    if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        reg_val = MODE_NORMAL;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if (ret < 0)
            return ret;
    }

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
        return ret; 

    FG_CW2015_LOG("the new ATHD have not set reg_val = 0x%x\n",reg_val);
    
    if ((reg_val & 0xf8) != ATHD) {
        FG_CW2015_LOG("the new ATHD have not set\n");
        reg_val &= 0x07;    /* clear ATHD */
        reg_val |= ATHD;    /* set ATHD */
        ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0)
            return ret;
    }   

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0) 
        return ret;
    
    FG_CW2015_LOG("cw_init REG_CONFIG = %d\n",reg_val); 

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
        FG_CW2015_LOG("update flag for new battery info have not set\n");
        ret = cw_update_config_info(cw_bat);
        if (ret < 0)
            return ret;
    } else {
        for(i = 0; i < SIZE_BATINFO; i++) { 
            ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
            if (ret < 0)
                return ret; 

            if (cw_bat->plat_data->cw_bat_config_info[i] != reg_val)
                break;
        }
        
        if (i != SIZE_BATINFO) {
            FG_CW2015_LOG("update flag for new battery info have not set\n"); 
            ret = cw_update_config_info(cw_bat);
            if (ret < 0)
                return ret;
        }
    }   

    for (i = 0; i < 30; i++) {
        msleep(100);
        ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
        if (ret < 0)
            return ret;
        else if (reg_val <= 0x64) 
            break;
        
        if (i > 25)
            FG_CW2015_ERR("cw2015/cw2013 input unvalid power error\n");      
    }

    if (i >= 30) {
        reg_val = MODE_SLEEP;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        FG_CW2015_ERR("cw2015/cw2013 input unvalid power error_2\n");
        return -1;
    } 
    return 0;
}

//ok
static void cw_update_time_member_charge_start(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;
    
    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;
    
    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;
    
    cw_bat->run_time_charge_start = new_run_time;
    cw_bat->sleep_time_charge_start = new_sleep_time;
}

//ok
static void cw_update_time_member_capacity_change(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;
    
    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;
    
    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;
    
    cw_bat->run_time_capacity_change = new_run_time;
    cw_bat->sleep_time_capacity_change = new_sleep_time;
}

//ok
static int rk_usb_update_online(struct cw_battery *cw_bat)
{
    
    FG_CW2015_LOG("rk_usb_update_online FG_charging_status = %d\n", FG_charging_status);
    FG_CW2015_LOG("rk_usb_update_online FG_charging_type = %d\n", FG_charging_type);
    
    if (FG_charging_status) {
        
        if (FG_charging_type == STANDARD_HOST) {
            cw_bat->charger_mode = USB_CHARGER_MODE;
            if (cw_bat->usb_online != 1){
                cw_bat->usb_online = 1;
                cw_update_time_member_charge_start(cw_bat);
            }
            return 0;
        } else {
            cw_bat->charger_mode = AC_CHARGER_MODE;
            if (cw_bat->usb_online != 1){
                cw_bat->usb_online = 1;
                cw_update_time_member_charge_start(cw_bat);
            }
            return 0;
        }
    } else {
        cw_bat->charger_mode = FG_charging_status;
        
        if (cw_bat->usb_online == 0) {
            return 0;
        } else {
            cw_update_time_member_charge_start(cw_bat);
            cw_bat->usb_online = 0;
            return 1;
        }
    }
}

//ok
static int cw_get_capacity(struct cw_battery *cw_bat)
{
    int cw_capacity;
    int ret;
    u8 reg_val[2];
    
    struct timespec ts;
    long new_run_time;
    long new_sleep_time;
    long capacity_or_aconline_time;
    int allow_change;
    int allow_capacity;
    static int if_quickstart = 0;
    static int jump_flag =0;
    static int reset_loop =0;
    int charge_time;
    u8 reset_val;
    
    ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
    if (ret < 0)
        return ret;

    FG_CW2015_LOG("cw_get_capacity cw_capacity_0 = %d,cw_capacity_1 = %d\n",reg_val[0],reg_val[1]);
    cw_capacity = reg_val[0];

    if ((cw_capacity < 0) || (cw_capacity > 100)) {
        FG_CW2015_ERR("get cw_capacity error; cw_capacity = %d\n", cw_capacity);
        reset_loop++;
        
        if (reset_loop > 5) {   
            reset_val = MODE_SLEEP;
            ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
            if (ret < 0)
                return ret;
            reset_val = MODE_NORMAL;
            msleep(10);
            ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
            if (ret < 0)
                return ret;
            
            ret = cw_init(cw_bat);
            if (ret)
                return ret;
            reset_loop =0;
        }
        return cw_capacity;
    } else 
        reset_loop =0;
    
    if (cw_capacity == 0)
        FG_CW2015_LOG("the cw201x capacity is 0 !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
    else
        FG_CW2015_LOG("the cw201x capacity is %d, funciton: %s\n", cw_capacity, __func__);
    
    // ret = cw_read(cw_bat->client, REG_SOC + 1, &reg_val);
    #ifdef  BAT_CHANGE_ALGORITHM
    cw_capacity = cw_algorithm(cw_bat,cw_capacity);
    #endif
    
    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;
    
    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;
    printk("cw_get_capacity cw_bat->charger_mode = %d\n",cw_bat->charger_mode);
    
    if (((cw_bat->charger_mode > 0) && (cw_capacity <= (cw_bat->capacity - 1)) && (cw_capacity > (cw_bat->capacity - 9)))
        || ((cw_bat->charger_mode == 0) && (cw_capacity == (cw_bat->capacity + 1)))) {             // modify battery level swing
            
            if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) {
                cw_capacity = cw_bat->capacity;
            }
        }
        
        if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {     // avoid no charge full
            
            capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
            capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
            allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_UP_MAX_CHANGE;
            if (allow_change > 0) {
                allow_capacity = cw_bat->capacity + allow_change;
                cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
                jump_flag =1;
            } else if (cw_capacity <= cw_bat->capacity) {
                cw_capacity = cw_bat->capacity;
            }
            
        }
        
        else if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (jump_flag == 1)) {     // avoid battery level jump to CW_BAT
            capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
            capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
            allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_DOWN_CHANGE;
            if (allow_change > 0) {
                allow_capacity = cw_bat->capacity - allow_change;
                if (cw_capacity >= allow_capacity){
                    jump_flag =0;
                }
                else{
                    cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
                }
            } else if (cw_capacity <= cw_bat->capacity) {
                cw_capacity = cw_bat->capacity;
            }
        }
        
        if ((cw_capacity == 0) && (cw_bat->capacity > 1)) {              // avoid battery level jump to 0% at a moment from more than 2%
            allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
            allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);
            
            allow_capacity = cw_bat->capacity - allow_change;
            cw_capacity = (allow_capacity >= cw_capacity) ? allow_capacity: cw_capacity;
            FG_CW2015_LOG("report GGIC POR happened\n");
            
            reset_val = MODE_SLEEP;
            ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
            if (ret < 0)
                return ret;
            reset_val = MODE_NORMAL;
            msleep(10);
            ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
            if (ret < 0)
                return ret;
            
            ret = cw_init(cw_bat);
            if (ret)
                return ret;
            
        }
        
        #if 1
        if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
        {
            charge_time = new_sleep_time + new_run_time - cw_bat->sleep_time_charge_start - cw_bat->run_time_charge_start;
            if ((charge_time > BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE) && (if_quickstart == 0)) {
                reset_val = MODE_SLEEP;
                ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
                if (ret < 0)
                    return ret;
                reset_val = MODE_NORMAL;
                msleep(10);
                ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
                if (ret < 0)
                    return ret;
                
                ret = cw_init(cw_bat);
                if (ret)
                    return ret;      // if the cw_capacity = 0 the cw2015 will qstrt
                FG_CW2015_LOG("report battery capacity still 0 if in changing\n");
                if_quickstart = 1;
            }
        } else if ((if_quickstart == 1)&&(cw_bat->charger_mode == 0)) {
            if_quickstart = 0;
        }
        
        #endif
        
        #ifdef SYSTEM_SHUTDOWN_VOLTAGE
        if ((cw_bat->charger_mode == 0) && (cw_capacity <= 20) && (cw_bat->voltage <= SYSTEM_SHUTDOWN_VOLTAGE)){
            if (if_quickstart == 10){
                
                allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
                allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);
                
                allow_capacity = cw_bat->capacity - allow_change;
                cw_capacity = (allow_capacity >= 0) ? allow_capacity: 0;
                
                if (cw_capacity < 1){
                    cw_quickstart(cw_bat);
                    if_quickstart = 12;
                    cw_capacity = 0;
                }
            } else if (if_quickstart <= 10)
                if_quickstart =if_quickstart+2;
            FG_CW2015_LOG("the cw201x voltage is less than SYSTEM_SHUTDOWN_VOLTAGE !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
        } else if ((cw_bat->charger_mode > 0)&& (if_quickstart <= 12)) {
            if_quickstart = 0;
        }
        #endif
        return cw_capacity;
}

//ok
static void rk_bat_update_capacity(struct cw_battery *cw_bat)
{
    int cw_capacity;

    cw_capacity = cw_get_capacity(cw_bat);
    FG_CW2015_ERR("cw2015_file_test userdata,	%ld,	%d,	%d\n",get_seconds(),cw_capacity,cw_bat->voltage);

    if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
        cw_bat->capacity = cw_capacity;
        cw_bat->bat_change = 1;
        cw_update_time_member_capacity_change(cw_bat);       
        if (cw_bat->capacity == 0)
            FG_CW2015_LOG("report battery capacity 0 and will shutdown if no changing\n");    
    }
    FG_CW2015_LOG("rk_bat_update_capacity cw_capacity = %d\n",cw_bat->capacity);
}

//ok
static void rk_bat_update_vol(struct cw_battery *cw_bat)
{
    int ret;
    ret = cw_get_vol(cw_bat);
    if ((ret >= 0) && (cw_bat->voltage != ret)) {
        cw_bat->voltage = ret;
        cw_bat->bat_change = 1;
    }
}

//ok
static void cw_bat_work(struct work_struct *work)
{
    struct delayed_work *delay_work;
    struct cw_battery *cw_bat;
    int ret;

    FG_CW2015_FUN();
    
    delay_work = container_of(work, struct delayed_work, work);
    cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);
    
    ret = rk_usb_update_online(cw_bat);
    if (ret == 1) 
        rk_usb_update_online(cw_bat);

    if (cw_bat->usb_online == 1) 
        rk_usb_update_online(cw_bat);
        
    rk_bat_update_capacity(cw_bat);
    rk_bat_update_vol(cw_bat);
    g_cw2015_capacity = cw_bat->capacity;
    g_cw2015_vol = cw_bat->voltage;
    
    printk("cw_bat_work 777 vol = %d,cap = %d\n",cw_bat->voltage,cw_bat->capacity);
    
    if (cw_bat->bat_change) 
        cw_bat->bat_change = 0;
    
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10000));
}

/*----------------------------------------------------------------------------*/
static int cw2015_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    FG_CW2015_FUN(); 

    strcpy(info->type, CW2015_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int cw2015_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct cw_battery *cw_bat;
    int ret;
    
    FG_CW2015_FUN(); 
   
    mt_set_gpio_mode(GPIO_I2C4_SDA_PIN, GPIO_I2C4_SDA_PIN_M_SDA);
    mt_set_gpio_mode(GPIO_I2C4_SCA_PIN, GPIO_I2C4_SCA_PIN_M_SCL);

    cw_bat = kzalloc(sizeof(struct cw_battery), GFP_KERNEL);
    if (!cw_bat) {
        FG_CW2015_ERR("fail to allocate memory\n");
        return -ENOMEM;
    }
    
    memset(cw_bat, 0, sizeof(*cw_bat));
    
    i2c_set_clientdata(client, cw_bat);
    
    cw_bat->plat_data = client->dev.platform_data;
    cw_bat->client = client;
    cw_bat->plat_data = &cw_bat_platdata;
    ret = cw_init(cw_bat);
    
    if (ret) 
        return ret;
    
    cw_bat->dc_online = 0;
    cw_bat->usb_online = 0;
    cw_bat->charger_mode = 0;
    cw_bat->capacity = 1;
    cw_bat->voltage = 0;
    cw_bat->status = 0;
    cw_bat->time_to_empty = 0;
    cw_bat->bat_change = 0;
    
    cw_update_time_member_capacity_change(cw_bat);
    cw_update_time_member_charge_start(cw_bat);
    
    cw_bat->battery_workqueue = create_singlethread_workqueue("rk_battery");
    INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10));

    FG_CW2015_LOG("cw2015/cw2013 driver v1.2 probe sucess\n");
    return 0;
}

static int cw2015_i2c_remove(struct i2c_client *client)
{
    struct cw_battery *data = i2c_get_clientdata(client);
    
    FG_CW2015_FUN();    
    cancel_delayed_work(&data->battery_delay_work);
    i2c_unregister_device(client);
    kfree(data);
    return 0;
}

static int cw2015_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct cw_battery *cw_bat = i2c_get_clientdata(client);
    
    FG_CW2015_FUN(); 
    cancel_delayed_work(&cw_bat->battery_delay_work); 
    return 0;
}

static int cw2015_i2c_resume(struct i2c_client *client)
{
    struct cw_battery *cw_bat = i2c_get_clientdata(client);
    
    FG_CW2015_FUN(); 
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(100));
    return 0;
}

static const struct i2c_device_id FG_CW2015_i2c_id[] = {{CW2015_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_FG_CW2015={ I2C_BOARD_INFO("CW2015", 0x62)};

static struct i2c_driver cw2015_i2c_driver = {	
    .probe      = cw2015_i2c_probe,
    .remove     = cw2015_i2c_remove,
    .detect     = cw2015_i2c_detect,
    .suspend    = cw2015_i2c_suspend,
    .resume     = cw2015_i2c_resume,
    .id_table   = FG_CW2015_i2c_id,
    .driver = {
        .name           = CW2015_DEV_NAME,
    },
};

/*----------------------------------------------------------------------------*/
static int __init cw_bat_init(void)
{
    FG_CW2015_LOG("%s: \n", __func__); 
    printk("cw_bat_init\n");
    
    i2c_register_board_info(I2C_BUSNUM, &i2c_FG_CW2015, 1);
    if(i2c_add_driver(&cw2015_i2c_driver))
    {
        FG_CW2015_ERR("add driver error\n");
        return -1;
    }
    return 0;
}

static void __exit cw_bat_exit(void)
{
    FG_CW2015_LOG("%s: \n", __func__); 
    printk("cw_bat_exit\n");
}

module_init(cw_bat_init);
module_exit(cw_bat_exit);

MODULE_AUTHOR("xhc<xhc@rock-chips.com>");
MODULE_DESCRIPTION("cw2015/cw2013 battery driver");
MODULE_LICENSE("GPL");
