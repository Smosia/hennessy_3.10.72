/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include "tpd.h"
#include <linux/i2c.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "tpd_custom_ft5x0x.h"
#include "cust_gpio_usage.h"

#ifdef TPD_AUTO_UPGRADE_SUPPORT

#include "tpd_custom_upgrade.h"


#define TPD_PACKET_LENGTH  128
#define TPD_NULL		0x0
#define TPD_TRUE		0x1
#define TPD_FALSE		0x0

typedef unsigned char		TPD_BYTE;
typedef unsigned short		TPD_WORD;
typedef unsigned int		TPD_DWRD;
typedef unsigned char		TPD_BOOL;

typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC,
	ERR_DL_ERASE_FAIL,
	ERR_DL_PROGRAM_FAIL,
	ERR_DL_VERIFY_FAIL
}UPGRADE_ERR_TYPE;

static struct tpd_contain
{
	unsigned int size;
	unsigned char *tpd_pt;
};

static struct tpd_contain tpd_compat;
static unsigned char tpd_supplier = 0;

extern u8 *tpd_i2c_dma_va;
extern u32 tpd_i2c_dma_pa;

static void tpd_fw_init()
{
#ifdef CTP_DETECT_SUPPLIER_THROUGH_GPIO
	int tpd_choice[2] = {0};

#ifdef GPIO_CTP_COMPAT_PIN1
	mt_set_gpio_mode(GPIO_CTP_COMPAT_PIN1, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_COMPAT_PIN1, GPIO_DIR_IN);
	tpd_choice[0] = mt_get_gpio_in(GPIO_CTP_COMPAT_PIN1);
#endif

#ifdef GPIO_CTP_COMPAT_PIN2
	mt_set_gpio_mode(GPIO_CTP_COMPAT_PIN2, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_COMPAT_PIN2, GPIO_DIR_IN);
	tpd_choice[1] = mt_get_gpio_in(GPIO_CTP_COMPAT_PIN2);
#endif
    TPD_DMESG("[TPD] tpd_choice[0] = 0x%x,tpd_choice[1] = 0x%x\n", tpd_choice[0], tpd_choice[1]);

	if(tpd_choice[0] || tpd_choice[1])
	{
		tpd_compat.tpd_pt = TPD_FW0;
		tpd_compat.size = sizeof(TPD_FW0);
		tpd_supplier = TPD_SUPPLIER_0;
	}
	else
	{
		tpd_compat.tpd_pt = TPD_FW1;
		tpd_compat.size = sizeof(TPD_FW1);
		tpd_supplier = TPD_SUPPLIER_1;
	}
#else //CTP_DETECT_SUPPLIER_THROUGH_GPIO
	tpd_compat.tpd_pt = TPD_FW;
	tpd_compat.size = sizeof(TPD_FW);
#endif //CTP_DETECT_SUPPLIER_THROUGH_GPIO
}

static int tpd_allow_upgrade(TPD_BYTE tpd_fw_ver, TPD_BYTE host_fw_ver)
{
#ifdef TPD_ALWAYS_UPGRADE_FW
	return 1;
#elif CTP_DETECT_SUPPLIER_THROUGH_GPIO
	switch(tpd_supplier)
	{
		case TPD_SUPPLIER_0:
			if(tpd_fw_ver != host_fw_ver)
				return 1;
			break;
		case TPD_SUPPLIER_1:
			if(tpd_fw_ver != host_fw_ver)
				return 1;
			break;
		case TPD_SUPPLIER_2:
			if(tpd_fw_ver != host_fw_ver)
				return 1;
			break;
		case TPD_SUPPLIER_3:
			if(tpd_fw_ver != host_fw_ver)
				return 1;
			break;
		default:
			break;
	}
	return 0;
#else
	if(tpd_fw_ver != host_fw_ver)
		return 1;
	return 0;
#endif
}

static void tpd_delay_ms(unsigned long ms)
{
	unsigned long i, j;

	for(i = 0; i < ms; i++)
	{
		for(j = 0; j < 1000; j++)
			udelay(1);
	}
}

static TPD_BOOL tpd_i2c_read_interface(struct i2c_client *client, TPD_BYTE *pbt_buf, TPD_DWRD sw_length)
{
	int ret;

	client->addr = client->addr & ~I2C_DMA_FLAG;
	ret = i2c_master_recv(client, pbt_buf, sw_length);
	if(ret <= 0)
	{
		TPD_DMESG("tpd_i2c_read_interface error\n");
		return TPD_FALSE;
	}

	return TPD_TRUE;
}

static TPD_BOOL tpd_i2c_write_interface(struct i2c_client *client, TPD_BYTE *pbt_buf, TPD_DWRD sw_length)
{
	int ret;

	client->addr = client->addr & ~I2C_DMA_FLAG;
	ret = i2c_master_send(client, pbt_buf, sw_length);
	if(ret < 0)
	{
		TPD_DMESG("tpd_i2c_write_interface error\n");
		return TPD_FALSE;
	}

	return TPD_TRUE;
}

static TPD_BYTE tpd_register_read(struct i2c_client *client, TPD_BYTE reg_name, TPD_BYTE *pbt_buf, TPD_BYTE bt_len)
{
	TPD_BYTE read_cmd[3] = {0};
	TPD_BYTE cmd_len = 0;

	read_cmd[0] = reg_name;
	cmd_len = 1;

	/*call the write callback function*/
	if(!tpd_i2c_write_interface(client, read_cmd, cmd_len))
		return TPD_FALSE;

	/*call the read callback function to get the register value*/
	if(!tpd_i2c_read_interface(client, pbt_buf, bt_len))
		return TPD_FALSE;

	return TPD_TRUE;
}

static TPD_BOOL tpd_register_write(struct i2c_client *client, TPD_BYTE reg_name, TPD_BYTE bt_value)
{
	TPD_BYTE write_cmd[2] = {0};

	write_cmd[0] = reg_name;
	write_cmd[1] = bt_value;

	/*call the write callback function*/
	return tpd_i2c_write_interface(client, write_cmd, 2);
}

static TPD_BOOL tpd_cmd_write(struct i2c_client *client, TPD_BYTE btcmd, TPD_BYTE btPara1, TPD_BYTE btPara2, TPD_BYTE btPara3, TPD_BYTE num)
{
	TPD_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;

	return tpd_i2c_write_interface(client, write_cmd, num);
}

static TPD_BOOL tpd_byte_write(struct i2c_client *client, TPD_BYTE *pbt_buf, TPD_DWRD dw_len)
{
	return tpd_i2c_write_interface(client, pbt_buf, dw_len);
}

static TPD_BOOL tpd_byte_read(struct i2c_client *client, TPD_BYTE* pbt_buf, TPD_BYTE bt_len)
{
	return tpd_i2c_read_interface(client, pbt_buf, bt_len);
}

static int tpd_dma_write(struct i2c_client *client, TPD_BYTE *pbt_buf, TPD_DWRD dw_len)
{
	int i = 0;
	for(i = 0; i < dw_len; i++)
		tpd_i2c_dma_va[i] = pbt_buf[i];

	if(dw_len <= 8)
	{
		client->addr = client->addr & ~I2C_DMA_FLAG;
		return i2c_master_send(client, pbt_buf, dw_len);
	}
	else
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		return i2c_master_send(client, tpd_i2c_dma_pa, dw_len);
	}
}

static int tpd_dma_read(struct i2c_client *client, TPD_BYTE *pbt_buf, TPD_BYTE bt_len)
{
	int ret = -1;
	int retries = 0;
	struct i2c_msg msgs[2];

	if (!client || !tpd_i2c_dma_va )
	{
		/*pr_err("%s, i2c_client or icn83xx_i2c_dma_va is null pointer\n");*/
		pr_err("i2c_client or icn83xx_i2c_dma_va is null pointer\n");
		return -1;
	}

		msgs[0].addr =client->addr | I2C_DMA_FLAG;
		msgs[0].flags = 0;
		msgs[0].len = 2;
		msgs[0].buf = tpd_i2c_dma_pa;
//		msgs[0].ext_flag = 0;
//		msgs[0].timing = 400;

		msgs[1].addr = client->addr | I2C_DMA_FLAG;
		msgs[1].flags = I2C_M_RD;
		msgs[1].len = bt_len;
		msgs[1].buf = tpd_i2c_dma_pa;		
//		msgs[1].ext_flag = 0;
//		msgs[1].timing = 400;
		
//		icn83xx_i2c_dma_va[0] = U16HIBYTE(addr);
//		icn83xx_i2c_dma_va[1] = U16LOBYTE(addr);
	
		
		while(retries < 3)
		{
			ret = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
			if (ret == sizeof(msgs)/sizeof(msgs[0]))
				break;
			retries++;
		}

		if(retries >= 3)
		{
			pr_err("%s i2c read error: %d, rxdata_len = %d\n", __func__, ret, bt_len);
		}
		else
		{
			int i = 0;
			for (i = 0; i<bt_len; i++)
			{
				pbt_buf[i] = tpd_i2c_dma_va[i];
			}
		}
	return ret;
}

static TPD_BYTE tpd_read_fw_ver(struct i2c_client *client)
{
	TPD_BYTE ver;

	i2c_smbus_read_i2c_block_data(client, 0xA6, 1, &ver);

	return ver;
}

//add by liuhuan
int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr & ~I2C_DMA_FLAG,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr & ~I2C_DMA_FLAG,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			pr_err("f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr & ~I2C_DMA_FLAG,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr & ~I2C_DMA_FLAG,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error.\n", __func__);

	return ret;
}


int ft5x0x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;
				
	return ft5x0x_i2c_Write(client, buf, sizeof(buf));
}

int ft5x0x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
			return ft5x0x_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

//add end


static TPD_BYTE tpd_get_i_file_ver()
{
	TPD_DWRD ui_sz;

	ui_sz = tpd_compat.size;
	if(ui_sz > 2)
		return tpd_compat.tpd_pt[ui_sz - 2];

	return 0xFF;
}

static UPGRADE_ERR_TYPE tpd_fw_upgrade(struct i2c_client *client, unsigned char *pbt_buf, unsigned int dw_length)
{
	TPD_BYTE reg_val[2] = {0};
	TPD_DWRD i = 0;
	TPD_DWRD packet_number;
	TPD_DWRD j;
	TPD_DWRD temp;
	TPD_DWRD length;
	TPD_BYTE packet_buf[TPD_PACKET_LENGTH + 6];
	TPD_BYTE auc_i2c_write_buf[10];
	TPD_BYTE bt_ecc;
	int ret;
        int IS_FT5436_NEW_BOOTLOADER = 0;
	TPD_BYTE val = 0;
	
	
	/********** Step 1: Reset CTPM **********/
	
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	//tpd_delay_ms(30);
	
	//msleep(30);//liuhuan
	/*
//add by liuhuan
	Step 1:Reset  CTPM
	*write 0xaa to register 0xfc
	
	ret=ft5x0x_write_reg(client, 0xfc, 0xaa);
	if(ret<0)
	{
		pr_err("liuhuan---write 0xaa to 0xfc---\n");
		return -1;
			}
	//msleep(50);//ok
	msleep(50);
			
	//write 0x55 to register 0xfc 
	ft5x0x_write_reg(client, 0xfc, 0x55);
	if(ret<0)
	{
		pr_err("liuhuan---write 0x55 to 0xfc---\n");
		return -1;
			}
	//msleep(30);//ok  */
	msleep(30);//
//add end
	
	
	
	/********** Step 2: Enter upgrade mode **********/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xAA;
	do
	{
		i++;
		ret = tpd_i2c_write_interface(client, auc_i2c_write_buf, 2);
		tpd_delay_ms(10);
		//msleep(5);
	} while ((ret <= 0) && (i < 5));
	TPD_DMESG("[TPD] Step 2: Enter update mode\n");
	//udelay(10);
	msleep(1);//liuhuan
	/********** Step 3: check READ-ID **********/
	/*send the opration head*/
	do{
		if(i > 3)
		{
			return ERR_READID;
		}

		/*read out the CTPM ID*/
		tpd_cmd_write(client, 0x90, 0x00, 0x00, 0x00, 4);
		tpd_byte_read(client, reg_val, 2);
		i++;
		TPD_DMESG("Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
while(reg_val[0] != 0x79 || reg_val[1] != 0x11);
/* 
#ifdef FT5X36_UPGADE
    while(reg_val[0] != 0x79 || reg_val[1] != 0x11);
#else
    while(reg_val[0] != 0x79 || reg_val[1] != 0x03);
#endif */

	tpd_cmd_write(client, 0xCD, 0x0, 0x00, 0x00, 1);
	tpd_byte_read(client, reg_val, 1);
	TPD_DMESG("[TPD] bootloader version = 0x%x\n", reg_val[0]);
        if (reg_val[0] > 4)
           IS_FT5436_NEW_BOOTLOADER = 1;
     

	/********** Step 4: Erase app and panel parameter area **********/
	tpd_cmd_write(client, 0x61, 0x00, 0x00, 0x00, 1); //Erase app area
	//tpd_delay_ms(1500);
	msleep(1500);
	//msleep(1800);//liuhuan
	tpd_cmd_write(client, 0x63, 0x00, 0x00, 0x00, 1); //Erase panel parameter area
	tpd_delay_ms(100);
	//msleep(100);//liuhuan
	TPD_DMESG("[TPD] Step 4: Erase.\n");

	/********** Step 5: Write firmware(FW) to ctpm flash **********/
	bt_ecc = 0;
	TPD_DMESG("[TPD] Step 5: Start upgrade.\n");
	if (dw_length <= 14)
	{
        return ret; 
	}
	
	dw_length = dw_length - 14;
	packet_number = (dw_length) / TPD_PACKET_LENGTH;
	packet_buf[0] = 0xBF;
	packet_buf[1] = 0x00;

	for(j = 0; j < packet_number; j++)
	{
		temp = j * TPD_PACKET_LENGTH;
		packet_buf[2] = (TPD_BYTE)(temp >> 8);
		packet_buf[3] = (TPD_BYTE)temp;
		length = TPD_PACKET_LENGTH;
		packet_buf[4] = (TPD_BYTE)(length >> 8);
		packet_buf[5] = (TPD_BYTE)length;

		for(i = 0; i < TPD_PACKET_LENGTH; i++)
		{
			packet_buf[6 + i] = pbt_buf[j * TPD_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		TPD_DMESG("[TPD] dma start .\n");
		ret=tpd_dma_write(client, &packet_buf[0], TPD_PACKET_LENGTH + 6);
		TPD_DMESG("[TPD] dma end.\n");
		if(ret < 0)
			{
				pr_err("liuhuan----tpd dma write---err\n");
				return ret;	
					}
		//tpd_byte_write(client, &packet_buf[0], TPD_PACKET_LENGTH + 6);

		//msleep(TPD_PACKET_LENGTH/6 + 1);//ok
		//msleep(5);
		tpd_delay_ms(20);
		//deleted by liuhuan
		/*
		if((j * TPD_PACKET_LENGTH % 1024) == 0)
		{
			TPD_DMESG("[TPD] upgrade the 0x%xth byte. \n", ((unsigned int)j) * TPD_PACKET_LENGTH);
		}
		*/
	}

	TPD_DMESG("[TPD] Step 6: Start dma .\n");

	if((dw_length) % TPD_PACKET_LENGTH > 0)
	{
		temp = packet_number * TPD_PACKET_LENGTH;
		packet_buf[2] = (TPD_BYTE)(temp >> 8);
		packet_buf[3] = (TPD_BYTE)temp;

		temp = (dw_length) % TPD_PACKET_LENGTH;
		packet_buf[4] = (TPD_BYTE)(temp >> 8);
		packet_buf[5] = (TPD_BYTE)temp;

		for(i = 0; i < temp; i++)
		{
			packet_buf[6 + i] = pbt_buf[packet_number * TPD_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ret=tpd_dma_write(client, &packet_buf[0], temp + 6);
		if(ret < 0)
			{
				pr_err("liuhuan----tpd dma write---err1\n");
				return ret;	
					}
		//tpd_byte_write(client, &packet_buf[0], temp + 6);
		tpd_delay_ms(20);
		//msleep(20);
	}

	//Send the last six byte
	for(i = 0; i < 12; i++)
	{
	//temp = 0x7FF4 + i;		

           if (IS_FT5436_NEW_BOOTLOADER )
		temp = 0x7FF4 + i;
           else
		temp = 0x6FF4 + i;


		packet_buf[2] = (TPD_BYTE)(temp >> 8);
		packet_buf[3] = (TPD_BYTE)temp;
		temp = 1;
		packet_buf[4] = (TPD_BYTE)(temp >> 8);
		packet_buf[5] = (TPD_BYTE)temp;
		packet_buf[6] = pbt_buf[dw_length + i];
		bt_ecc ^= packet_buf[6];

		tpd_dma_write(client, &packet_buf[0], 7);
		//tpd_byte_write(client, &packet_buf[0], 7);
		tpd_delay_ms(20);
		//msleep(1);
	}

	TPD_DMESG("[TPD] Step 7: Start to read out checksum.\n");
	/********** Step 6: Read out checksum **********/
	/* Send the operation head */
//	tpd_cmd_write(client, 0xCC, 0x00, 0x00, 0x00, 1);
	auc_i2c_write_buf[0]=0xcc;
	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, &val, 1);
	if (val != bt_ecc)
	{
		pr_err("liuhuan------checksum---val:0x%x --- bt_ecc:0x%x\n", val, bt_ecc);
		return -1;
	}
//	msleep(300); //make sure CTP startup normally
	//msleep(50); //make sure CTP startup normally //liuhuan



    TPD_DMESG("[TPD] Step 8: Start to write.\n");
	/***********************************************/
	auc_i2c_write_buf[0]=0x07;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);

	TPD_DMESG("[TPD] Step 9: End upgrade.\n");	

	return ERR_OK;
}

static int tpd_auto_calibrate(struct i2c_client *client)
{
	TPD_BYTE temp;
	TPD_BYTE i;

	TPD_DMESG("[TPD] start auto calibrate.\n");
	msleep(200);
	//msleep(30);//liuhuan

	temp = 0x40;
	i2c_smbus_write_i2c_block_data(client, 0x00, 1, &temp);
	tpd_delay_ms(100);	//make sure already enter factory mode
	//msleep(100);//liuhuan
	temp = 0x4;
	i2c_smbus_write_i2c_block_data(client, 0x02, 1, &temp);	//write command to start calibration
	tpd_delay_ms(300);
	//msleep(300);//liuhuan
	for(i = 0; i< 100; i++)
	{
		i2c_smbus_read_i2c_block_data(client, 0x02, 1, &temp);
		if(((temp & 0x70) >> 4) == 0x0)	//return to normal mode, calibration finish
			break;
		tpd_delay_ms(200);
		//msleep(20);//liuhuan
		TPD_DMESG("[TPD] waiting calibration %d\n", i);
	}
	TPD_DMESG("[TPD] calibration OK.\n");

	//msleep(300);
	msleep(300);//liuhuan
	temp = 0x40;
	i2c_smbus_write_i2c_block_data(client, 0x00, 1, &temp); //goto factory mode
	tpd_delay_ms(100); //make sure already enter factory mode
	//msleep(100);//liuhuan
	temp = 0x5;
	i2c_smbus_write_i2c_block_data(client, 0x02, 1, &temp); //store calibration result
	tpd_delay_ms(300);
//	msleep(300);//liuhuan
	temp = 0x0;
	i2c_smbus_write_i2c_block_data(client, 0x0, 1, &temp); //return to normal mode
	//msleep(300);
	msleep(300);//liuhuan

	TPD_DMESG("[TPD] store calibration result OK.\n");

	return 0;
}

static int tpd_fw_upgrade_with_i_file(struct i2c_client *client)
{
	TPD_BYTE *pbt_buf = 0x0;
	int ret;

	pbt_buf = tpd_compat.tpd_pt;

	/* Call the upgrade function */
	ret = tpd_fw_upgrade(client, pbt_buf, tpd_compat.size);
	if(ret != 0)
	{
		TPD_DMESG("[TPD] upgrade failed ret = %d.\n", ret);
	}
	else
	{
		TPD_DMESG("[TPD] upgrade successfully.\n");
		//tpd_auto_calibrate(client);
	}

	return ret;
} 

int tpd_auto_upgrade(struct i2c_client *client)
{
	TPD_BYTE host_fm_ver;
	TPD_BYTE tp_fm_ver;
	int ret;

	tpd_fw_init();
        msleep(300);
	tp_fm_ver = tpd_read_fw_ver(client);
	host_fm_ver = tpd_get_i_file_ver();

	TPD_DMESG("[TPD] tp_fm_ver = 0x%x, host_fm_ver = 0x%x\n", tp_fm_ver, host_fm_ver);

	if(tpd_allow_upgrade(tp_fm_ver, host_fm_ver))
	{
		msleep(100);//liuhuan

		ret = tpd_fw_upgrade_with_i_file(client);
		if(ret == 0)
		{
			//msleep(300);
			i2c_smbus_write_i2c_block_data(client,  0xA6, 1, &host_fm_ver);
			msleep(300);//liuhuan
			host_fm_ver = tpd_read_fw_ver(client);
			TPD_DMESG("[TPD] upgrade to new version 0x%x\n", host_fm_ver);
		}
		else
		{
			TPD_DMESG("[TPD] upgrade failed ret = %d.\n", ret);
		}
	}

	return 0;
}

#endif //TPD_AUTO_UPGRADE_SUPPORT
