/*
   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   Copyright (c) 2011 Synaptics, Inc.

   Permission is hereby granted, free of charge, to any person obtaining a copy of
   this software and associated documentation files (the "Software"), to deal in
   the Software without restriction, including without limitation the rights to use,
   copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
   Software, and to permit persons to whom the Software is furnished to do so,
   subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
#include "RefCode_F54.h"
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>	/* msleep */
#include <linux/file.h>		/* for file access */
#include <linux/syscalls.h>	/* for file access */
#include <linux/uaccess.h>	/* for file access */
#include <linux/firmware.h>

static char line[49152] = { 0 };

int UpperImage[32][32];
int LowerImage[32][32];
int SensorSpeedUpperImage[32][32];
int SensorSpeedLowerImage[32][32];
int ADCUpperImage[32][32];
int ADCLowerImage[32][32];

/* extern struct i2c_client *ds4_i2c_client; */
/* extern struct i2c_client *tpd_i2c_client; */


/* extern int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf); */
/* extern int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 *buf); */

#include "s3320_driver.h"
/* extern int synaptics_ts_read(struct i2c_client *client, u8 reg, int num, u8 *buf); */
/* extern int synaptics_ts_write(struct i2c_client *client, u8 reg, u8 *buf, int len); */
/* extern int synaptics_ts_read_f54(struct i2c_client *client, u8 reg, int num, u8 *buf); */

int Read8BitRegisters(unsigned short regAddr, unsigned char *data, int length)
{
	/* I2C read */
	int rst = 0;

	if (ds4_i2c_client == NULL) {
		pr_debug("s3528 ds4_i2c_client is null");
		return -1;
	}

	rst = synaptics_ts_read(ds4_i2c_client, regAddr, length, data);
	if (rst < 0)
		TPD_ERR("Read8BitRegisters read fail\n");

	return rst;
}

int ReadF54BitRegisters(unsigned short regAddr, unsigned char *data, int length)
{
	TPD_FUN();
	/* I2C read */
	int rst = 0;

	if (ds4_i2c_client == NULL) {
		pr_debug("s3528 ds4_i2c_client is null");
		return -1;
	}

	rst = synaptics_ts_read_f54(ds4_i2c_client, regAddr, length, data);
	if (rst < 0)
		TPD_ERR("Read8BitRegisters read fail\n");

	return rst;
}



int Write8BitRegisters(unsigned short regAddr, unsigned char *data, int length)
{
	/* I2C write */
	int rst = 0;

	if (ds4_i2c_client == NULL) {
		pr_debug("s3528 ds4_i2c_client is null");
		return -1;
	}


	rst = synaptics_ts_write(ds4_i2c_client, regAddr, data, length);

	if (rst < 0)
		TPD_ERR("Write8BitRegisters read fail\n");

	return rst;
}

void delayMS_DS5(int val)
{
	/* Wait for val MS */
	msleep(val);
}

int write_file(char *filename, char *data)
{
	int fd = 0;

	fd = sys_open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
	if (fd < 0) {
		pr_debug("[s3528_write_file] :  Open file error [ %d ]\n", fd);
		return fd;
	} else {
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	return 0;
}


int write_log_DS5(char *filename, char *data)
{
	/* extern int f54_window_crack; */
	/* extern int f54_window_crack_check_mode; */

	int fd;
	char *fname = "/mnt/sdcard/touch_self_test.txt";
	int cap_file_exist = 0;

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	if (filename == NULL) {
		fd = sys_open(fname, O_WRONLY | O_CREAT | O_APPEND, 0666);
		pr_debug("write log in /mnt/sdcard/touch_self_test.txt\n");
	} else {
		fd = sys_open(filename, O_WRONLY | O_CREAT, 0666);
		pr_debug("write log in /sns/touch/cap_diff_test.txt\n");
	}

	pr_debug("[s3528-write_log]write file open %s, fd : %d\n", (fd >= 0) ? "success" : "fail",
		 fd);

	if (fd >= 0) {
		/*because of erro, this code is blocked. */
		/*if(sys_newstat((char __user *) fname, (struct stat *)&fstat) < 0) {
		   pr_debug("[Touch] cannot read %s stat info\n", fname);
		   } else {
		   if(fstat.st_size > 5 * 1024 * 1024) {
		   pr_debug("[Touch] delete %s\n", fname);
		   sys_unlink(fname);
		   sys_close(fd);

		   fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0644);
		   if(fd >= 0) {
		   sys_write(fd, data, strlen(data));
		   }
		   } else {
		   sys_write(fd, data, strlen(data));
		   }
		   sys_close(fd);
		   } */
		sys_write(fd, data, strlen(data));
		sys_close(fd);

		if (filename != NULL)
			cap_file_exist = 1;
	}
	set_fs(old_fs);

	return cap_file_exist;
}

/* seungmin.noh */
#if 0
void read_log(char *filename, const struct touch_platform_data *pdata)
{
	int fd;
	char *buf = NULL;
	int rx_num = 0;
	int tx_num = 0;
	int data_pos = 0;
	int offset = 0;
	int ret = 0;

	struct touch_platform_data *ppdata = (struct touch_platform_data *)pdata;

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_RDONLY, 0);
	buf = kzalloc(1024, GFP_KERNEL);

	pr_debug("[s3528-read_log]read file open %s, fd : %d\n", (fd >= 0) ? "success" : "fail",
		 fd);

	if (fd >= 0) {
		pr_debug("[s3528-read_log]open read_log funcion in /sns/touch/cap_diff_test.txt\n",
			 __func__);
		while (sys_read(fd, buf, 1024)) {

			pr_debug("[s3528-read_log]sys_read success\n");

			for (rx_num = 0; rx_num < (ppdata->rx_ch_count) - 1; rx_num++) {
				ret =
				    sscanf(buf + data_pos, "%d%n", &ppdata->rx_cap[rx_num],
					   &offset);
				if (ret > 0)
					data_pos += offset;
			}

			for (tx_num = 0; tx_num < (ppdata->tx_ch_count) - 1; tx_num++) {
				ret =
				    sscanf(buf + data_pos, "%d%n", &ppdata->tx_cap[tx_num],
					   &offset);
				if (ret > 0)
					data_pos += offset;
			}

			pr_debug("[s3528-read_log]rx_num = %d, tx_num = %d\n", rx_num, tx_num);
			pr_debug("[s3528-read_log]rx_ch_count = %d, tx_ch_count = %d\n",
				 ppdata->rx_ch_count, ppdata->tx_ch_count);

			if ((rx_num == (ppdata->rx_ch_count) - 1)
			    && (tx_num == (ppdata->tx_ch_count) - 1))
				break;
		}

		sys_close(fd);
	}

	kfree(buf);

	set_fs(old_fs);

}
#endif

#if 0
int get_limit(unsigned char Tx, unsigned char Rx, struct i2c_client client,
	      const struct touch_platform_data *pdata, char *breakpoint, int limit_data[32][32])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	int rx_num = 0;
	int tx_num = 0;
	const struct firmware *fwlimit = NULL;

	pr_debug("[s3528-get_limit] Tx=[%d], Rx=[%d]\n", (int)Tx, (int)Rx);
	pr_debug("[s3528-get_limit] breakpoint = [%s]\n", breakpoint);

	if (pdata->panel_spec == NULL) {
		pr_debug("panel_spec_file name is null\n");
		ret = -1;
		goto exit;
	}

	if (request_firmware(&fwlimit, pdata->panel_spec, &client.dev) < 0) {
		pr_debug(" request ihex is failed\n");
		ret = -1;
		goto exit;
	}

	strcpy(line, fwlimit->data);

	q = strstr(line, breakpoint) - line;

	if (q < 0) {
		pr_debug("failed to find breakpoint. The panel_spec_file is wrong");
		ret = -1;
		goto exit;
	}

	memset(limit_data, 0, (TRX_max * TRX_max) * 4);

	while (1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') && (line[q - p] <= '9'); p++) {
				limit_data[tx_num][rx_num] += ((line[q - p] - '0') * cipher);
				/* pr_debug("[r = %d]limit_data[%d][%d] = %d\n", r, tx_num,
				   rx_num, limit_data[tx_num][rx_num]); */
				cipher *= 10;
			}
			if (line[q - p] == '-') {
				limit_data[tx_num][rx_num] = (-1) * (limit_data[tx_num][rx_num]);
				/* pr_debug("[r = %d]limit_data[%d][%d] = %d\n", r, tx_num,
				   rx_num, limit_data[tx_num][rx_num]); */
			}
			r++;

			if (r % (int)Rx == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;

		if (r == (int)Tx * (int)Rx) {
			pr_debug("panel_spec_file scanning is success\n");
			break;
		}
	}

	if (fwlimit)
		release_firmware(fwlimit);

	return ret;

exit:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;

}
#endif

#if 0
void main(void)
	/* Please be informed this main() function & related functions are an example for host implementation */
{
	PowerOnSensor();
	delayMS(400);

	F54Test();
	PowerOffSensor();
}
#endif
