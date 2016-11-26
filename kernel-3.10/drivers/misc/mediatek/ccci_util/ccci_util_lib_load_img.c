#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>
#include <linux/slab.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#ifdef CONFIG_OF
#include <linux/of_fdt.h>
#endif
#include <asm/setup.h>
#include <linux/atomic.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot_common.h>
#include <mach/sec_osal.h>
#include <mach/mt_sec_export.h>
#include <mach/mt_ccci_common.h>
#include "ccci_util_log.h"
#include "ccci_util_lib_main.h"
#if defined(CONFIG_MTK_AEE_FEATURE)
#include <linux/aee.h>
#endif
#define ENABLE_MEM_SIZE_CHECK
#define MAX_MD_NUM (6)		/* Max 4 internal + Max 2 exteranl */

/*==================================================== */
/* Image process section */
/*==================================================== */
#define IMG_POST_FIX_LEN	(32)
#define AP_PLATFORM_LEN		(16)
/*Note: must sync with sec lib, if ccci and sec has dependency change */
#define CURR_SEC_CCCI_SYNC_VER	(1)
static char *type_str[] = {[md_type_invalid] = "invalid",
	[modem_2g] = "2g",
	[modem_3g] = "3g",
	[modem_wg] = "wg",
	[modem_tg] = "tg",
	[modem_lwg] = "lwg",
	[modem_ltg] = "ltg",
	[modem_sglte] = "sglte",
	[modem_ultg] = "ultg",
	[modem_ulwg] = "ulwg",
	[modem_ulwtg] = "ulwtg",
	[modem_ulwcg] = "ulwcg",
	[modem_ulwctg] = "ulwctg",
	[modem_ulttg] = "ulttg",
	[modem_ulfwg] = "ulfwg",
	[modem_ulfwcg] = "ulfwcg",
	[modem_ulctg] = "ulctg",
	[modem_ultctg] = "ultctg"
};

static int curr_ubin_id;
static char *product_str[] = {[INVALID_VARSION] = INVALID_STR,
	[DEBUG_VERSION] = DEBUG_STR,
	[RELEASE_VERSION] = RELEASE_STR
};

static struct md_check_header md_img_header[MAX_MD_NUM];
/*static struct md_check_header_v3 md_img_header_v3[MAX_MD_NUM];*/
static struct md_check_header_v4 md_img_header_v4[MAX_MD_NUM];
static struct md_check_header_v5 md_img_header_v5[MAX_MD_NUM];
/*static struct ccci_image_info		img_info[MAX_MD_NUM][IMG_NUM]; */
char md_img_info_str[MAX_MD_NUM][256];

/*--- MD header check ------------ */
static int check_dsp_header(int md_id, void *parse_addr, struct ccci_image_info *image)
{
	return 0;
}

int scan_image_list(int md_id, char fmt[], int out_img_list[], int img_list_size)
{
	int i;
	int img_num = 0;
	char full_path[64] = { 0 };
	char img_name[32] = { 0 };
	struct file *filp = NULL;
	for (i = 0; i < (sizeof(type_str) / sizeof(char *)); i++) {
		snprintf(img_name, 32, fmt, md_id + 1, type_str[i]);
		/* Find at CIP first */
		snprintf(full_path, 64, "%s%s", CONFIG_MODEM_FIRMWARE_CIP_PATH, img_name);
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "Find:%s\n", full_path);
		filp = filp_open(full_path, O_RDONLY, 0644);
		if (IS_ERR(filp)) {
			/* Find at default */
			snprintf(full_path, 64, "%s%s", CONFIG_MODEM_FIRMWARE_PATH, img_name);
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "Find:%s\n", full_path);
			filp = filp_open(full_path, O_RDONLY, 0644);
			if (IS_ERR(filp)) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "%s not found(%d,%d)\n", full_path, img_num, i);
				continue;
			}
		}
		/* Run here means open image success */
		filp_close(filp, NULL);
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "Image:%s found\n", full_path);
		if (img_num < img_list_size)
			out_img_list[img_num] = i;
		img_num++;
	}
	return img_num;
}

static int check_md_header_v3(int md_id, void *parse_addr, struct ccci_image_info *image)
{
	int ret;
	bool md_type_check = false;
	bool md_plat_check = false;
	bool md_sys_match = false;
	bool md_size_check = false;
	int idx;
	unsigned int md_size;
	unsigned char *start, *ptr;
	int region_id, domain_id; /* add for v4 v5 */
	/* struct md_check_header_v3 *head = &md_img_header_v3[md_id]; */
	struct md_check_header_v4 *head = &md_img_header_v4[md_id];

	get_md_resv_mem_info(md_id, NULL, &md_size, NULL, NULL);
	/*memcpy(head, (void*)(parse_addr - sizeof(struct md_check_header_v3)), sizeof(struct md_check_header_v3)); */
	start = (unsigned char *)(head);
	ptr = (unsigned char *)(parse_addr - sizeof(struct md_check_header_v3));
	for (idx = 0; idx < sizeof(struct md_check_header_v3); idx++)
		*start++ = *ptr++;

	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "**********************MD image check V3 %d***************************\n",
				  (int)sizeof(struct md_check_header_v3));
	ret = strncmp(head->check_header, MD_HEADER_MAGIC_NO, 12);
	if (ret) {
		CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "md check header not exist!\n");
		ret = -CCCI_ERR_LOAD_IMG_CHECK_HEAD;
	} else {
		if (head->header_verno < 3) {
			CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "[Error]md check header version mis-match to AP:[%d]!\n",
						  head->header_verno);
		} else {
#ifdef ENABLE_2G_3G_CHECK
			if ((head->image_type != 0) && (head->image_type == md->config.load_type))
				md_type_check = true;
#else
			md_type_check = true;
#endif

#ifdef ENABLE_CHIP_VER_CHECK
			if (!strncmp(head->platform, ccci_get_ap_platform(), AP_PLATFORM_LEN))
				md_plat_check = true;
#else
			md_plat_check = true;
#endif

			if (head->bind_sys_id == (md_id + 1))
				md_sys_match = true;
#ifdef ENABLE_MEM_SIZE_CHECK
			if (head->header_verno >= 2) {
				/*md_size = md->mem_layout.md_region_size; */
				if (head->mem_size == md_size) {
					md_size_check = true;
				} else if (head->mem_size < md_size) {
					md_size_check = true;
					CCCI_UTIL_INF_MSG_WITH_ID(md_id,
								  "[Warning]md size in md header isn't sync to DFO setting: (%08x, %08x)\n",
								  head->mem_size, md_size);
				}
				image->img_info.mem_size = head->mem_size;
				image->ap_info.mem_size = md_size;
			} else {
				md_size_check = true;
			}
#else
			md_size_check = true;
#endif

			if ((md_id == MD_SYS1) && (head->image_type >= modem_ultg) && (head->image_type <= MAX_IMG_NUM))
				curr_ubin_id = head->image_type;
			image->ap_info.image_type = type_str[head->image_type];
			image->ap_info.platform = ccci_get_ap_platform();
			image->img_info.image_type = type_str[head->image_type];
			image->img_info.platform = head->platform;
			image->img_info.build_time = head->build_time;
			image->img_info.build_ver = head->build_ver;
			image->img_info.product_ver = product_str[head->product_ver];
			image->img_info.version = head->product_ver;
			image->img_info.header_verno = head->header_verno;

			if (md_type_check && md_plat_check && md_sys_match && md_size_check) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "Modem header check OK!\n");
			} else {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Error]Modem header check fail!\n");
				if (!md_type_check)
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD type(2G/3G) mis-match to AP!\n");

				if (!md_plat_check)
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD platform mis-match to AP!\n");

				if (!md_sys_match)
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD image is not for MD SYS%d!\n",
								  md_id + 1);

				if (!md_size_check)
					CCCI_UTIL_INF_MSG_WITH_ID(md_id,
								  "[Reason]MD mem size mis-match to AP setting!\n");

				ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
			}

			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[type]=%s, (AP)[type]=%s\n", image->img_info.image_type,
						  image->ap_info.image_type);
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[plat]=%s, (AP)[plat]=%s\n", image->img_info.platform,
						  image->ap_info.platform);
			if (head->header_verno >= 2) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[size]=%x, (AP)[size]=%x\n",
							  image->img_info.mem_size, image->ap_info.mem_size);
				if (head->md_img_size) {
					if (image->size >= head->md_img_size)
						image->size = head->md_img_size;
					else {
						char title[50];
						char info[100];

						snprintf(title, sizeof(title),
							 "MD%d mem size smaller than image header setting", md_id + 1);
						snprintf(info, sizeof(info),
							"MD%d mem size(0x%x)<header size(0x%x),please check memory config in <chip>.dtsi",
							md_id + 1, image->size, head->md_img_size);
#if defined(CONFIG_MTK_AEE_FEATURE)
						aed_md_exception_api(NULL, 0, (const int *)info, sizeof(info),
							(const char *)title, DB_OPT_DEFAULT);
#endif
						CCCI_UTIL_INF_MSG_WITH_ID(md_id,
									  "[Reason]MD image size mis-match to AP!\n");
						ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
					}
					image->ap_info.md_img_size = image->size;
					image->img_info.md_img_size = head->md_img_size;
				}
				/* else {image->size -= 0x1A0;} workaround for md not check in check header */
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[img_size]=%x, (AP)[img_size]=%x\n",
							  head->md_img_size, image->size);
			}
			if (head->header_verno >= 3) {
				image->dsp_offset = head->dsp_img_offset;
				image->dsp_size = head->dsp_img_size;
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "DSP image offset=%x size=%x\n", image->dsp_offset,
							  image->dsp_size);
				if (image->dsp_offset == 0xCDCDCDAA) {
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "DSP on EMI disabled\n");
				} else if (((image->dsp_offset&0xFFFF) != 0) && (head->header_verno == 3)) {
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "DSP image offset not 64KB align\n");
					ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
				} else if (image->dsp_offset + image->dsp_size > md_size) {
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "DSP image size too large %x\n", md_size);
					ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
				}
			} else {
				image->dsp_offset = 0;
			}

			if (head->header_verno >= 4) {  /* RMPU only avilable after check header v4 */
				for (region_id = 0; region_id <= MPU_REGION_INFO_ID_LAST; region_id++) {
					image->rmpu_info.region_info[region_id].region_offset =
						head->region_info[region_id].region_offset;
					image->rmpu_info.region_info[region_id].region_size =
						head->region_info[region_id].region_size;
					CCCI_UTIL_INF_MSG_WITH_ID(md_id,
						"[CCCI_UTIL] load_image: check_header_v4, region(%d): size = %x , offset = %x\n",
						region_id, head->region_info[region_id].region_size,
								head->region_info[region_id].region_offset);
				}

				for (domain_id = 0; domain_id < MPU_DOMAIN_INFO_ID_TOTAL_NUM; domain_id++) {
					image->rmpu_info.domain_attr[domain_id] = head->domain_attr[domain_id];
					CCCI_UTIL_INF_MSG_WITH_ID(md_id,
						"[CCCI_UTIL] load_image: check_header_v4, domain(%d): attr = %x\n",
						domain_id, head->domain_attr[domain_id]);
				}
			}
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[build_ver]=%s, [build_time]=%s\n",
						  image->img_info.build_ver, image->img_info.build_time);
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[product_ver]=%s\n", image->img_info.product_ver);
		}
	}
	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "**********************MD image check V3***************************\n");

	return ret;
}

static int md_check_header_parser(int md_id, void *parse_addr, struct ccci_image_info *image)
{
	int ret;
	bool md_type_check = false;
	bool md_plat_check = false;
	bool md_sys_match = false;
	bool md_size_check = false;
	unsigned int md_size;
	unsigned int header_size;
	int idx, header_up;
	unsigned char *start, *ptr;
	int region_id, domain_id; /* add for v4 v5 */

	struct md_check_header_struct *head = NULL;
	struct md_check_header *headv12 = NULL;
	struct md_check_header_v4 *headv34 = NULL;
	struct md_check_header_v5 *headv5 = NULL;

	get_md_resv_mem_info(md_id, NULL, &md_size, NULL, NULL);

	header_size = *(((unsigned int *)parse_addr) - 1);
	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "MD image header size = %d\n", header_size);

	if (header_size == sizeof(struct md_check_header_v3)) { /* v3, v4 */
		headv34 = &md_img_header_v4[md_id];
		headv12 = (struct md_check_header *)headv34;
		head = (struct md_check_header_struct *)headv34;
		header_up = 4;
	} else if (header_size == sizeof(struct md_check_header_v5)) {/* v5 */
		headv5 = &md_img_header_v5[md_id];
		headv34 = (struct md_check_header_v4 *)headv5;
		headv12 = (struct md_check_header *)headv5;
		head = (struct md_check_header_struct *)headv5;
		header_up = 5;
	} else { /* Default Load v1/2 */
		/* if (header_size == sizeof(struct md_check_header)) {*//* v1, v2 */
		headv12 = &md_img_header[md_id];
		head = (struct md_check_header_struct *)headv12;
		header_up = 2;
	}

	start = (unsigned char *)(head);
	ptr = (unsigned char *)(parse_addr - header_size);
	for (idx = 0; idx < header_size; idx++)
		*start++ = *ptr++;

	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "**********************MD image check v%d %d***************************\n",
				  (int)head->header_verno, (int)header_size);
	ret = strncmp(head->check_header, MD_HEADER_MAGIC_NO, 12);
	if (ret) {
		CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "md check header not exist!\n");
		ret = -CCCI_ERR_LOAD_IMG_CHECK_HEAD;
	} else if (head->header_verno > header_up) {
			CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "[Error]md check header version mis-match to AP:[%d]!\n",
						  head->header_verno);
		ret = -CCCI_ERR_LOAD_IMG_CHECK_HEAD;
	} else {
#ifdef ENABLE_2G_3G_CHECK
		if ((head->image_type != 0) && (head->image_type == md->config.load_type))
			md_type_check = true;
#else
		md_type_check = true;
#endif

#ifdef ENABLE_CHIP_VER_CHECK
		if (!strncmp(head->platform, ccci_get_ap_platform(), AP_PLATFORM_LEN))
			md_plat_check = true;
#else
		md_plat_check = true;
#endif

		if (head->bind_sys_id == (md_id + 1))
			md_sys_match = true;
#ifdef ENABLE_MEM_SIZE_CHECK
		if (head->header_verno >= 2) {
			/*md_size = md->mem_layout.md_region_size; */
			if (head->mem_size == md_size) {
				md_size_check = true;
			} else if (head->mem_size < md_size) {
				md_size_check = true;
				CCCI_UTIL_INF_MSG_WITH_ID(md_id,
					"[Warning]md size in md header isn't sync to DFO setting: (%08x, %08x)\n",
					head->mem_size, md_size);
			}
			image->img_info.mem_size = head->mem_size;
			image->ap_info.mem_size = md_size;
		} else {
			md_size_check = true;
		}
#else
		md_size_check = true;
#endif
		if ((md_id == MD_SYS1) && (head->image_type >= modem_ultg) && (head->image_type <= MAX_IMG_NUM))
			curr_ubin_id = head->image_type;
		image->ap_info.image_type = type_str[head->image_type];
		image->ap_info.platform = ccci_get_ap_platform();
		image->img_info.image_type = type_str[head->image_type];
		image->img_info.platform = head->platform;
		image->img_info.build_time = head->build_time;
		image->img_info.build_ver = head->build_ver;
		image->img_info.product_ver = product_str[head->product_ver];
		image->img_info.version = head->product_ver;
		image->img_info.header_verno = head->header_verno;

		if (md_type_check && md_plat_check && md_sys_match && md_size_check) {
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "Modem header check OK!\n");
		} else {
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Error]Modem header check fail!\n");
			if (!md_type_check)
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD type(2G/3G) mis-match to AP!\n");

			if (!md_plat_check)
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD platform mis-match to AP!\n");

			if (!md_sys_match)
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD image is not for MD SYS%d!\n",
							  md_id + 1);

			if (!md_size_check)
				CCCI_UTIL_INF_MSG_WITH_ID(md_id,
							  "[Reason]MD mem size mis-match to AP setting!\n");

			ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
		}

		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[type]=%s, (AP)[type]=%s\n", image->img_info.image_type,
					  image->ap_info.image_type);
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[plat]=%s, (AP)[plat]=%s\n", image->img_info.platform,
					  image->ap_info.platform);
		if (head->header_verno >= 2) {
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[size]=%x, (AP)[size]=%x\n",
						  image->img_info.mem_size, image->ap_info.mem_size);
			if (head->md_img_size) {
				if (image->size >= head->md_img_size)
					image->size = head->md_img_size;
				else {
					char title[50];
					char info[100];

					snprintf(title, sizeof(title),
						 "MD%d mem size smaller than image header setting", md_id + 1);
					snprintf(info, sizeof(info),
						 "MD%d mem cfg size(0x%x)<header size(0x%x),please check memory config in <chip>.dtsi",
						 md_id + 1, image->size, head->md_img_size);
#if defined(CONFIG_MTK_AEE_FEATURE)
					aed_md_exception_api(NULL, 0, (const int *)info, sizeof(info),
							     (const char *)title, DB_OPT_DEFAULT);
#endif
					CCCI_UTIL_INF_MSG_WITH_ID(md_id,
								  "[Reason]MD image size mis-match to AP!\n");
					ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
				}
				image->ap_info.md_img_size = image->size;
				image->img_info.md_img_size = head->md_img_size;
			}
			/* else {image->size -= 0x1A0;} workaround for md not check in check header */
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[img_size]=%x, (AP)[img_size]=%x\n",
						  head->md_img_size, image->size);
		}

		if (head->header_verno >= 3) { /* dsp offset && size */
			image->dsp_offset = headv34->dsp_img_offset;
			image->dsp_size = headv34->dsp_img_size;
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "DSP image offset=%x size=%x\n", image->dsp_offset,
					image->dsp_size);
			if (image->dsp_offset == 0xCDCDCDAA) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "DSP on EMI disabled\n");
			} else if (((image->dsp_offset&0xFFFF) != 0) && (head->header_verno == 3)) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "DSP image offset not 64KB align\n");
				ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
			} else if (image->dsp_offset + image->dsp_size > md_size) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "DSP image size too large %x\n", md_size);
				ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
			}
		} else {
			image->dsp_offset = 0;
		}

		if (head->header_verno >= 4) {  /* RMPU only avilable after check header v4 */
			for (region_id = 0; region_id <= MPU_REGION_INFO_ID_LAST; region_id++) {
				image->rmpu_info.region_info[region_id].region_offset =
					headv34->region_info[region_id].region_offset;
				image->rmpu_info.region_info[region_id].region_size =
					headv34->region_info[region_id].region_size;
				CCCI_UTIL_INF_MSG_WITH_ID(md_id,
					"[CCCI_UTIL] load_image: check_header_v4, region(%d): size = %x , offset = %x\n",
					region_id, headv34->region_info[region_id].region_size,
					headv34->region_info[region_id].region_offset);
			}
			for (domain_id = 0; domain_id < MPU_DOMAIN_INFO_ID_TOTAL_NUM; domain_id++) {
				image->rmpu_info.domain_attr[domain_id] = headv34->domain_attr[domain_id];
				CCCI_UTIL_INF_MSG_WITH_ID(md_id,
					"[CCCI_UTIL] load_image: check_header_v4, domain(%d): attr = %x\n", domain_id,
							headv34->domain_attr[domain_id]);
			}
		}

		if (head->header_verno >= 5) {  /* ARM7 only avilable after check header v5 */
			image->arm7_offset = headv5->arm7_img_offset;
			image->arm7_size = headv5->arm7_img_size;
			CCCI_UTIL_INF_MSG_WITH_ID(md_id,
				"[CCCI_UTIL] load_image: check_header_v5, arm7_offset = 0x%08X, arm_size = 0x%08X\n",
				image->arm7_offset, image->arm7_size);
		}

		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[build_ver]=%s, [build_time]=%s\n",
					  image->img_info.build_ver, image->img_info.build_time);
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[product_ver]=%s\n", image->img_info.product_ver);

	}
	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "**********************MD image check end***************************\n");

	return ret;
}

static int check_md_header(int md_id, void *parse_addr, struct ccci_image_info *image)
{
	int ret;
	bool md_type_check = false;
	bool md_plat_check = false;
	bool md_sys_match = false;
	bool md_size_check = false;
	unsigned int md_size;
	unsigned int header_size;
	int idx;
	unsigned char *start, *ptr;
	struct md_check_header *head = &md_img_header[md_id];
	get_md_resv_mem_info(md_id, NULL, &md_size, NULL, NULL);

	header_size = *(((unsigned int *)parse_addr) - 1);
	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "MD image header size = %d\n", header_size);
	if (header_size == sizeof(struct md_check_header_v3)) /* v3, v4 */
		return check_md_header_v3(md_id, parse_addr, image);
	else if (header_size == sizeof(struct md_check_header_v5))
		return md_check_header_parser(md_id, parse_addr, image);

	/*memcpy(head, (void*)(parse_addr - sizeof(struct md_check_header)), sizeof(struct md_check_header)); */
	start = (unsigned char *)(head);
	ptr = (unsigned char *)(parse_addr - sizeof(struct md_check_header));
	for (idx = 0; idx < sizeof(struct md_check_header); idx++)
		*start++ = *ptr++;

	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "**********************MD image check %d***************************\n",
				  (int)sizeof(struct md_check_header));
	ret = strncmp(head->check_header, MD_HEADER_MAGIC_NO, 12);
	if (ret) {
		CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "md check header not exist! %d\n", header_size);
		ret = -CCCI_ERR_LOAD_IMG_CHECK_HEAD;
	} else {
		if (head->header_verno > 2) {
			CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "[Error]md check header version mis-match to AP:[%d]!\n",
						  head->header_verno);
		} else {
#ifdef ENABLE_2G_3G_CHECK
			if ((head->image_type != 0) && (head->image_type == md->config.load_type))
				md_type_check = true;
#else
			md_type_check = true;
#endif

#ifdef ENABLE_CHIP_VER_CHECK
			if (!strncmp(head->platform, ccci_get_ap_platform(), AP_PLATFORM_LEN))
				md_plat_check = true;
#else
			md_plat_check = true;
#endif

			if (head->bind_sys_id == (md_id + 1))
				md_sys_match = true;
#ifdef ENABLE_MEM_SIZE_CHECK
			if (head->header_verno == 2) {
				/*md_size = md->mem_layout.md_region_size; */
				if (head->mem_size == md_size) {
					md_size_check = true;
				} else if (head->mem_size < md_size) {
					md_size_check = true;
					CCCI_UTIL_INF_MSG_WITH_ID(md_id,
								  "[Warning]md size in md header isn't sync to DFO setting: (%08x, %08x)\n",
								  head->mem_size, md_size);
				}
				image->img_info.mem_size = head->mem_size;
				image->ap_info.mem_size = md_size;
			} else {
				md_size_check = true;
			}
#else
			md_size_check = true;
#endif

			image->ap_info.image_type = type_str[head->image_type];
			image->ap_info.platform = ccci_get_ap_platform();
			image->img_info.image_type = type_str[head->image_type];
			image->img_info.platform = head->platform;
			image->img_info.build_time = head->build_time;
			image->img_info.build_ver = head->build_ver;
			image->img_info.product_ver = product_str[head->product_ver];
			image->img_info.version = head->product_ver;

			if (md_type_check && md_plat_check && md_sys_match && md_size_check) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "Modem header check OK!\n");
			} else {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Error]Modem header check fail!\n");
				if (!md_type_check)
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD type(2G/3G) mis-match to AP!\n");

				if (!md_plat_check)
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD platform mis-match to AP!\n");

				if (!md_sys_match)
					CCCI_UTIL_INF_MSG_WITH_ID(md_id, "[Reason]MD image is not for MD SYS%d!\n",
								  md_id + 1);

				if (!md_size_check)
					CCCI_UTIL_INF_MSG_WITH_ID(md_id,
								  "[Reason]MD mem size mis-match to AP setting!\n");

				ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
			}

			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[type]=%s, (AP)[type]=%s\n", image->img_info.image_type,
						  image->ap_info.image_type);
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[plat]=%s, (AP)[plat]=%s\n", image->img_info.platform,
						  image->ap_info.platform);
			if (head->header_verno >= 2) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[size]=%x, (AP)[size]=%x\n",
							  image->img_info.mem_size, image->ap_info.mem_size);
				if (head->md_img_size) {
					if (image->size >= head->md_img_size)
						image->size = head->md_img_size;
					else {
						CCCI_UTIL_INF_MSG_WITH_ID(md_id,
									  "[Reason]MD image size mis-match to AP!\n");
						ret = -CCCI_ERR_LOAD_IMG_MD_CHECK;
					}
					image->ap_info.md_img_size = image->size;
					image->img_info.md_img_size = head->md_img_size;
				}
				/* else {image->size -= 0x1A0;}  workaround for md not check in check header */
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[img_size]=%x, (AP)[img_size]=%x\n",
							  head->md_img_size, image->size);
			}
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[build_ver]=%s, [build_time]=%s\n",
						  image->img_info.build_ver, image->img_info.build_time);
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "(MD)[product_ver]=%s\n", image->img_info.product_ver);
		}
	}
	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "**********************MD image check***************************\n");

	return ret;
}

char *ccci_get_md_info_str(int md_id)
{
	return md_img_info_str[md_id];
}

#ifdef ENABLE_MD_IMG_SECURITY_FEATURE
static int sec_lib_version_check(void)
{
	int ret = 0;

	int sec_lib_ver = masp_ccci_version_info();
	if (sec_lib_ver != CURR_SEC_CCCI_SYNC_VER) {
		CCCI_UTIL_ERR_MSG("[Error]sec lib for ccci mismatch: sec_ver:%d, ccci_ver:%d\n", sec_lib_ver,
				  CURR_SEC_CCCI_SYNC_VER);
		ret = -1;
	}

	return ret;
}

/*--------------------------------------------------------------------------------------------------*/
/* New signature check version. 2012-2-2.  */
/* Change to use masp_ccci_signfmt_verify_file	*/
/*		(char *file_path, unsigned int *data_offset, unsigned int *data_sec_len) */
/*  masp_ccci_signfmt_verify_file parameter description */
/*    @ file_path: such as etc/firmware/modem.img */
/*    @ data_offset: the offset address that bypass signature header */
/*    @ data_sec_len: length of signature header + tail */
/*    @ return value: 0-success; */
/*---------------------------------------------------------------------------------------------------*/
static int signature_check_v2(int md_id, char *file_path, unsigned int *sec_tail_length)
{
	unsigned int bypass_sec_header_offset = 0;
	unsigned int sec_total_len = 0;

	if (masp_ccci_signfmt_verify_file(file_path, &bypass_sec_header_offset, &sec_total_len) == 0) {
		/*signature lib check success */
		/*-- check return value */
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "sign check ret value 0x%x, 0x%x!\n", bypass_sec_header_offset,
					  sec_total_len);
		if (bypass_sec_header_offset > sec_total_len) {
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "sign check fail(0x%x, 0x%x!)!\n", bypass_sec_header_offset,
						  sec_total_len);
			return -CCCI_ERR_LOAD_IMG_SIGN_FAIL;
		} else {
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "sign check success(0x%x, 0x%x)!\n", bypass_sec_header_offset,
						  sec_total_len);
			*sec_tail_length = sec_total_len - bypass_sec_header_offset;
			return (int)bypass_sec_header_offset;	/* Note here, offset is more than 2G is not hoped  */
		}
	} else {
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "sign check fail!\n");
		return -CCCI_ERR_LOAD_IMG_SIGN_FAIL;
	}
}

static int load_cipher_firmware_v2(int md_id,
				   int fp_id,
				   struct ccci_image_info *img,
				   unsigned int cipher_img_offset, unsigned int cipher_img_len)
{
	int ret;
	void *addr = ioremap_nocache(img->address, cipher_img_len);
	void *addr_bak = addr;
	unsigned int data_offset;

	if (addr == NULL) {
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "ioremap image fialed!\n");
		ret = -CCCI_ERR_LOAD_IMG_NO_ADDR;
		goto out;
	}

	if (SEC_OK !=
		masp_ccci_decrypt_cipherfmt(fp_id, cipher_img_offset, (char *)addr, cipher_img_len, &data_offset)) {
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "cipher image decrypt fail!\n");
		ret = -CCCI_ERR_LOAD_IMG_CIPHER_FAIL;
		goto unmap_out;
	}

	img->size = cipher_img_len;
	img->offset += data_offset;
	addr += cipher_img_len;

	ret = check_md_header(md_id, addr, img);

 unmap_out:
	iounmap(addr_bak);
 out:
	return ret;
}
#endif

static void get_md_postfix(int md_id, char k[], char buf[], char buf_ex[])
{
	/* name format: modem_X_YY_K_Ex.img */
	int X, Ex = 0;
	char YY_K[IMG_POSTFIX_LEN];
	unsigned int feature_val = 0;

	if ((md_id < 0) || (md_id > MAX_MD_NUM)) {
		CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "wrong MD ID to get postfix\n");
		return;
	}

	/* X */
	X = md_id + 1;

	if ((curr_ubin_id != 0) && (md_id == MD_SYS1)) {
		if (buf) {
			snprintf(buf, IMG_POSTFIX_LEN, "%d_%s_n", X, type_str[curr_ubin_id]);
			CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "MD%d image postfix=%s\n", md_id + 1, buf);
		}

		if (buf_ex) {
			snprintf(buf_ex, IMG_POSTFIX_LEN, "%d_%s_n_E%d", X, type_str[curr_ubin_id], Ex);
			CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "MD%d image postfix=%s\n", md_id + 1, buf_ex);
		}
		return;
	}
	/* YY_ */
	YY_K[0] = '\0';
	switch (md_id) {
	case MD_SYS1:
		feature_val = get_modem_support_cap(MD_SYS1);
		break;
	case MD_SYS2:
		feature_val = get_modem_support_cap(MD_SYS2);
		break;
	case MD_SYS3:
		feature_val = get_modem_support_cap(MD_SYS3);
		break;
	case MD_SYS5:
		feature_val = get_modem_support_cap(MD_SYS5);
		break;
	default:
		CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "request MD ID %d not supported\n", md_id);
		break;
	}

	if ((feature_val == 0) || (feature_val >= MAX_IMG_NUM)) {
		CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "request MD type %d not supported\n", feature_val);
		feature_val = md_type_invalid;
	}

	/* K */
	if (k == NULL)
		snprintf(YY_K, IMG_POSTFIX_LEN, "_%s_n", type_str[feature_val]);
	else
		snprintf(YY_K, IMG_POSTFIX_LEN, "_%s_%s", type_str[feature_val], k);

	/* [_Ex] Get chip version */
#if 0
	if (get_chip_version() == CHIP_SW_VER_01)
		Ex = 1;
	else if (get_chip_version() == CHIP_SW_VER_02)
		Ex = 2;
#else
	Ex = 1;
#endif

	/* Gen post fix */
	if (buf) {
		snprintf(buf, IMG_POSTFIX_LEN, "%d%s", X, YY_K);
		CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "MD%d image postfix=%s\n", md_id + 1, buf);
	}

	if (buf_ex) {
		snprintf(buf_ex, IMG_POSTFIX_LEN, "%d%s_E%d", X, YY_K, Ex);
		CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "MD%d image postfix=%s\n", md_id + 1, buf_ex);
	}
}

static struct file *open_img_file(char *name, int *sec_fp_id)
{
#ifdef ENABLE_MD_IMG_SECURITY_FEATURE
	int fp_id = OSAL_FILE_NULL;
	fp_id = osal_filp_open_read_only(name);
	CCCI_UTIL_DBG_MSG("sec_open fd = (%d)!\n", fp_id);

	if (sec_fp_id != NULL)
		*sec_fp_id = fp_id;

	CCCI_UTIL_DBG_MSG("sec_open file ptr = (%p)!\n", osal_get_filp_struct(fp_id));

	return (struct file *)osal_get_filp_struct(fp_id);
#else
	CCCI_UTIL_DBG_MSG("std_open!\n");
	return filp_open(name, O_RDONLY, 0644);
#endif
}

static void close_img_file(struct file *filp_id, int sec_fp_id)
{
#ifdef ENABLE_MD_IMG_SECURITY_FEATURE
	CCCI_UTIL_DBG_MSG("sec_close (%d)!\n", sec_fp_id);
	osal_filp_close(sec_fp_id);
#else
	CCCI_UTIL_DBG_MSG("std_close!\n");
	filp_close(filp_id, current->files);
#endif
}

static int find_img_to_open(int md_id, MD_IMG_TYPE img_type, char active_file_path[], char active_post_fix[])
{
	char img_name[3][IMG_NAME_LEN];
	char full_path[IMG_PATH_LEN];
	int i;
	char post_fix[IMG_POSTFIX_LEN];
	char post_fix_ex[IMG_POSTFIX_LEN];
	struct file *filp = NULL;

	/* Gen file name */
	get_md_postfix(md_id, NULL, post_fix, post_fix_ex);

	if (img_type == IMG_MD) {	/* Gen MD image name */
		snprintf(img_name[0], IMG_NAME_LEN, "modem_%s.img", post_fix);
		snprintf(img_name[1], IMG_NAME_LEN, "modem_%s.img", post_fix_ex);
		snprintf(img_name[2], IMG_NAME_LEN, "%s", MOEDM_IMAGE_NAME);
	} else if (img_type == IMG_DSP) {	/* Gen DSP image name */
		snprintf(img_name[0], IMG_NAME_LEN, "dsp_%s.bin", post_fix);
		snprintf(img_name[1], IMG_NAME_LEN, "dsp_%s.bin", post_fix_ex);
		snprintf(img_name[2], IMG_NAME_LEN, "%s", DSP_IMAGE_NAME);
	} else {
		CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "Invalid img type%d\n", img_type);
		return -CCCI_ERR_INVALID_PARAM;
	}

	CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "Find img @CIP\n");
	for (i = 0; i < 3; i++) {
		CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "try to open %s\n", img_name[i]);
		snprintf(full_path, IMG_PATH_LEN, "%s%s", CONFIG_MODEM_FIRMWARE_CIP_PATH, img_name[i]);
		filp = filp_open(full_path, O_RDONLY, 0644);
		if (IS_ERR(filp)) {
			continue;
		} else {	/* Open image success */
			snprintf(active_file_path, IMG_PATH_LEN, full_path);
			filp_close(filp, current->files);
			if (i == 0)
				snprintf(active_post_fix, IMG_POSTFIX_LEN, "%s", post_fix);
			else if (i == 1)
				snprintf(active_post_fix, IMG_POSTFIX_LEN, "%s", post_fix_ex);
			else
				active_post_fix[0] = '\0';
			return 0;
		}
	}

	CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "Find img @default\n");
	for (i = 0; i < 3; i++) {
		CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "try to open %s\n", img_name[i]);
		snprintf(full_path, IMG_PATH_LEN, "%s%s", CONFIG_MODEM_FIRMWARE_PATH, img_name[i]);
		filp = filp_open(full_path, O_RDONLY, 0644);
		if (IS_ERR(filp)) {
			continue;
		} else {	/* Open image success */
			snprintf(active_file_path, IMG_PATH_LEN, full_path);
			filp_close(filp, current->files);
			if (i == 0)
				snprintf(active_post_fix, IMG_POSTFIX_LEN, "%s", post_fix);
			else if (i == 1)
				snprintf(active_post_fix, IMG_POSTFIX_LEN, "%s", post_fix_ex);
			else
				active_post_fix[0] = '\0';
			return 0;
		}
	}

	active_post_fix[0] = '\0';
	CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "No Image file found\n");
	{
		char title[100];
		char info[100];
		snprintf(title, sizeof(title), "MD%d image not exist:%s", md_id + 1, img_name[0]);
		snprintf(info, sizeof(info), "Modem %d image name: %s,please check CUSTOM_MODME in projectconfig.mk!",
			 md_id + 1, img_name[0]);
#if defined(CONFIG_MTK_AEE_FEATURE)
		aed_md_exception_api(NULL, 0, (const int *)info, sizeof(info), (const char *)title, DB_OPT_DEFAULT);
#endif
	}
	return -CCCI_ERR_LOAD_IMG_NOT_FOUND;
}

static int load_std_firmware(int md_id, struct file *filp, struct ccci_image_info *img)
{
	void __iomem *start;
	int ret = 0;
	int check_ret = 0;
	int read_size = 0;
	mm_segment_t curr_fs;
	phys_addr_t load_addr;
	void *end_addr;
	const int size_per_read = 1024 * 1024;
	const int size = 1024;

	curr_fs = get_fs();
	set_fs(KERNEL_DS);

	load_addr = img->address;
	filp->f_pos = img->offset;

	while (1) {
		/* Map 1M memory */
		start = ioremap_nocache((load_addr + read_size), size_per_read);
		/*CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "map 0x%p --> 0x%p\n", (void*)(load_addr+read_size), start);*/
		if (start == NULL) {
			CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "image ioremap fail: %p\n", start);
			set_fs(curr_fs);
			return -CCCI_ERR_LOAD_IMG_NOMEM;
		}

		ret = (int)filp->f_op->read(filp, start, size_per_read, &filp->f_pos);
		/*make sure image size isn't 0 */
		if ((ret < 0) || (ret > size_per_read) || ((ret == 0) && (read_size == 0))) {
			CCCI_UTIL_ERR_MSG_WITH_ID(md_id, "image read fail: size=%d\n", ret);
			ret = -CCCI_ERR_LOAD_IMG_FILE_READ;
			goto error;
		} else if (ret == size_per_read) {
			read_size += ret;
			iounmap(start);
		} else {
			read_size += ret;
			/* Note here, signatured image has file tail info. */
			img->size = read_size - img->tail_length;
			CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "%s, image size=%d, read size:%d, tail:%d\n",
						  img->file_name, img->size, read_size, img->tail_length);
			iounmap(start);
			break;
		}
	}

	if (img->type == IMG_MD) {
		/* Make sure in one scope */
		start = ioremap_nocache(round_down(load_addr + img->size - 0x4000, 0x4000), 0x4000 * 2);
		end_addr = start + 0x4000 + (img->size - round_down(img->size, 0x4000));
		check_ret = check_md_header(md_id, end_addr, img);
		if (check_ret < 0) {
			ret = check_ret;
			goto error;
		}
		iounmap(start);
	} else if (img->type == IMG_DSP) {
		start = ioremap_nocache(load_addr, size);
		check_ret = check_dsp_header(md_id, start, img);
		if (check_ret < 0) {
			ret = check_ret;
			goto error;
		}
		iounmap(start);
	}

	set_fs(curr_fs);
	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "Load %s (size=0x%x) to 0x%pa\n", img->file_name, read_size, &load_addr);
	return read_size;

 error:
	iounmap(start);
	set_fs(curr_fs);
	return ret;
}

/*
 * this function should be universal to both MD image and DSP image
 */
static int load_image(int md_id, struct ccci_image_info *img_inf, char post_fix[])
{
	struct file *filp = NULL;
	int fp_id;
	int ret = 0;
	int offset = 0;
	unsigned int sec_tail_length = 0;
	struct ccci_image_info *img = NULL;
	MD_IMG_TYPE img_type;
#ifdef ENABLE_MD_IMG_SECURITY_FEATURE
	unsigned int img_len = 0;
#endif
	img = img_inf;
	img_type = img->type;
	if (find_img_to_open(md_id, img_type, img->file_name, post_fix) < 0) {
		ret = -CCCI_ERR_LOAD_IMG_FILE_OPEN;
		filp = NULL;
		goto out;
	}

	filp = open_img_file(img->file_name, &fp_id);
	if (IS_ERR(filp)) {
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "open %s fail: %ld\n", img->file_name, PTR_ERR(filp));
		ret = -CCCI_ERR_LOAD_IMG_FILE_OPEN;
		filp = NULL;
		goto out;
	} else {
		CCCI_UTIL_DBG_MSG_WITH_ID(md_id, "open %s OK\n", img->file_name);
	}

	/*Begin to check header, only modem.img need check signature and cipher header */
	sec_tail_length = 0;
	if (img_type == IMG_MD) {
		/*step1:check if need to signature */
#ifdef ENABLE_MD_IMG_SECURITY_FEATURE
		offset = signature_check_v2(md_id, img->file_name, &sec_tail_length);
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "signature_check offset:%d, tail:%d\n", offset, sec_tail_length);
		if (offset < 0) {
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "signature_check failed ret=%d\n", offset);
			ret = offset;
			goto out;
		}
#endif
		img->offset = offset;
		img->tail_length = sec_tail_length;

		/*step2:check if need to cipher */
#ifdef ENABLE_MD_IMG_SECURITY_FEATURE
		if (masp_ccci_is_cipherfmt(fp_id, offset, &img_len)) {
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "cipher image\n");
			ret = load_cipher_firmware_v2(md_id, fp_id, img, offset, img_len);
			if (ret < 0) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "load_cipher_firmware failed:ret=%d!\n", ret);
				goto out;
			}
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "load_cipher_firmware done! (=%d)\n", ret);
		} else {
#endif
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "Not cipher image\n");
			ret = load_std_firmware(md_id, filp, img);
			if (ret < 0) {
				CCCI_UTIL_INF_MSG_WITH_ID(md_id, "load_firmware failed: ret=%d!\n", ret);
				goto out;
			}
#ifdef ENABLE_MD_IMG_SECURITY_FEATURE
		}
#endif
	} else if (img_type == IMG_DSP) {
		/*dsp image check signature during uboot, and ccci not need check for dsp. */
		ret = load_std_firmware(md_id, filp, img);
		if (ret < 0) {
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "load_firmware for %s failed:ret=%d!\n", img->file_name, ret);
			goto out;
		}
	}

 out:
	if (filp != NULL)
		close_img_file(filp, fp_id);
	return ret;
}

int ccci_load_firmware(int md_id, void *img_inf, char img_err_str[], char post_fix[], struct device *dev)
{
	int ret = 0;
	struct ccci_image_info *img_ptr = (struct ccci_image_info *)img_inf;
	char *img_str;

	img_str = md_img_info_str[md_id];

	/*step1: clear modem protection when start to load firmware */
	/*ccci_clear_md_region_protection(md_id); */

	/*step2: load image */
	ret = load_image(md_id, img_ptr, post_fix);
	if (ret < 0) {
		/* if load_image failed, md->img_info won't have valid file name */
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "fail to load firmware!\n");
	}

	/* Construct image information string */
	if (img_ptr->type == IMG_MD) {
		sprintf(img_str, "MD:%s*%s*%s*%s*%s\nAP:%s*%s*%08x (MD)%08x\n",
			img_ptr->img_info.image_type, img_ptr->img_info.platform,
			img_ptr->img_info.build_ver, img_ptr->img_info.build_time,
			img_ptr->img_info.product_ver, img_ptr->ap_info.image_type,
			img_ptr->ap_info.platform, img_ptr->ap_info.mem_size, img_ptr->img_info.mem_size);
	}

	/* Prepare error string if needed */
	if (img_err_str != NULL) {
		if (ret == -CCCI_ERR_LOAD_IMG_SIGN_FAIL) {
			snprintf(img_err_str, IMG_ERR_STR_LEN, "%s Signature check fail\n", img_ptr->file_name);
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "signature check fail!\n");
		} else if (ret == -CCCI_ERR_LOAD_IMG_CIPHER_FAIL) {
			snprintf(img_err_str, IMG_ERR_STR_LEN, "%s Cipher chekc fail\n", img_ptr->file_name);
			CCCI_UTIL_INF_MSG_WITH_ID(md_id, "cipher check fail!\n");
		} else if (ret == -CCCI_ERR_LOAD_IMG_FILE_OPEN) {
			snprintf(img_err_str, IMG_ERR_STR_LEN, "Modem image not exist\n");
		} else if (ret == -CCCI_ERR_LOAD_IMG_MD_CHECK) {
			snprintf(img_err_str, IMG_ERR_STR_LEN, "Modem mismatch to AP\n");
		}
	}

	return ret;
}

#if 0
int get_img_info(int md_id, int img_type, struct ccci_image_info *info_ptr)
{
	if ((md_id >= MAX_MD_NUM) || (info_ptr == NULL) || (img_type >= IMG_NUM))
		return -1;

	info_ptr->type = img_info[md_id][img_type].type;
	memcpy(info_ptr->file_name, img_info[md_id][img_type].file_name, IMG_PATH_LEN);
	info_ptr->address = img_info[md_id][img_type].address;
	info_ptr->size = img_info[md_id][img_type].size;
	info_ptr->offset = img_info[md_id][img_type].offset;
	info_ptr->tail_length = img_info[md_id][img_type].tail_length;
	info_ptr->ap_platform = img_info[md_id][img_type].ap_platform;

	info_ptr->img_info.product_ver = img_info[md_id][img_type].img_info.product_ver;
	info_ptr->img_info.image_type = img_info[md_id][img_type].img_info.image_type;
	info_ptr->img_info.platform = img_info[md_id][img_type].img_info.platform;
	info_ptr->img_info.build_time = img_info[md_id][img_type].img_info.build_time;
	info_ptr->img_info.build_ver = img_info[md_id][img_type].img_info.build_ver;
	info_ptr->img_info.mem_size = img_info[md_id][img_type].img_info.mem_size;

	info_ptr->ap_info.product_ver = img_info[md_id][img_type].ap_info.product_ver;
	info_ptr->ap_info.image_type = img_info[md_id][img_type].ap_info.image_type;
	info_ptr->ap_info.platform = img_info[md_id][img_type].ap_info.platform;
	info_ptr->ap_info.build_time = img_info[md_id][img_type].ap_info.build_time;
	info_ptr->ap_info.build_ver = img_info[md_id][img_type].ap_info.build_ver;
	info_ptr->ap_info.mem_size = img_info[md_id][img_type].ap_info.mem_size;

	return 0;
}
#endif

int ccci_init_security(void)
{
	int ret = 0;
#ifdef ENABLE_MD_IMG_SECURITY_FEATURE
	static int security_init;	/* for multi-modem support */
	if (security_init)
		return ret;
	security_init = 1;

	ret = masp_boot_init();
	if (ret != 0) {
		CCCI_UTIL_ERR_MSG("masp_boot_init fail: %d\n", ret);
		ret = -EIO;
	}

	if (sec_lib_version_check() != 0) {
		CCCI_UTIL_ERR_MSG("sec lib version check error\n");
		ret = -EIO;
	}
	CCCI_UTIL_INF_MSG("security is on!\n");
#else
	CCCI_UTIL_INF_MSG("security is off!\n");
#endif
	return ret;
}

static const int md1_capability_array[] = {
	0, 0, 0, 0, 0, 0, 0, 0,
/* ultg */
	((1 << modem_ulwtg) | (1 << modem_ultg) | (0 << modem_ulwg) | (0 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ulwg */
	((1 << modem_ulwtg) | (0 << modem_ultg) | (1 << modem_ulwg) | (1 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ulwtg */
	((1 << modem_ulwtg) | (0 << modem_ultg) | (1 << modem_ulwg) | (1 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ulwcg */
	((0 << modem_ulwtg) | (0 << modem_ultg) | (0 << modem_ulwg) | (1 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ulwctg */
	((0 << modem_ulwtg) | (0 << modem_ultg) | (0 << modem_ulwg) | (0 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ulttg */
	((1 << modem_ulwtg) | (1 << modem_ultg) | (0 << modem_ulwg) | (0 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ulfwg */
	((1 << modem_ulwtg) | (0 << modem_ultg) | (1 << modem_ulwg) | (1 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ulfwcg */
	((0 << modem_ulwtg) | (0 << modem_ultg) | (0 << modem_ulwg) | (1 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ulctg */
	((0 << modem_ulwtg) | (0 << modem_ultg) | (0 << modem_ulwg) | (0 << modem_ulwcg) | (1 << modem_ulwctg)),
/* ultctg */
	((0 << modem_ulwtg) | (0 << modem_ultg) | (0 << modem_ulwg) | (0 << modem_ulwcg) | (1 << modem_ulwctg)),
};

static const int ap_md_wm_id_map_array[] = { 0x0,	/* 0-invalid */
	0x0,			/* 1-2g */
	0x0,			/* 2-3g */
	0x0,			/* 3-wg */
	0x0,			/* 4-tg */
	0x0,			/* 5-lwg */
	0x0,			/* 6-ltg */
	0x0,			/* 7-sglte */
	0x33,			/* 8-ultg */
	0x39,			/* 9-ulwg */
	0x3B,			/* 10-ulwtg */
	0x3D,			/* 11-ulwcg */
	0x3F,			/* 12-ulwctg */
	0x13,			/* 13-ulttg */
	0x29,			/* 14-ulfwg */
	0x2D,			/* 15-ulfwcg */
	0x37,			/* 16-ulctg */
	0x17,			/* 17-ultctg */
};

int get_md_wm_id_map(int ap_wm_id)
{
	if (ap_wm_id < (sizeof(ap_md_wm_id_map_array) / sizeof(int)))
		return ap_md_wm_id_map_array[ap_wm_id];
	return 0;
}

int md_capability(int md_id, int wm_id, int curr_md_type)
{
	if (curr_md_type >= MAX_IMG_NUM)
		return -1;
	if (wm_id >= MAX_IMG_NUM)
		return -2;

	if (curr_ubin_id == 0) {
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "curr_ubin_id is default val\n");
		return 1;
	}
	if (md_id == MD_SYS1) {
		if (md1_capability_array[wm_id] & (1 << curr_ubin_id))	/* Note here, curr_md_type not used */
			return 1;
		return 0;
	}
	return -3;
}

#define IMG_MAGIC		0x58881688
#define EXT_MAGIC		0x58891689

#define IMG_NAME_SIZE		32
#define IMG_HDR_SIZE		512
typedef union {
	struct {
		unsigned int magic;	/* always IMG_MAGIC */
		unsigned int dsize;	/* image size, image header and padding are not included */
		char name[IMG_NAME_SIZE];
		unsigned int maddr;	/* image load address in RAM */
		unsigned int mode;	/* maddr is counted from the beginning or end of RAM */
		/* extension */
		unsigned int ext_magic;	/* always EXT_MAGIC */
		unsigned int hdr_size;	/* header size is 512 bytes currently, but may extend in the future */
		unsigned int hdr_version; /* see HDR_VERSION */
		unsigned int img_type;	/* please refer to #define beginning with SEC_IMG_TYPE_ */
		unsigned int img_list_end; /* end of image list? 0: this image is followed by another image 1: end */
		unsigned int align_size; /* image size alignment setting in bytes, 16 by default for AES encryption */
		unsigned int dsize_extend; /* high word of image size for 64 bit address support */
		unsigned int maddr_extend; /* high word of image load address in RAM for 64 bit address support */
	} info;
	unsigned char data[IMG_HDR_SIZE];
} prt_img_hdr_t;

int ccci_get_md_check_hdr_inf(int md_id, void *img_inf, char post_fix[])
{
	int ret = 0;
	struct ccci_image_info *img_ptr = (struct ccci_image_info *)img_inf;
	char *img_str;
	char *buf;
	unsigned int md_type = 0;

	buf = kmalloc(1024, GFP_KERNEL);
	if (buf == NULL) {
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "fail to allocate memor for chk_hdr\n");
		return -1;
	}

	img_str = md_img_info_str[md_id];

	ret = get_raw_check_hdr(md_id, buf, 1024);
	if (ret < 0) {
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "fail to load header(%d)!\n", ret);
		kfree(buf);
		return -1;
	}

	img_ptr->size = get_md_img_raw_size(md_id);
	ret = check_md_header(md_id, buf+ret, img_ptr);
	if (ret < 0) {
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "check header fail(%d)!\n", ret);
		kfree(buf);
		return -1;
	}

	/* Get modem capability */
	md_type = get_md_type_from_lk(md_id);

	kfree(buf);

	/* Construct image information string */
	sprintf(img_str, "MD:%s*%s*%s*%s*%s\nAP:%s*%s*%08x (MD)%08x\n",
		img_ptr->img_info.image_type, img_ptr->img_info.platform,
		img_ptr->img_info.build_ver, img_ptr->img_info.build_time,
		img_ptr->img_info.product_ver, img_ptr->ap_info.image_type,
		img_ptr->ap_info.platform, img_ptr->ap_info.mem_size, img_ptr->img_info.mem_size);

	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "check header str[%s]!\n", img_str);

	if (md_id == MD_SYS1) {
		curr_ubin_id = md_type;
		CCCI_UTIL_INF_MSG_WITH_ID(md_id, "type @ header(%d)!\n", curr_ubin_id);
	}

	snprintf(post_fix, IMG_POSTFIX_LEN, "%d_%s_n", md_id+1, img_ptr->img_info.image_type);

	CCCI_UTIL_INF_MSG_WITH_ID(md_id, "post fix[%s]!\n", post_fix);

	return 0;
}

