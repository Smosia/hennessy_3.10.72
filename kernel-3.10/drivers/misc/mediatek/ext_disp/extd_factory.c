#include <linux/delay.h>
#include <asm/uaccess.h>

#include "extd_log.h"
#include "extd_factory.h"
#include "extd_info.h"
#include "external_display.h"


static HDMI_DRIVER *hdmi_tx_drv;
static int is_context_inited;
disp_ddp_path_config hdmi_factory_dpi_params;
DPI_PARAM_CONTEXT DPI_Params_Context;

typedef struct {
	bool hdmi_factory_inited;
	bool hdmi_callback_returned;
} hdmi_factory_context;

static hdmi_factory_context *_get_context(void)
{
	static hdmi_factory_context g_context;
	if (!is_context_inited) {
		memset((void *)&g_context, 0, sizeof(hdmi_factory_context));
		is_context_inited = 1;
		EXTD_FACTORY_LOG("[hdmi]_get_context set is_context_inited\n");
	}

	return &g_context;
}

#define pgc	_get_context()

#if defined(CONFIG_MTK_HDMI_SUPPORT)
static bool hdmi_factory_callback(HDMI_STATE state)
{
	EXTD_FACTORY_LOG("[hdmi]hdmi_factory_callback, state: %d\n", state);
	pgc->hdmi_callback_returned = state;
	return pgc->hdmi_callback_returned;
}

int hdmi_factory_mode_init(void)
{
	EXTD_FACTORY_FUNC();

	hdmi_tx_drv = (HDMI_DRIVER *) HDMI_GetDriver();
	if (NULL == hdmi_tx_drv) {
		EXTD_FACTORY_ERR("[hdmi]%s, hdmi_init fail, can not get hdmi driver handle\n", __func__);
		return -1;
	}
	hdmi_tx_drv->register_callback(hdmi_factory_callback);

	pgc->hdmi_factory_inited = true;
	return 0;
}

void hdmi_factory_dpi_parameters(int arg, int io_driving)
{
	HDMI_POLARITY clk_pol, de_pol, hsync_pol, vsync_pol;
	DPI_U32 dpi_clock = 0;
	DPI_U32 dpi_clk_div, dpi_clk_duty, hsync_pulse_width, hsync_back_porch, hsync_front_porch;
	DPI_U32 vsync_pulse_width, vsync_back_porch, vsync_front_porch, intermediat_buffer_num;

	switch (arg) {
	case HDMI_VIDEO_720x480p_60Hz:
		{
			clk_pol = HDMI_POLARITY_FALLING;
			de_pol = HDMI_POLARITY_RISING;
			hsync_pol = HDMI_POLARITY_RISING;
			vsync_pol = HDMI_POLARITY_RISING;

			dpi_clk_div = 2;

			hsync_pulse_width = 62;
			hsync_back_porch = 60;
			hsync_front_porch = 16;

			vsync_pulse_width = 6;
			vsync_back_porch = 30;
			vsync_front_porch = 9;

			DPI_Params_Context.bg_height = ((480 * DPI_Params_Context.scaling_factor) / 100 >> 2) << 2;
			DPI_Params_Context.bg_width = ((720 * DPI_Params_Context.scaling_factor) / 100 >> 2) << 2;
			DPI_Params_Context.hdmi_width = 720 - DPI_Params_Context.bg_width;
			DPI_Params_Context.hdmi_height = 480 - DPI_Params_Context.bg_height;
			DPI_Params_Context.output_video_resolution = HDMI_VIDEO_720x480p_60Hz;
			dpi_clock = 27027;
			break;
		}
	case HDMI_VIDEO_1280x720p_60Hz:
		{
			clk_pol = HDMI_POLARITY_FALLING;
			de_pol = HDMI_POLARITY_RISING;
			hsync_pol = HDMI_POLARITY_FALLING;
			vsync_pol = HDMI_POLARITY_FALLING;

			dpi_clk_div = 2;

			hsync_pulse_width = 40;
			hsync_back_porch = 220;
			hsync_front_porch = 110;

			vsync_pulse_width = 5;
			vsync_back_porch = 20;
			vsync_front_porch = 5;
			dpi_clock = 74250;

			DPI_Params_Context.bg_height = ((720 * DPI_Params_Context.scaling_factor) / 100 >> 2) << 2;
			DPI_Params_Context.bg_width = ((1280 * DPI_Params_Context.scaling_factor) / 100 >> 2) << 2;
			DPI_Params_Context.hdmi_width = 1280 - DPI_Params_Context.bg_width;
			DPI_Params_Context.hdmi_height = 720 - DPI_Params_Context.bg_height;

			DPI_Params_Context.output_video_resolution = HDMI_VIDEO_1280x720p_60Hz;
			break;
		}
	case HDMI_VIDEO_1920x1080p_30Hz:
		{
			clk_pol = HDMI_POLARITY_FALLING;
			de_pol = HDMI_POLARITY_RISING;
			hsync_pol = HDMI_POLARITY_FALLING;
			vsync_pol = HDMI_POLARITY_FALLING;

			dpi_clk_div = 2;

			hsync_pulse_width = 44;
			hsync_back_porch = 148;
			hsync_front_porch = 88;

			vsync_pulse_width = 5;
			vsync_back_porch = 36;
			vsync_front_porch = 4;

			DPI_Params_Context.bg_height = ((1080 * DPI_Params_Context.scaling_factor) / 100 >> 2) << 2;
			DPI_Params_Context.bg_width = ((1920 * DPI_Params_Context.scaling_factor) / 100 >> 2) << 2;
			DPI_Params_Context.hdmi_width = 1920 - DPI_Params_Context.bg_width;
			DPI_Params_Context.hdmi_height = 1080 - DPI_Params_Context.bg_height;

			DPI_Params_Context.output_video_resolution = HDMI_VIDEO_1920x1080p_30Hz;
			dpi_clock = 74250;
			break;
		}
	case HDMI_VIDEO_1920x1080p_60Hz:
		{
			clk_pol = HDMI_POLARITY_FALLING;
			de_pol = HDMI_POLARITY_RISING;
			hsync_pol = HDMI_POLARITY_FALLING;
			vsync_pol = HDMI_POLARITY_FALLING;

			dpi_clk_div = 2;

			hsync_pulse_width = 44;
			hsync_back_porch = 148;
			hsync_front_porch = 88;

			vsync_pulse_width = 5;
			vsync_back_porch = 36;
			vsync_front_porch = 4;

			DPI_Params_Context.bg_height = ((1080 * DPI_Params_Context.scaling_factor) / 100 >> 2) << 2;
			DPI_Params_Context.bg_width = ((1920 * DPI_Params_Context.scaling_factor) / 100 >> 2) << 2;
			DPI_Params_Context.hdmi_width = 1920 - DPI_Params_Context.bg_width;
			DPI_Params_Context.hdmi_height = 1080 - DPI_Params_Context.bg_height;

			DPI_Params_Context.output_video_resolution = HDMI_VIDEO_1920x1080p_60Hz;
			dpi_clock = 148500;
			break;
		}

	default:
		break;
	}

	hdmi_factory_dpi_params.dispif_config.dpi.width = DPI_Params_Context.hdmi_width;
	hdmi_factory_dpi_params.dispif_config.dpi.height = DPI_Params_Context.hdmi_height;
	hdmi_factory_dpi_params.dispif_config.dpi.bg_width = DPI_Params_Context.bg_width;
	hdmi_factory_dpi_params.dispif_config.dpi.bg_height = DPI_Params_Context.bg_height;

	hdmi_factory_dpi_params.dispif_config.dpi.clk_pol = clk_pol;
	hdmi_factory_dpi_params.dispif_config.dpi.de_pol = de_pol;
	hdmi_factory_dpi_params.dispif_config.dpi.vsync_pol = vsync_pol;
	hdmi_factory_dpi_params.dispif_config.dpi.hsync_pol = hsync_pol;

	hdmi_factory_dpi_params.dispif_config.dpi.hsync_pulse_width = hsync_pulse_width;
	hdmi_factory_dpi_params.dispif_config.dpi.hsync_back_porch = hsync_back_porch;
	hdmi_factory_dpi_params.dispif_config.dpi.hsync_front_porch = hsync_front_porch;
	hdmi_factory_dpi_params.dispif_config.dpi.vsync_pulse_width = vsync_pulse_width;
	hdmi_factory_dpi_params.dispif_config.dpi.vsync_back_porch = vsync_back_porch;
	hdmi_factory_dpi_params.dispif_config.dpi.vsync_front_porch = vsync_front_porch;

	hdmi_factory_dpi_params.dispif_config.dpi.format = 0;
	hdmi_factory_dpi_params.dispif_config.dpi.rgb_order = 0;
	hdmi_factory_dpi_params.dispif_config.dpi.i2x_en = true;
	hdmi_factory_dpi_params.dispif_config.dpi.i2x_edge = 2;
	hdmi_factory_dpi_params.dispif_config.dpi.embsync = false;
	hdmi_factory_dpi_params.dispif_config.dpi.io_driving_current = (LCM_DRIVING_CURRENT) io_driving;
	hdmi_factory_dpi_params.dispif_config.dpi.dpi_clock = dpi_clock;

	EXTD_FACTORY_LOG("[hdmi]hdmi_factory_dpi_parameters:%d\n", arg);
}

int hdmi_factory_mode_test(HDMI_FACTORY_TEST test_step, void *info)
{
	int ret = 0;
	if (pgc->hdmi_factory_inited == false)
		hdmi_factory_mode_init();

	switch (test_step) {
	case STEP1_CHIP_INIT:
		{
			EXTD_FACTORY_LOG("[hdmi] STEP1_CHIP_INIT\n");
			hdmi_tx_drv->power_on();
			hdmi_tx_drv->audio_enable(1);
			break;
		}
	case STEP2_JUDGE_CALLBACK:
		{
			EXTD_FACTORY_LOG("[hdmi] STEP2_JUDGE_CALLBACK: %d\n", pgc->hdmi_callback_returned);
			int hdmi_status = (int)pgc->hdmi_callback_returned;
			if (copy_to_user(info, &hdmi_status, sizeof(hdmi_status))) {
				EXTD_FACTORY_ERR("[HDMI]copy_to_user failed! line:%d\n", __LINE__);
				ret = -1;
			}
			break;
		}
	case STEP3_START_DPI_AND_CONFIG:
		{
			/*
			 *  test_type(24bit-31bit), resolution/
			 *  test_case(16bit-23bit), hsync/vsync/de/clk io drivint(8bit-15bit),
			 *  data io driving(0bit-7bit)
			 *  test_type (factory:0, HQA:1, DVT:2)
			 */
			int test_type = ((long int)info >> 24);
			int resolution = (((long int)info >> 16) & 0xFF);
			int test_case = resolution;
			int vsync_io_driving = (((long int)info >> 8) & 0xFF);
			int data_io_driving = ((long int)info & 0xFF);
			int io_driving = ((long int)info & 0xFFFF);

			EXTD_FACTORY_LOG("STEP3_START_DPI_AND_CONFIG +\n");
			EXTD_FACTORY_LOG("resolution/test case:%d, driving:0x%x\n", resolution, io_driving);

			if (test_type == 0) {	/* Factory mode */
				hdmi_factory_dpi_parameters(resolution, io_driving);
				ext_disp_factory_test(0, (void *)&hdmi_factory_dpi_params);

				msleep(100);
				hdmi_tx_drv->video_config(resolution, HDMI_VIN_FORMAT_RGB888, HDMI_VOUT_FORMAT_RGB888);
			} else if (test_type == 1) {	/* HQA */
				hdmi_factory_dpi_parameters(resolution, io_driving);
				ext_disp_factory_test(0, (void *)&hdmi_factory_dpi_params);

				EXTD_FACTORY_LOG("[hdmi] Not need video config for DPI HQA\n");
			} else if (test_type == 2) {	/* DVT */
#ifdef HDMI_DPI_DVT_TEST	/* Need to add dvt header file */
				EXTD_FACTORY_LOG("[hdmi] Start DPI DVT Test\n");
				hdmi_dvt_dpi_ioctl(test_case);
#endif
			}

			break;
		}
	case STEP4_DPI_STOP_AND_POWER_OFF:
		{
			EXTD_FACTORY_LOG("[hdmi] STEP4_DPI_STOP_AND_POWER_OFF\n");
			hdmi_tx_drv->power_off();

			pgc->hdmi_factory_inited = false;
			is_context_inited = false;

			hdmi_tx_drv->unregister_callback(hdmi_factory_callback);
			break;
		}
	default:
		break;
	}

	return ret;
}
#endif				/* CONFIG_MTK_HDMI_SUPPORT */

const EXTD_DRIVER *EXTD_Factory_HDMI_Driver(void)
{
	static const EXTD_DRIVER extd_factory_hdmi = {
#if defined(CONFIG_MTK_HDMI_SUPPORT)
		.init = hdmi_factory_mode_init,
		.deinit = NULL,
		.enable = NULL,
		.power_enable = NULL,
		.set_audio_enable = NULL,
		.set_resolution = NULL,
		.get_dev_info = NULL,
		.get_capability = NULL,
		.get_edid = NULL,
		.wait_vsync = NULL,
		.fake_connect = NULL,
		.factory_mode_test = hdmi_factory_mode_test,
		.ioctl = NULL
#else
		.init = 0
#endif
	};

	return &extd_factory_hdmi;
}


const EXTD_DRIVER *EXTD_Factory_EPD_Driver(void)
{
	static const EXTD_DRIVER extd_factory_epd = {
#if defined(CONFIG_MTK_EPD_SUPPORT)
		.init = NULL,
		.deinit = NULL,
		.enable = NULL,
		.power_enable = NULL,
		.set_audio_enable = NULL,
		.set_resolution = NULL,
		.get_dev_info = NULL,
		.get_capability = NULL,
		.get_edid = NULL,
		.wait_vsync = NULL,
		.fake_connect = NULL,
		.factory_mode_test = epd_factory_mode_test,
		.ioctl = NULL
#else
		.init = 0
#endif
	};

	return &extd_factory_epd;
}
