/*

SiI8348 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See 
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.             

*/

#if !defined(SI_8348_DRV_H)
#define SI_8348_DRV_H

#define LOW		0
#define	HIGH		1

#define	AUDIO_IF_SIZE	14	// 13 bytes Audio IF + 1 checksum


struct drv_hw_context {
	struct interrupt_info *intr_info;
	uint8_t 	chip_rev_id;
	uint16_t	chip_device_id;
	uint8_t		cbus_status;
	uint8_t		gen2_write_burst;
	uint8_t		ready_for_mdt;
	uint8_t		video_path;
	uint8_t		video_ready;
	uint8_t		audio_poll_enabled;
	uint8_t		current_edid_block_data [4*EDID_BLOCK_SIZE];
	uint8_t		current_edid_request_block;
	uint8_t		current_edid_request_block_batch;
	uint8_t		edid_fifo_block_number;
	uint8_t		saved_reg_mhltx_ctl2;
	//uint8_t		valid_audio_if;
	uint8_t		valid_vsif;
	//uint8_t		valid_avif;
	uint8_t		valid_3d;
#ifdef CONFIG_MTK_HDMI_3D_SUPPORT	
	uint8_t		valid_3d_fs;
#endif
	struct workqueue_struct 	*demo_loop_wq;
	struct delayed_work 		demo_loop_work;

	//int				current_audio_configure;
	//uint8_t				current_audio_info_frame[AUDIO_IF_SIZE];
	avi_info_frame_t		current_avi_info_frame;
	vendor_specific_info_frame_t	current_vs_info_frame;
	hw_avi_payload_t       		outgoingAviPayLoad;
	uint8_t				write_burst_data[MHL_SCRATCHPAD_SIZE];
};

void si_mhl_tx_set_pp_link(struct mhl_dev_context *dev_context, uint8_t value);


#endif /* if !defined(SI_8348_DRV_H) */
