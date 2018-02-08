#include "port_cfg.h"

#define TAG     "cfg"

#ifdef CONFIG_MTK_ENABLE_MD1
static struct ccci_port md1_ccci_ports[] = {
/* network port first for performace */
	{CCCI_CCMNI1_TX, CCCI_CCMNI1_RX, 3, 3, 0xF4, 0xFF, 8, &net_port_ops, 0xF1, "ccmni0",},
	{CCCI_CCMNI2_TX, CCCI_CCMNI2_RX, 3, 4, 0xF4, 0xFF, 8, &net_port_ops, 0xF2, "ccmni1",},
	{CCCI_CCMNI3_TX, CCCI_CCMNI3_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF3, "ccmni2",},
/*for IMS*/
	{CCCI_CCMNI4_TX, CCCI_CCMNI4_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF4, "ccmni3",},
	{CCCI_CCMNI5_TX, CCCI_CCMNI5_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF5, "ccmni4",},
	{CCCI_CCMNI6_TX, CCCI_CCMNI6_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF6, "ccmni5",},
	{CCCI_CCMNI7_TX, CCCI_CCMNI7_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF7, "ccmni6",},
	{CCCI_CCMNI8_TX, CCCI_CCMNI8_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF8, "ccmni7",},
/* char port, notes ccci_monitor must be first for md_cd_get_port_by_minor() implement */
	{CCCI_MONITOR_CH, CCCI_MONITOR_CH, 0xFF, 0xFF, 0xFF, 0xFF, 4, &char_port_ops, 0, "ccci_monitor",},
	{CCCI_PCM_TX, CCCI_PCM_RX, 0, 0, 0xFF, 0xFF, 4, &char_port_ops, 1, "ccci_aud",},
	{CCCI_UART1_TX, CCCI_UART1_RX, 1, 1, 3, 3, 0, &char_port_ops, 2, "ccci_md_log_ctrl",},
	{CCCI_UART2_TX, CCCI_UART2_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 3, "ttyC0",},
	{CCCI_FS_TX, CCCI_FS_RX, 1, 1, 1, 1, 4, &char_port_ops, 4, "ccci_fs",},
	{CCCI_IPC_UART_TX, CCCI_IPC_UART_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 5, "ttyC2",},
	{CCCI_ICUSB_TX, CCCI_ICUSB_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 6, "ttyC3",},
	{CCCI_MD_LOG_TX, CCCI_MD_LOG_RX, 2, 2, 2, 2, 8, &char_port_ops, 7, "ttyC1",},
	{CCCI_IMSV_UL, CCCI_IMSV_DL, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 8, "ccci_imsv",},
	{CCCI_IMSC_UL, CCCI_IMSC_DL, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 9, "ccci_imsc",},
	{CCCI_IMSA_UL, CCCI_IMSA_DL, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 10, "ccci_imsa",},
	{CCCI_IMSDC_UL, CCCI_IMSDC_DL, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 11, "ccci_imsdc",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 12, "ccci_ioctl0",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 13, "ccci_ioctl1",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 14, "ccci_ioctl2",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 15, "ccci_ioctl3",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 16, "ccci_ioctl4",},
	{CCCI_IT_TX, CCCI_IT_RX, 0, 0, 0xFF, 0xFF, 4, &char_port_ops, 17, "ccci_it",},
	{CCCI_LB_IT_TX, CCCI_LB_IT_RX, 0, 0, 0xFF, 0xFF, 0, &char_port_ops, 18, "ccci_lb_it",},
	{CCCI_MDL_MONITOR_UL, CCCI_MDL_MONITOR_DL, 1, 1, 3, 3, 0, &char_port_ops, 19, "ccci_mdl_monitor",},
	{CCCI_RPC_TX, CCCI_RPC_RX, 1, 1, 1, 1, 4, &char_port_ops, 20, "ccci_rpc",},
	{CCCI_RPC_TX, CCCI_RPC_RX, 1, 1, 1, 1, 0, &kernel_port_ops, 0, "ccci_rpc_k",},

/* IPC char port minor= minor idx + CCCI_IPC_MINOR_BASE(100) */
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 0, "ccci_ipc_1220_0",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 2, "ccci_ipc_2",},
/* IPC kernel port */
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &ipc_kern_port_ops, 3, "ccci_ipc_3",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 4, "ccci_ipc_4",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 5, "ccci_ipc_5",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &ipc_kern_port_ops, 6, "ccci_ipc_6",},
/* sys port */
	{CCCI_CONTROL_TX, CCCI_CONTROL_RX, 0, 0, 0, 0, 0, &kernel_port_ops, 0, "ccci_ctrl",},
	{CCCI_SYSTEM_TX, CCCI_SYSTEM_RX, 0, 0, 0xFF, 0xFF, 0, &kernel_port_ops, 0, "ccci_sys",},
	{CCCI_STATUS_TX, CCCI_STATUS_RX, 0, 0, 0, 0, 0, &kernel_port_ops, 0, "ccci_poll",},
};
#endif

#ifdef CONFIG_MTK_ENABLE_MD2
static struct ccci_port md2_ccci_ports[] = {
/* network port first for performace */
	{CCCI_CCMNI1_TX, CCCI_CCMNI1_RX, 3, 3, 0xF4, 0xFF, 8, &net_port_ops, 0xF1, "cc2mni0",},
	{CCCI_CCMNI2_TX, CCCI_CCMNI2_RX, 3, 4, 0xF4, 0xFF, 8, &net_port_ops, 0xF2, "cc2mni1",},
	{CCCI_CCMNI3_TX, CCCI_CCMNI3_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF3, "cc2mni2",},
/*for IMS*/
	{CCCI_CCMNI4_TX, CCCI_CCMNI4_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF4, "ccmni3",},
	{CCCI_CCMNI5_TX, CCCI_CCMNI5_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF5, "ccmni4",},
	{CCCI_CCMNI6_TX, CCCI_CCMNI6_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF6, "ccmni5",},
	{CCCI_CCMNI7_TX, CCCI_CCMNI7_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF7, "ccmni6",},
	{CCCI_CCMNI8_TX, CCCI_CCMNI8_RX, 5, 5, 0xFF, 0xFF, 8, &net_port_ops, 0xF8, "ccmni7",},
/* char port, notes ccci_monitor must be first */
	{CCCI_MONITOR_CH, CCCI_MONITOR_CH, 0xFF, 0xFF, 0xFF, 0xFF, 4, &char_port_ops, 0, "ccci2_monitor",},
	{CCCI_PCM_TX, CCCI_PCM_RX, 0, 0, 0xFF, 0xFF, 4, &char_port_ops, 1, "ccci2_aud",},
	{CCCI_UART1_TX, CCCI_UART1_RX, 1, 1, 3, 3, 0, &char_port_ops, 2, "ccci2_md_log_ctrl",},
	{CCCI_UART2_TX, CCCI_UART2_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 3, "ccci2_tty0",},
	{CCCI_FS_TX, CCCI_FS_RX, 1, 1, 1, 1, 4, &char_port_ops, 4, "ccci2_fs",},
	{CCCI_IPC_UART_TX, CCCI_IPC_UART_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 5, "ccci2_tty2",},
	{CCCI_ICUSB_TX, CCCI_ICUSB_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 6, "ccci2_tty3",},
	{CCCI_MD_LOG_TX, CCCI_MD_LOG_RX, 2, 2, 2, 2, 8, &char_port_ops, 7, "ccci2_tty1",},
	{CCCI_IMSV_UL, CCCI_IMSV_DL, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 8, "ccci2_imsv",},
	{CCCI_IMSC_UL, CCCI_IMSC_DL, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 9, "ccci2_imsc",},
	{CCCI_IMSA_UL, CCCI_IMSA_DL, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 10, "ccci2_imsa",},
	{CCCI_IMSDC_UL, CCCI_IMSDC_DL, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 11, "ccci2_imsdc",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 12, "ccci2_ioctl0",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 13, "ccci2_ioctl1",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 14, "ccci2_ioctl2",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 15, "ccci2_ioctl3",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 16, "ccci2_ioctl4",},
	{CCCI_IT_TX, CCCI_IT_RX, 0, 0, 0xFF, 0xFF, 4, &char_port_ops, 17, "ccci2_it",},
	{CCCI_LB_IT_TX, CCCI_LB_IT_RX, 0, 0, 0xFF, 0xFF, 0, &char_port_ops, 18, "ccci2_lb_it",},
	{CCCI_RPC_TX, CCCI_RPC_RX, 1, 1, 1, 1, 4, &char_port_ops, 19, "ccci2_rpc",},
	{CCCI_RPC_TX, CCCI_RPC_RX, 1, 1, 1, 1, 0, &kernel_port_ops, 0, "ccci2_rpc_k",},

/* IPC char port minor= minor idx + CCCI_IPC_MINOR_BASE(100) */
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 0, "ccci2_ipc_0",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 2, "ccci2_ipc_2",},
/* IPC kernel port */
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &ipc_kern_port_ops, 3, "ccci2_ipc_3",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 4, "ccci2_ipc_4",},
/* sys port */
	{CCCI_CONTROL_TX, CCCI_CONTROL_RX, 0, 0, 0, 0, 0, &kernel_port_ops, 0, "ccci2_ctrl",},
	{CCCI_SYSTEM_TX, CCCI_SYSTEM_RX, 0, 0, 0xFF, 0xFF, 0, &kernel_port_ops, 0, "ccci2_sys",},
	{CCCI_STATUS_TX, CCCI_STATUS_RX, 0, 0, 0, 0, 0, &kernel_port_ops, 0, "ccci_poll",},
};

#endif

#ifdef CONFIG_MTK_ECCCI_C2K
static struct ccci_port md3_ccci_ports[] = {
/* network port first for performace */
	{CCCI_CCMNI1_TX, CCCI_CCMNI1_RX, 2, 2, 0xF2, 0xFF, 8, &net_port_ops, 0xF1, "cc3mni0",},
	{CCCI_CCMNI2_TX, CCCI_CCMNI2_RX, 2, 2, 0xF2, 0xFF, 8, &net_port_ops, 0xF2, "cc3mni1",},
	{CCCI_CCMNI3_TX, CCCI_CCMNI3_RX, 2, 2, 0xF2, 0xFF, 8, &net_port_ops, 0xF3, "cc3mni2",},
/* char port, notes ccci_monitor must be first */
	{CCCI_MONITOR_CH, CCCI_MONITOR_CH, 0xFF, 0xFF, 0xFF, 0xFF, 4, &char_port_ops, 0, "ccci3_monitor",},
	{CCCI_PCM_TX, CCCI_PCM_RX, 1, 1, 0xFF, 0xFF, 0, &char_port_ops, 1, "ccci3_aud",},
	{CCCI_UART1_TX, CCCI_UART1_RX, 0, 0, 0, 0, 0, &char_port_ops, 2, "ccci3_md_log_ctrl",},
	{CCCI_FS_TX, CCCI_FS_RX, 4, 4, 4, 4, 4, &char_port_ops, 4, "ccci3_fs",},
	{CCCI_IPC_UART_TX, CCCI_IPC_UART_RX, 6, 6, 0xFF, 0xFF, 0, &char_port_ops, 5, "ccci3_tty2",},	/*for agps */
	{CCCI_MD_LOG_TX, CCCI_MD_LOG_RX, C2K_MD_LOG_TX_Q, C2K_MD_LOG_RX_Q, C2K_MD_LOG_TX_Q, C2K_MD_LOG_RX_Q,
		0, &char_port_ops, 7, "ccci3_tty1",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 12, "ccci3_ioctl0",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 13, "ccci3_ioctl1",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 14, "ccci3_ioctl2",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 15, "ccci3_ioctl3",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF, 0, &char_port_ops, 16, "ccci3_ioctl4",},
	{CCCI_IT_TX, CCCI_IT_RX, 0, 0, 0xFF, 0xFF, 4, &char_port_ops, 17, "ccci3_it",},
	{CCCI_LB_IT_TX, CCCI_LB_IT_RX, 0, 0, 0xFF, 0xFF, 0, &char_port_ops, 18, "ccci3_lb_it",},
/* c2k only port */
{CCCI_C2K_PPP_DATA,	CCCI_C2K_PPP_DATA, 2, 2, 0xFF, 0xFF, 0, &char_port_ops, 19, "ccci3_data",},
	{CCCI_C2K_AT, CCCI_C2K_AT, 5, 5, 0xF4, 0xFF, 0, &char_port_ops, 20, "ccci3_at",},
	{CCCI_C2K_AT2, CCCI_C2K_AT2, 5, 5, 0xFF, 0xFF, 0, &char_port_ops, 21, "ccci3_at2",},
	{CCCI_C2K_AT3, CCCI_C2K_AT3, 5, 5, 0xFF, 0xFF, 0, &char_port_ops, 22, "ccci3_at3",},
	{CCCI_C2K_LB_DL, CCCI_C2K_LB_DL, 5, 5, 0xFF, 0xFF, 0, &char_port_ops, 23, "ccci3_lb_dl",},
/* sys port */
	{CCCI_CONTROL_TX, CCCI_CONTROL_RX, 0, 0, 0, 0, 0, &kernel_port_ops, 0, "ccci3_ctrl",},
	{CCCI_STATUS_TX, CCCI_STATUS_RX, 0, 0, 0, 0, 0,	&kernel_port_ops, 0, "ccci3_poll",},
};

#endif

int md_port_cfg(struct ccci_modem *md)
{
	switch (md->index) {
#ifdef CONFIG_MTK_ENABLE_MD1
	case MD_SYS1:
		md->ports = md1_ccci_ports;
		md->port_number = ARRAY_SIZE(md1_ccci_ports);
		break;
#endif
#ifdef CONFIG_MTK_ENABLE_MD2
	case MD_SYS2:
		md->ports = md2_ccci_ports;
		md->port_number = ARRAY_SIZE(md2_ccci_ports);
		break;
#endif
#ifdef CONFIG_MTK_ECCCI_C2K
	case MD_SYS3:
		md->ports = md3_ccci_ports;
		md->port_number = ARRAY_SIZE(md3_ccci_ports);
		break;
#endif
	default:
		md->ports = NULL;
		md->port_number = 0;
		CCCI_ERR_MSG(md->index, TAG, "md_port_cfg:no md enable\n");
		return -1;
	}
	return 0;
}
