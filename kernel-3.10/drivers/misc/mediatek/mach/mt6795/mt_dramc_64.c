#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kallsyms.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/vmalloc.h>
#include <linux/memblock.h>
#include <linux/sched.h> 
#include <linux/delay.h>
#include <asm/cacheflush.h>
//#include <asm/outercache.h>
//#include <asm/system.h>
//#include <asm/delay.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_clkmgr.h>
#include "mt_freqhopping.h"
#include <mach/emi_bwl.h>
#include <mach/mt_typedefs.h>
#include <mach/memory.h>
#include <mach/mt_sleep.h>
#include <mach/mt_dramc.h>
#include <mach/dma.h>
#include "mt_freqhopping_drv.h"
#include <mach/sync_write.h>
#include <mach/mt_chip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <mach/mt_ccci_common.h>
#include <mach/mt_vcore_dvfs.h>

//#include "md32_ipi.h"
//#include "md32_helper.h"

#ifdef CONFIG_OF_RESERVED_MEM
#define DRAM_R0_DUMMY_READ_RESERVED_KEY "reserve-memory-dram_r0_dummy_read"
#define DRAM_R1_DUMMY_READ_RESERVED_KEY "reserve-memory-dram_r1_dummy_read"
#include <mach/mtk_memcfg.h>
#include <linux/of_reserved_mem.h>
#endif

static void __iomem *APMIXED_BASE_ADDR;
static void __iomem *CQDMA_BASE_ADDR;
static void __iomem *DRAMCAO_BASE_ADDR;
static void __iomem *DDRPHY_BASE_ADDR;
static void __iomem *DRAMCNAO_BASE_ADDR;
static void __iomem *SLEEP_BASE_ADDR;

volatile unsigned char *dst_array_v;
volatile unsigned char *src_array_v;
volatile unsigned int dst_array_p;
volatile unsigned int src_array_p;
char dfs_dummy_buffer[BUFF_LEN] __attribute__ ((aligned (PAGE_SIZE)));
int init_done = 0;
int org_dram_data_rate = 0;
static unsigned int dram_rank_num;
phys_addr_t dram_base, dram_add_rank0_base;

void enter_pasr_dpd_config(unsigned char segment_rank0, unsigned char segment_rank1)
{
    if(segment_rank1 == 0xFF) //all segments of rank1 are not reserved -> rank1 enter DPD
    {
        slp_dpd_en(1);
	segment_rank1 = 0x0;
    }
    
    slp_pasr_en(1, segment_rank0 | (segment_rank1 << 8) | (((unsigned short)org_dram_data_rate) << 16)); 
}

void exit_pasr_dpd_config(void)
{
    slp_dpd_en(0);
    slp_pasr_en(0, 0);
}

#define FREQREG_SIZE 21 
#define PLLGRPREG_SIZE	20
vcore_dvfs_info_t __nosavedata g_vcore_dvfs_info;
unsigned int __nosavedata vcore_dvfs_data[((FREQREG_SIZE*4) + (PLLGRPREG_SIZE*2) + 2)];

void store_vcore_dvfs_setting(void)
{
    unsigned int *atag_ptr, *buf;
    unsigned int pll_setting_num, freq_setting_num;
    
#ifdef CONFIG_OF
    if (of_chosen) 
    {
        atag_ptr = (unsigned int*)of_get_property(of_chosen, "atag,vcore_dvfs", NULL);      
        if(atag_ptr)
        {
            atag_ptr += 2; //skip tag header
            
            memcpy((void*)vcore_dvfs_data, (void*)atag_ptr, (FREQREG_SIZE*4 + PLLGRPREG_SIZE*2 + 2)*sizeof(unsigned int));
            buf = (unsigned int *)&vcore_dvfs_data[0];
            pll_setting_num = g_vcore_dvfs_info.pll_setting_num = *buf++;
            freq_setting_num = g_vcore_dvfs_info.freq_setting_num = *buf++;

            printk("[vcore dvfs][kernel]pll_setting_num = %d\n", g_vcore_dvfs_info.pll_setting_num);
            printk("[vcore dvfs][kernel]freq_setting_num = %d\n", g_vcore_dvfs_info.freq_setting_num);             
            ASSERT((pll_setting_num == PLLGRPREG_SIZE) && (freq_setting_num == FREQREG_SIZE));

            g_vcore_dvfs_info.low_freq_pll_setting_addr = (unsigned long)buf;
            buf += pll_setting_num;
            g_vcore_dvfs_info.low_freq_cha_setting_addr = (unsigned long)buf;
            buf += freq_setting_num;
            g_vcore_dvfs_info.low_freq_chb_setting_addr = (unsigned long)buf;
            buf += freq_setting_num;
            g_vcore_dvfs_info.high_freq_pll_setting_addr = (unsigned long)buf;
            buf += pll_setting_num;
            g_vcore_dvfs_info.high_freq_cha_setting_addr = (unsigned long)buf;
            buf += freq_setting_num;
            g_vcore_dvfs_info.high_freq_chb_setting_addr = (unsigned long)buf;    
            printk("[vcore dvfs][kernel]low_freq_pll_setting_addr = 0x%lx, value[0] = 0x%x\n", g_vcore_dvfs_info.low_freq_pll_setting_addr, *(unsigned int*)(g_vcore_dvfs_info.low_freq_pll_setting_addr));
            printk("[vcore dvfs][kernel]low_freq_cha_setting_addr = 0x%lx, value[0] = 0x%x\n", g_vcore_dvfs_info.low_freq_cha_setting_addr, *(unsigned int*)(g_vcore_dvfs_info.low_freq_cha_setting_addr));
            printk("[vcore dvfs][kernel]low_freq_chb_setting_addr = 0x%lx, value[0] = 0x%x\n", g_vcore_dvfs_info.low_freq_chb_setting_addr, *(unsigned int*)(g_vcore_dvfs_info.low_freq_chb_setting_addr));
            printk("[vcore dvfs][kernel]high_freq_pll_setting_addr = 0x%lx, value[0] = 0x%x\n", g_vcore_dvfs_info.high_freq_pll_setting_addr, *(unsigned int*)(g_vcore_dvfs_info.high_freq_pll_setting_addr));
            printk("[vcore dvfs][kernel]high_freq_cha_setting_addr = 0x%lx, value[0] = 0x%x\n", g_vcore_dvfs_info.high_freq_cha_setting_addr, *(unsigned int*)(g_vcore_dvfs_info.high_freq_cha_setting_addr));
            printk("[vcore dvfs][kernel]high_freq_chb_setting_addr = 0x%lx, value[0] = 0x%x\n", g_vcore_dvfs_info.high_freq_chb_setting_addr, *(unsigned int*)(g_vcore_dvfs_info.high_freq_chb_setting_addr));             

#ifdef DUMP_VCORE_TABLE
            {
              int i;
              for(i=0; i<freq_setting_num; i++)
                printk("[vcore dvfs][kernel]low_freq_cha_setting_addr = 0x%lx, value = 0x%x\n", g_vcore_dvfs_info.low_freq_cha_setting_addr+i*4, *(unsigned int*)(g_vcore_dvfs_info.low_freq_cha_setting_addr+i*4));
                 
              for(i=0; i<freq_setting_num; i++)
                printk("[vcore dvfs][kernel]low_freq_chb_setting_addr = 0x%lx, value = 0x%x\n", g_vcore_dvfs_info.low_freq_chb_setting_addr+i*4, *(unsigned int*)(g_vcore_dvfs_info.low_freq_chb_setting_addr+i*4));
              
              for(i=0; i<freq_setting_num; i++)
                printk("[vcore dvfs][kernel]high_freq_cha_setting_addr = 0x%lx, value = 0x%x\n", g_vcore_dvfs_info.high_freq_cha_setting_addr+i*4, *(unsigned int*)(g_vcore_dvfs_info.high_freq_cha_setting_addr+i*4));
              
              for(i=0; i<freq_setting_num; i++)
                printk("[vcore dvfs][kernel]high_freq_chb_setting_addr = 0x%lx, value = 0x%x\n", g_vcore_dvfs_info.high_freq_chb_setting_addr+i*4, *(unsigned int*)(g_vcore_dvfs_info.high_freq_chb_setting_addr+i*4));            
            }
#endif
        }
        else
            printk("[%s] No atag,vcore_dvfs!\n", __func__);
    }
    else
        printk("[%s] of_chosen is NULL!\n", __func__);
#endif   
    {
        unsigned long high_cha, high_chb, low_cha, low_chb;
        unsigned int num;
        get_freq_table_info(&high_cha, &high_chb, &low_cha, &low_chb, &num);
        printk("[vcore dvfs]high_cha_addr = 0x%lx, high_chb_addr = 0x%lx, low_cha_addr = 0x%lx, low_chb_addr = 0x%lx, num = %d\n", high_cha, high_chb, low_cha, low_chb, num);
    }     
}

#if 0
void vcore_dvfs_ipi_handler(int id, void *data, unsigned int len)
{
    int i;
    unsigned long src_ptr, dst_ptr;
    int pll_setting_num = g_vcore_dvfs_info.pll_setting_num;
    int freq_setting_num = g_vcore_dvfs_info.freq_setting_num;
    
    vcore_dvfs_info_t *md32_dvfs_info = (vcore_dvfs_info_t*)data;
    *(unsigned int*)(MD32_DTCM + (unsigned long)&(md32_dvfs_info->pll_setting_num)) = pll_setting_num;
    *(unsigned int*)(MD32_DTCM + (unsigned long)&(md32_dvfs_info->freq_setting_num)) = freq_setting_num;
    
    printk("[vcore dvfs]md32 ipi handler is called\n");
   
    src_ptr = g_vcore_dvfs_info.low_freq_pll_setting_addr;
    dst_ptr = MD32_DTCM + md32_dvfs_info->low_freq_pll_setting_addr;
    for(i=0; i<pll_setting_num; i++)
    {
        *(u32*)dst_ptr = *(u32*)src_ptr;
        //printk("[vcore dvfs][md32]pll_low: dst_ptr = 0x%x, src_ptr=0x%x, value=0x%x\n", dst_ptr, src_ptr, *(u32*)dst_ptr);
        dst_ptr+=4;
        src_ptr+=4;
    }      
    
    src_ptr = g_vcore_dvfs_info.low_freq_cha_setting_addr;
    dst_ptr = MD32_DTCM + md32_dvfs_info->low_freq_cha_setting_addr;
    for(i=0; i<freq_setting_num; i++)
    {
        *(u32*)dst_ptr = *(u32*)src_ptr;
        //printk("[vcore dvfs][md32]cha_low: dst_ptr = 0x%x, src_ptr=0x%x, value=0x%x\n", dst_ptr, src_ptr, *(u32*)dst_ptr);
        dst_ptr+=4;
        src_ptr+=4;
    } 
        
    src_ptr = g_vcore_dvfs_info.low_freq_chb_setting_addr;
    dst_ptr = MD32_DTCM + md32_dvfs_info->low_freq_chb_setting_addr;
    for(i=0; i<freq_setting_num; i++)
    {
        *(u32*)dst_ptr = *(u32*)src_ptr;
        //printk("[vcore dvfs][md32]chb_low: dst_ptr = 0x%x, src_ptr=0x%x, value=0x%x\n", dst_ptr, src_ptr, *(u32*)dst_ptr);
        dst_ptr+=4;
        src_ptr+=4;
    } 
        
    src_ptr = g_vcore_dvfs_info.high_freq_pll_setting_addr;
    dst_ptr = MD32_DTCM + md32_dvfs_info->high_freq_pll_setting_addr;
    for(i=0; i<pll_setting_num; i++)
    {
        *(u32*)dst_ptr = *(u32*)src_ptr;
        //printk("[vcore dvfs][md32]pll_high: dst_ptr = 0x%x, src_ptr=0x%x, value=0x%x\n", dst_ptr, src_ptr, *(u32*)dst_ptr);
        dst_ptr+=4;
        src_ptr+=4;
    }  
    
    src_ptr = g_vcore_dvfs_info.high_freq_cha_setting_addr;
    dst_ptr = MD32_DTCM + md32_dvfs_info->high_freq_cha_setting_addr;
    for(i=0; i<freq_setting_num; i++)
    {
        *(u32*)dst_ptr = *(u32*)src_ptr;
        //printk("[vcore dvfs][md32]cha_high: dst_ptr = 0x%x, src_ptr=0x%x, value=0x%x\n", dst_ptr, src_ptr, *(u32*)dst_ptr);
        dst_ptr+=4;
        src_ptr+=4;        
    } 
        
    src_ptr = g_vcore_dvfs_info.high_freq_chb_setting_addr;
    dst_ptr = MD32_DTCM + md32_dvfs_info->high_freq_chb_setting_addr;
    for(i=0; i<freq_setting_num; i++)
    {
        *(u32*)dst_ptr = *(u32*)src_ptr;
        //printk("[vcore dvfs][md32]chb_high: dst_ptr = 0x%x, src_ptr=0x%x, value=0x%x\n", dst_ptr, src_ptr, *(u32*)dst_ptr);
        dst_ptr+=4;
        src_ptr+=4;        
    }   
}
#endif

void get_freq_table_info(unsigned long *high_cha_addr, unsigned long *high_chb_addr, unsigned long *low_cha_addr, unsigned long *low_chb_addr, unsigned int *num)
{
    unsigned int freq_setting_num;
    freq_setting_num = g_vcore_dvfs_info.freq_setting_num;
    
    if(freq_setting_num != 0)
    {    
        *high_cha_addr = g_vcore_dvfs_info.high_freq_cha_setting_addr;     
        *high_chb_addr = g_vcore_dvfs_info.high_freq_chb_setting_addr;
        *low_cha_addr = g_vcore_dvfs_info.low_freq_cha_setting_addr;     
        *low_chb_addr = g_vcore_dvfs_info.low_freq_chb_setting_addr;        
        *num = freq_setting_num;
    }
    else
    {
        *high_cha_addr = *high_chb_addr = *low_cha_addr = *low_chb_addr = *num = 0; 
    }
}

void updae_gating_value(unsigned int high_table, unsigned int cha_r0_fine, unsigned int cha_r1_fine, unsigned int chb_r0_fine, unsigned int chb_r1_fine)
{
    unsigned int *table_cha_addr, *table_chb_addr;
    unsigned long r0_fine_addr, r1_fine_addr;
    
    if(high_table == 0)
    {
      table_cha_addr = g_vcore_dvfs_info.low_freq_cha_setting_addr;
      printk("[vcore dvfs][channel A]modify low freq table: cha table addr = 0x%lx\n", (unsigned long)table_cha_addr);
      
      r0_fine_addr = table_cha_addr + 3;
      r1_fine_addr = table_cha_addr + 6;
      printk("[vcore dvfs][channel A]before modify: r0 fine -> addr = 0x%lx, value = 0x%x\n", r0_fine_addr, *(unsigned int*)(r0_fine_addr));
      printk("[vcore dvfs][channel A]before modify: r1 fine -> addr = 0x%lx, value = 0x%x\n", r1_fine_addr, *(unsigned int*)(r1_fine_addr));
      *(unsigned int*)(r0_fine_addr) = cha_r0_fine;
      *(unsigned int*)(r1_fine_addr) = cha_r1_fine;
      printk("[vcore dvfs][channel A]after  modify: r0 fine -> addr = 0x%lx, value = 0x%x\n", r0_fine_addr, *(unsigned int*)(r0_fine_addr));
      printk("[vcore dvfs][channel A]after  modify: r1 fine -> addr = 0x%lx, value = 0x%x\n", r1_fine_addr, *(unsigned int*)(r1_fine_addr));
                
                
      table_chb_addr = g_vcore_dvfs_info.low_freq_chb_setting_addr; 
      printk("[vcore dvfs][channel B]modify low freq table: chb table addr = 0x%lx\n", (unsigned long)table_chb_addr);
            
      r0_fine_addr = table_chb_addr + 3;
      r1_fine_addr = table_chb_addr + 6;
      printk("[vcore dvfs][channel B] before modify: r0 fine -> addr = 0x%lx, value = 0x%x\n", r0_fine_addr, *(unsigned int*)(r0_fine_addr));
      printk("[vcore dvfs][channel B] before modify: r1 fine -> addr = 0x%lx, value = 0x%x\n", r1_fine_addr, *(unsigned int*)(r1_fine_addr));
      *(unsigned int*)(r0_fine_addr) = chb_r0_fine;
      *(unsigned int*)(r1_fine_addr) = chb_r1_fine;
      printk("[vcore dvfs][channel B] after  modify: r0 fine -> addr = 0x%lx, value = 0x%x\n", r0_fine_addr, *(unsigned int*)(r0_fine_addr));
      printk("[vcore dvfs][channel B] after  modify: r1 fine -> addr = 0x%lx, value = 0x%x\n", r1_fine_addr, *(unsigned int*)(r1_fine_addr));      
    }
    else if(high_table == 1)
    {
      table_cha_addr = g_vcore_dvfs_info.high_freq_cha_setting_addr;
      printk("[vcore dvfs][channel A]modify high freq table: cha table addr = 0x%lx\n", (unsigned long)table_cha_addr);
      
      r0_fine_addr = table_cha_addr + 3;
      r1_fine_addr = table_cha_addr + 6;
      printk("[vcore dvfs][channel A]before modify: r0 fine -> addr = 0x%lx, value = 0x%x\n", r0_fine_addr, *(unsigned int*)(r0_fine_addr));
      printk("[vcore dvfs][channel A]before modify: r1 fine -> addr = 0x%lx, value = 0x%x\n", r1_fine_addr, *(unsigned int*)(r1_fine_addr));
      *(unsigned int*)(r0_fine_addr) = cha_r0_fine;
      *(unsigned int*)(r1_fine_addr) = cha_r1_fine;
      printk("[vcore dvfs][channel A]after  modify: r0 fine -> addr = 0x%lx, value = 0x%x\n", r0_fine_addr, *(unsigned int*)(r0_fine_addr));
      printk("[vcore dvfs][channel A]after  modify: r1 fine -> addr = 0x%lx, value = 0x%x\n", r1_fine_addr, *(unsigned int*)(r1_fine_addr));
                
                
      table_chb_addr = g_vcore_dvfs_info.high_freq_chb_setting_addr; 
      printk("[vcore dvfs][channel B]modify high freq table: chb table addr = 0x%lx\n", (unsigned long)table_chb_addr);
            
      r0_fine_addr = table_chb_addr + 3;
      r1_fine_addr = table_chb_addr + 6;
      printk("[vcore dvfs][channel B] before modify: r0 fine -> addr = 0x%lx, value = 0x%x\n", r0_fine_addr, *(unsigned int*)(r0_fine_addr));
      printk("[vcore dvfs][channel B] before modify: r1 fine -> addr = 0x%lx, value = 0x%x\n", r1_fine_addr, *(unsigned int*)(r1_fine_addr));
      *(unsigned int*)(r0_fine_addr) = chb_r0_fine;
      *(unsigned int*)(r1_fine_addr) = chb_r1_fine;
      printk("[vcore dvfs][channel B] after  modify: r0 fine -> addr = 0x%lx, value = 0x%x\n", r0_fine_addr, *(unsigned int*)(r0_fine_addr));
      printk("[vcore dvfs][channel B] after  modify: r1 fine -> addr = 0x%lx, value = 0x%x\n", r1_fine_addr, *(unsigned int*)(r1_fine_addr));      
    } 
}

#define MEM_TEST_SIZE 0x2000
#define PATTERN1 0x5A5A5A5A
#define PATTERN2 0xA5A5A5A5
int Binning_DRAM_complex_mem_test (void)
{
    unsigned char *MEM8_BASE;
    unsigned short *MEM16_BASE;
    unsigned int *MEM32_BASE;
    unsigned int *MEM_BASE;
    unsigned long mem_ptr;
    unsigned char pattern8;
    unsigned short pattern16;
    unsigned int i, j, size, pattern32;
    unsigned int value;
    unsigned int len=MEM_TEST_SIZE;
    void *ptr;   
    ptr = vmalloc(PAGE_SIZE*2);
    MEM8_BASE=(unsigned char *)ptr;
    MEM16_BASE=(unsigned short *)ptr;
    MEM32_BASE=(unsigned int *)ptr;
    MEM_BASE=(unsigned int *)ptr;
    printk("Test DRAM start address 0x%lx\n",(unsigned long)ptr);
    printk("Test DRAM SIZE 0x%x\n",MEM_TEST_SIZE);
    size = len >> 2;

    /* === Verify the tied bits (tied high) === */
    for (i = 0; i < size; i++)
    {
        MEM32_BASE[i] = 0;
    }

    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0)
        {
            vfree(ptr);
            return -1;
        }
        else
        {
            MEM32_BASE[i] = 0xffffffff;
        }
    }

    /* === Verify the tied bits (tied low) === */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0xffffffff)
        {
            vfree(ptr);
            return -2;
        }
        else
            MEM32_BASE[i] = 0x00;
    }

    /* === Verify pattern 1 (0x00~0xff) === */
    pattern8 = 0x00;
    for (i = 0; i < len; i++)
        MEM8_BASE[i] = pattern8++;
    pattern8 = 0x00;
    for (i = 0; i < len; i++)
    {
        if (MEM8_BASE[i] != pattern8++)
        { 
            vfree(ptr);
            return -3;
        }
    }

    /* === Verify pattern 2 (0x00~0xff) === */
    pattern8 = 0x00;
    for (i = j = 0; i < len; i += 2, j++)
    {
        if (MEM8_BASE[i] == pattern8)
            MEM16_BASE[j] = pattern8;
        if (MEM16_BASE[j] != pattern8)
        {
            vfree(ptr);
            return -4;
        }
        pattern8 += 2;
    }

    /* === Verify pattern 3 (0x00~0xffff) === */
    pattern16 = 0x00;
    for (i = 0; i < (len >> 1); i++)
        MEM16_BASE[i] = pattern16++;
    pattern16 = 0x00;
    for (i = 0; i < (len >> 1); i++)
    {
        if (MEM16_BASE[i] != pattern16++)                                                                                                    
        {
            vfree(ptr);
            return -5;
        }
    }

    /* === Verify pattern 4 (0x00~0xffffffff) === */
    pattern32 = 0x00;
    for (i = 0; i < (len >> 2); i++)
        MEM32_BASE[i] = pattern32++;
    pattern32 = 0x00;
    for (i = 0; i < (len >> 2); i++)
    {
        if (MEM32_BASE[i] != pattern32++)
        { 
            vfree(ptr);
            return -6;
        }
    }

    /* === Pattern 5: Filling memory range with 0x44332211 === */
    for (i = 0; i < size; i++)
        MEM32_BASE[i] = 0x44332211;

    /* === Read Check then Fill Memory with a5a5a5a5 Pattern === */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0x44332211)
        {
            vfree(ptr);
            return -7;
        }
        else
        {
            MEM32_BASE[i] = 0xa5a5a5a5;
        }
    }

    /* === Read Check then Fill Memory with 00 Byte Pattern at offset 0h === */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0xa5a5a5a5)
        { 
            vfree(ptr);
            return -8;  
        }
        else                                                                                                                              
        {
            MEM8_BASE[i * 4] = 0x00;
        }
    }

    /* === Read Check then Fill Memory with 00 Byte Pattern at offset 2h === */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0xa5a5a500)
        {
            vfree(ptr);
            return -9;
        }
        else
        {
            MEM8_BASE[i * 4 + 2] = 0x00;
        }
    }

    /* === Read Check then Fill Memory with 00 Byte Pattern at offset 1h === */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0xa500a500)
        {
            vfree(ptr);
            return -10;
        }
        else
        {
            MEM8_BASE[i * 4 + 1] = 0x00;
        }
    }

    /* === Read Check then Fill Memory with 00 Byte Pattern at offset 3h === */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0xa5000000)
        {
            vfree(ptr);
            return -11;
        }
        else
        {
            MEM8_BASE[i * 4 + 3] = 0x00;
        }
    }

    /* === Read Check then Fill Memory with ffff Word Pattern at offset 1h == */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0x00000000)
        {
            vfree(ptr);
            return -12;
        }
        else
        {
            MEM16_BASE[i * 2 + 1] = 0xffff;
        }
    }


    /* === Read Check then Fill Memory with ffff Word Pattern at offset 0h == */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0xffff0000)
        {
            vfree(ptr);
            return -13;
        }
        else
        {
            MEM16_BASE[i * 2] = 0xffff;
        }
    }
    /*===  Read Check === */
    for (i = 0; i < size; i++)
    {
        if (MEM32_BASE[i] != 0xffffffff)
        {
            vfree(ptr);
            return -14;
        }
    }


    /************************************************
    * Additional verification
    ************************************************/
    /* === stage 1 => write 0 === */

    for (i = 0; i < size; i++)
    {
        MEM_BASE[i] = PATTERN1;
    }


    /* === stage 2 => read 0, write 0xF === */
    for (i = 0; i < size; i++)
    {
        value = MEM_BASE[i];

        if (value != PATTERN1)
        {
            vfree(ptr);
            return -15;
        }
        MEM_BASE[i] = PATTERN2;
    }


    /* === stage 3 => read 0xF, write 0 === */
    for (i = 0; i < size; i++)
    {
        value = MEM_BASE[i];
        if (value != PATTERN2)
        {
            vfree(ptr);
            return -16;
        }
        MEM_BASE[i] = PATTERN1;
    }


    /* === stage 4 => read 0, write 0xF === */
    for (i = 0; i < size; i++)
    {
        value = MEM_BASE[i];
        if (value != PATTERN1)
        {
            vfree(ptr);
            return -17;
        }
        MEM_BASE[i] = PATTERN2;
    }


    /* === stage 5 => read 0xF, write 0 === */
    for (i = 0; i < size; i++)
    {
        value = MEM_BASE[i];
        if (value != PATTERN2)
        { 
            vfree(ptr);
            return -18;
        }
        MEM_BASE[i] = PATTERN1;
    }


    /* === stage 6 => read 0 === */
    for (i = 0; i < size; i++)
    {
        value = MEM_BASE[i];
        if (value != PATTERN1)
        {
            vfree(ptr);
            return -19;
        }
    }


    /* === 1/2/4-byte combination test === */
    mem_ptr = (unsigned long)MEM_BASE;
    while (mem_ptr < ((unsigned long)MEM_BASE + (size << 2)))
    {
        *((unsigned char *) mem_ptr) = 0x78;
        mem_ptr += 1;
        *((unsigned char *) mem_ptr) = 0x56;
        mem_ptr += 1;
        *((unsigned short *) mem_ptr) = 0x1234;
        mem_ptr += 2;
        *((unsigned int *) mem_ptr) = 0x12345678;
        mem_ptr += 4;
        *((unsigned short *) mem_ptr) = 0x5678;
        mem_ptr += 2;
        *((unsigned char *) mem_ptr) = 0x34;
        mem_ptr += 1;
        *((unsigned char *) mem_ptr) = 0x12;
        mem_ptr += 1;
        *((unsigned int *) mem_ptr) = 0x12345678;
        mem_ptr += 4;
        *((unsigned char *) mem_ptr) = 0x78;
        mem_ptr += 1;
        *((unsigned char *) mem_ptr) = 0x56;
        mem_ptr += 1;
        *((unsigned short *) mem_ptr) = 0x1234;
        mem_ptr += 2;
        *((unsigned int *) mem_ptr) = 0x12345678;
        mem_ptr += 4;
        *((unsigned short *) mem_ptr) = 0x5678;
        mem_ptr += 2;
        *((unsigned char *) mem_ptr) = 0x34;
        mem_ptr += 1;
        *((unsigned char *) mem_ptr) = 0x12;
        mem_ptr += 1;
        *((unsigned int *) mem_ptr) = 0x12345678;
        mem_ptr += 4;
    }
    for (i = 0; i < size; i++)
    {
        value = MEM_BASE[i];
        if (value != 0x12345678)
        {
            vfree(ptr);
            return -20;
        }
    }


    /* === Verify pattern 1 (0x00~0xff) === */
    pattern8 = 0x00;
    MEM8_BASE[0] = pattern8;
    for (i = 0; i < size * 4; i++)
    {
        unsigned char waddr8, raddr8;
        waddr8 = i + 1;
        raddr8 = i;
        if (i < size * 4 - 1)
            MEM8_BASE[waddr8] = pattern8 + 1;
        if (MEM8_BASE[raddr8] != pattern8)
        {
            vfree(ptr);
            return -21;
        }
        pattern8++;
    }


    /* === Verify pattern 2 (0x00~0xffff) === */
    pattern16 = 0x00;
    MEM16_BASE[0] = pattern16;
    for (i = 0; i < size * 2; i++)
    {
        if (i < size * 2 - 1)
            MEM16_BASE[i + 1] = pattern16 + 1;
        if (MEM16_BASE[i] != pattern16)
        {
            vfree(ptr);
            return -22;
        }
        pattern16++;
    }
    /* === Verify pattern 3 (0x00~0xffffffff) === */
    pattern32 = 0x00;
    MEM32_BASE[0] = pattern32;
    for (i = 0; i < size; i++)
    {
        if (i < size - 1)
            MEM32_BASE[i + 1] = pattern32 + 1;
        if (MEM32_BASE[i] != pattern32)
        {
            vfree(ptr);
            return -23;
        }
        pattern32++;
    }
    printk("complex R/W mem test pass\n");
    vfree(ptr);
    return 1;
}

static ssize_t complex_mem_test_show(struct device_driver *driver, char *buf)
{
    int ret;
    ret=Binning_DRAM_complex_mem_test();
    if(ret>0)
    {
      return snprintf(buf, PAGE_SIZE, "MEM Test all pass\n");
    }
    else
    {
      return snprintf(buf, PAGE_SIZE, "MEM TEST failed %d \n", ret);
    }
}

static ssize_t complex_mem_test_store(struct device_driver *driver, const char *buf, size_t count)
{
    return count;
}

#ifdef APDMA_TEST
static ssize_t DFS_APDMA_TEST_show(struct device_driver *driver, char *buf)
{
	dma_dummy_read_for_vcorefs(7);

	if (dram_rank_num == DULE_RANK)
		return snprintf(buf, PAGE_SIZE, "DFS APDMA Dummy Read Address src1:%pa src2:%pa\n",
				&dram_base, &dram_add_rank0_base);
	else if (dram_rank_num == SINGLE_RANK)
		return snprintf(buf, PAGE_SIZE, "DFS APDMA Dummy Read Address src1:%pa\n", &dram_base);
	else
		return snprintf(buf, PAGE_SIZE, "DFS APDMA Dummy Read rank number incorrect = %d !!!\n", dram_rank_num);
}

static ssize_t DFS_APDMA_TEST_store(struct device_driver *driver,
				    const char *buf, size_t count)
{
	return count;
}
#endif

U32 ucDram_Register_Read(unsigned long u4reg_addr)
{
    U32 pu4reg_value;

   	pu4reg_value = (*(volatile unsigned int *)(DRAMCAO_BASE_ADDR + (u4reg_addr))) |
				   (*(volatile unsigned int *)(DDRPHY_BASE_ADDR + (u4reg_addr))) |
				   (*(volatile unsigned int *)(DRAMCNAO_BASE_ADDR + (u4reg_addr)));
 
    return pu4reg_value;
}

void ucDram_Register_Write(unsigned long u4reg_addr, unsigned int u4reg_value)
{
	(*(volatile unsigned int *)(DRAMCAO_BASE_ADDR + (u4reg_addr))) = u4reg_value;
	(*(volatile unsigned int *)(DDRPHY_BASE_ADDR + (u4reg_addr))) = u4reg_value;
	(*(volatile unsigned int *)(DRAMCNAO_BASE_ADDR + (u4reg_addr))) = u4reg_value;
    dsb();
}

int dram_can_support_fh(void)
{
    unsigned int value;
    
    value = ucDram_Register_Read(0xf4);
    //printk("dummy regs 0x%x\n", value);

    if(value & (0x1 <<15))
    {
      //printk("DFS could be enabled\n");
      return 1;
    }
    else
    {
      //printk("DFS could NOT be enabled\n");
      return 0;
    }
}

int dram_do_dfs_by_fh(unsigned int target_freq)
{
    int detect_fh = dram_can_support_fh();
    unsigned int target_dds, current_freq;
    
    if(detect_fh == 0)
       return -1;
    
    current_freq = get_dram_data_rate();
    
    if((current_freq == 1333) && (target_freq == 1600000))
    {
        enable_clock(MT_CG_INFRA_GCE, "CQDMA");
        mt_dfs_mpll(0x15ed5e);  //hopping to 1466
        mt_dfs_mpll(0x17ee70);  //hopping to 1600
        disable_clock(MT_CG_INFRA_GCE, "CQDMA");
    }
    else if((current_freq == 1600) && (target_freq == 1333000))
    {
        enable_clock(MT_CG_INFRA_GCE, "CQDMA"); 
        mt_dfs_mpll(0x15ed5e);  //hopping to 1466
        mt_dfs_mpll(0x13f01A);  //hopping to 1333
        disable_clock(MT_CG_INFRA_GCE, "CQDMA");      
    }
    else
    {           
        switch(target_freq)
        {     
            case 1333000:
              target_dds = 0x13f01A;  ///< 1333Mbps
              break;          
            case 1466000:
              target_dds = 0x15ed5e;  ///< 1466Mbps
              break;           
            case 1600000:
              target_dds = 0x17ee70;  ///< 1600Mbps
              break;
            default:
              return -1;  
        }

        enable_clock(MT_CG_INFRA_GCE, "CQDMA");
        mt_dfs_mpll(target_dds);
        disable_clock(MT_CG_INFRA_GCE, "CQDMA");     
    }    

    /* fix up CON_SCE_VSS setting depend on dram freq*/
extern unsigned int fixup_emi_setting_vss(unsigned int cur_dram_freq, int skip);
    fixup_emi_setting_vss((target_freq/1000), 0);

#if 0
    	/* print dst buffer for verification */
	print_hex_dump(KERN_ALERT, "dfs_dummy_buffer: ", DUMP_PREFIX_ADDRESS,
			32, 4, dfs_dummy_buffer, BUFF_LEN, 0);
#endif 

    return 0;
}

bool pasr_is_valid(void)
{
	unsigned int ddr_type=0;

		ddr_type=get_ddr_type();
		/* Following DDR types can support PASR */
      		if(ddr_type == LPDDR3_1866 || 
      		   ddr_type == DUAL_LPDDR3_1600 || 
      		   ddr_type == LPDDR2) 
        {
			return true;
		}

	return false;
}

//-------------------------------------------------------------------------
/** Round_Operation
 *  Round operation of A/B
 *  @param  A   
 *  @param  B   
 *  @retval round(A/B) 
 */
//-------------------------------------------------------------------------
U32 Round_Operation(U32 A, U32 B)
{
    U32 temp;

    if (B == 0)
    {
        return 0xffffffff;
    }
    
    temp = A/B;
        
    if ((A-temp*B) >= ((temp+1)*B-A))
    {
        return (temp+1);
    }
    else
    {
        return temp;
    }    
}

U32 get_dram_data_rate()
{
    U32 u4value1, u4value2, MPLL_POSDIV, MPLL_PCW, MPLL_FOUT;
    U32 MEMPLL_FBKDIV, MEMPLL_FOUT;
    U32 MODE_1PLL;
    
    if((SLEEP_BASE_ADDR == NULL) || (APMIXED_BASE_ADDR == NULL) || (DDRPHY_BASE_ADDR == NULL))
        return 0;
      
    MODE_1PLL = ((*(volatile unsigned int *)(SLEEP_BASE_ADDR + 0x010)) & 0x08000000) >> 27;     
               
    u4value1 = (*(volatile unsigned int *)(APMIXED_BASE_ADDR + 0x280));
    u4value2 = (u4value1 & 0x00000070) >> 4;
    if (u4value2 == 0)
    {
        MPLL_POSDIV = 1;
    }
    else if (u4value2 == 1)
    {
        MPLL_POSDIV = 2;
    }
    else if (u4value2 == 2)
    {
        MPLL_POSDIV = 4;
    }
    else if (u4value2 == 3)
    {
        MPLL_POSDIV = 8;
    }
    else
    {
        MPLL_POSDIV = 16;
    }

    u4value1 = *(volatile unsigned int *)(APMIXED_BASE_ADDR + 0x284);
    MPLL_PCW = (u4value1 & 0x001fffff);

    MPLL_FOUT = 26/1*MPLL_PCW;
    MPLL_FOUT = Round_Operation(MPLL_FOUT, MPLL_POSDIV*28); // freq*16384

    if (MODE_1PLL)
    {
        // PLL 3
        u4value1 = *(volatile unsigned int *)(DDRPHY_BASE_ADDR + 0x620);
    }
    else
    {
        // PLL 2
        u4value1 = *(volatile unsigned int *)(DDRPHY_BASE_ADDR + 0x614);
    }
    
    MEMPLL_FBKDIV = (u4value1 & 0x007f0000) >> 16;

    MEMPLL_FOUT = MPLL_FOUT*1*4*(MEMPLL_FBKDIV+1);
    MEMPLL_FOUT = Round_Operation(MEMPLL_FOUT, 16384);

    //printk("MPLL_POSDIV=%d, MPLL_PCW=0x%x, MPLL_FOUT=%d, MEMPLL_FBKDIV=%d, MEMPLL_FOUT=%d\n", MPLL_POSDIV, MPLL_PCW, MPLL_FOUT, MEMPLL_FBKDIV, MEMPLL_FOUT);
    // return 1333, even if is 1332, since this API uses always treat it as 1333
	if (MEMPLL_FOUT == 1332)
	{
		MEMPLL_FOUT = 1333;
	}
    return MEMPLL_FOUT;
}

unsigned int DRAM_MRR(int MRR_num)
{
    unsigned int MRR_value = 0x0;
    unsigned int u4value; 
          
    // set DQ bit 0, 1, 2, 3, 4, 5, 6, 7 pinmux for LPDDR3             
    ucDram_Register_Write(DRAMC_REG_RRRATE_CTL, 0x13121110);
    ucDram_Register_Write(DRAMC_REG_MRR_CTL, 0x17161514);
    
    ucDram_Register_Write(DRAMC_REG_MRS, MRR_num);
    ucDram_Register_Write(DRAMC_REG_SPCMD, ucDram_Register_Read(DRAMC_REG_SPCMD) | 0x00000002);
    //udelay(1);
    while ((ucDram_Register_Read(DRAMC_REG_SPCMDRESP) & 0x02) == 0);
    ucDram_Register_Write(DRAMC_REG_SPCMD, ucDram_Register_Read(DRAMC_REG_SPCMD) & 0xFFFFFFFD);

    
    u4value = ucDram_Register_Read(DRAMC_REG_SPCMDRESP);
    MRR_value = (u4value >> 20) & 0xFF;

    return MRR_value;
}

unsigned int read_dram_temperature(void)
{
    unsigned int value;
        
    value = DRAM_MRR(4) & 0x7;
    return value;
}

static int dt_scan_dram_info(unsigned long node, const char *uname, int depth, void *data)
{
	char *type = of_get_flat_dt_prop(node, "device_type", NULL);
	__be32 *reg, *endp;
	dram_info_t *dram_info = NULL;
	unsigned long l;

	/* We are scanning "memory" nodes only */
	if (type == NULL) {
		/*
		* The longtrail doesn't have a device_type on the
		* /memory node, so look for the node called /memory@0.
		*/
		if (depth != 1 || strcmp(uname, "memory@0") != 0)
			return 0;
	} else if (strcmp(type, "memory") != 0)
		return 0;

	reg = of_get_flat_dt_prop(node, "reg", &l);
	if (reg == NULL)
		return 0;

	endp = reg + (l / sizeof(__be32));

	if (node) {
		/* orig_dram_info */
		dram_info = (const struct dram_info *)of_get_flat_dt_prop(node, "orig_dram_info", NULL);
		if (dram_info == NULL)
			return 0;

		dram_rank_num = dram_info->rank_num;
		dram_base = dram_info->rank_info[0].start;

		if (dram_rank_num == SINGLE_RANK)
			pr_err("[DFS]  enable (dram base): (%pa)\n", &dram_base);
		else {
			dram_add_rank0_base = dram_info->rank_info[1].start;
			pr_err("[DFS]  enable (dram base, dram rank1 base): (%pa,%pa)\n",
								&dram_base, &dram_add_rank0_base);
		}
	}

	return node;
}
#ifdef READ_DRAM_TEMP_TEST
static ssize_t read_dram_temp_show(struct device_driver *driver, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "DRAM MR4 = 0x%x \n", read_dram_temperature());
}
static ssize_t read_dram_temp_store(struct device_driver *driver, const char *buf, size_t count)
{
    return count;
}
#endif

#ifdef FREQ_HOPPING_TEST
static ssize_t freq_hopping_test_show(struct device_driver *driver, char *buf)
{
    int dfs_ability = 0;
    
    dfs_ability = dram_can_support_fh();
    
    if(dfs_ability == 0)
      return snprintf(buf, PAGE_SIZE, "DRAM DFS can not be enabled\n");
    else
      return snprintf(buf, PAGE_SIZE, "DRAM DFS can be enabled\n");
}

static ssize_t freq_hopping_test_store(struct device_driver *driver, const char *buf, size_t count)
{
    unsigned int freq;
    
    if (sscanf(buf, "%u", &freq) != 1)
      return -EINVAL;
    
    printk("[DRAM DFS] freqency hopping to %dKHz\n", freq);
    dram_do_dfs_by_fh(freq);
      
    return count;
}
#endif

static ssize_t read_dram_data_rate_show(struct device_driver *driver, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "DRAM data rate = %d \n", get_dram_data_rate());
}
static ssize_t read_dram_data_rate_store(struct device_driver *driver, const char *buf, size_t count)
{
    return count;
}

DRIVER_ATTR(emi_clk_mem_test, 0664, complex_mem_test_show, complex_mem_test_store);

#ifdef APDMA_TEST
DRIVER_ATTR(dram_dummy_read_test, 0664, DFS_APDMA_TEST_show, DFS_APDMA_TEST_store);
#endif

#ifdef READ_DRAM_TEMP_TEST
DRIVER_ATTR(read_dram_temp_test, 0664, read_dram_temp_show, read_dram_temp_store);
#endif

#ifdef FREQ_HOPPING_TEST
DRIVER_ATTR(freq_hopping_test, 0664, freq_hopping_test_show, freq_hopping_test_store);
#endif

DRIVER_ATTR(read_dram_data_rate, 0664, read_dram_data_rate_show, read_dram_data_rate_store);

static struct device_driver dram_test_drv =
{
    .name = "emi_clk_test",
    .bus = &platform_bus_type,
    .owner = THIS_MODULE,
};

extern char __ssram_text, _sram_start, __esram_text;
int __init dram_test_init(void)
{
    int ret;
    struct device_node *node;

    /* DTS version */  
    node = of_find_compatible_node(NULL, NULL, "mediatek,APMIXED");
    if(node) {
        APMIXED_BASE_ADDR = of_iomap(node, 0);
        printk("get APMIXED_BASE_ADDR @ %p\n", APMIXED_BASE_ADDR);
    } else {
        printk("can't find APMIXED compatible node\n");
        return -1;
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,CQDMA");
    if(node) {
        CQDMA_BASE_ADDR = of_iomap(node, 0);
        printk("[DRAMC]get CQDMA_BASE_ADDR @ %p\n", CQDMA_BASE_ADDR);
    } else {
        printk("[DRAMC]can't find CQDMA compatible node\n");
        return -1;
    }
        
	  node = of_find_compatible_node(NULL, NULL, "mediatek,DRAMC0");
	  if (node) {
        DRAMCAO_BASE_ADDR = of_iomap(node, 0);
        printk("[DRAMC]get DRAMCAO_BASE_ADDR @ %p\n", DRAMCAO_BASE_ADDR);
    }
    else {
        printk("[DRAMC]can't find DRAMC0 compatible node\n");
        return -1;
    }
    
    node = of_find_compatible_node(NULL, NULL, "mediatek,DDRPHY");
    if(node) {
        DDRPHY_BASE_ADDR = of_iomap(node, 0);
        printk("[DRAMC]get DDRPHY_BASE_ADDR @ %p\n", DDRPHY_BASE_ADDR);
    }
    else {
        printk("[DRAMC]can't find DDRPHY compatible node\n");
        return -1;
    }
    
    node = of_find_compatible_node(NULL, NULL, "mediatek,DRAMC_NAO");
    if(node) {
        DRAMCNAO_BASE_ADDR = of_iomap(node, 0);
        printk("[DRAMC]get DRAMCNAO_BASE_ADDR @ %p\n", DRAMCNAO_BASE_ADDR);
    }
    else {
        printk("[DRAMC]can't find DRAMCNAO compatible node\n");
	  	return -1;
	  }

    node = of_find_compatible_node(NULL, NULL, "mediatek,SLEEP");
    if(node) {
        SLEEP_BASE_ADDR = of_iomap(node, 0);
        printk("[DRAMC]get SLEEP_BASE_ADDR @ %p\n", SLEEP_BASE_ADDR);
    }
    else {
        printk("[DRAMC]can't find SLEEP compatible node\n");
	  	return -1;
	  }
#ifdef CONFIG_OF
	if (dram_rank_num == 0) { /* happend to no rsv mem node from LK */
		node = of_scan_flat_dt(dt_scan_dram_info, NULL);
		if (node) {
			pr_err("[DRAMC]find dt_scan_dram_info\n");
		} else {
			pr_err("[DRAMC]can't find dt_scan_dram_info\n");
			return -1;
		}
	}
#else
	pr_err("[DRAMC] Error: undefined dummy read!!!\n");
#endif
           
    ret = driver_register(&dram_test_drv);
    if (ret) {
        printk(KERN_ERR "fail to create the dram_test driver\n");
        return ret;
    }

    ret = driver_create_file(&dram_test_drv, &driver_attr_emi_clk_mem_test);
    if (ret) {
        printk(KERN_ERR "fail to create the emi_clk_mem_test sysfs files\n");
        return ret;
    }

#ifdef APDMA_TEST
    ret = driver_create_file(&dram_test_drv, &driver_attr_dram_dummy_read_test);
    if (ret) {
        printk(KERN_ERR "fail to create the DFS sysfs files\n");
        return ret;
    }
#endif

#ifdef READ_DRAM_TEMP_TEST
    ret = driver_create_file(&dram_test_drv, &driver_attr_read_dram_temp_test);
    if (ret) {
        printk(KERN_ERR "fail to create the read dram temp sysfs files\n");
        return ret;
}
#endif

#ifdef FREQ_HOPPING_TEST
    ret = driver_create_file(&dram_test_drv, &driver_attr_freq_hopping_test);
    if (ret) {
        printk(KERN_ERR "fail to create the read dram temp sysfs files\n");
        return ret;
    }
#endif

    ret = driver_create_file(&dram_test_drv, &driver_attr_read_dram_data_rate);
    if (ret) {
        printk(KERN_ERR "fail to create the read dram data rate sysfs files\n");
        return ret;
    }
    
    printk(KERN_INFO "[DRAMC Driver] Store Vcore DVFS settings...\n");
    store_vcore_dvfs_setting();
#if 0
    printk(KERN_INFO "[DRAMC Driver] Register MD32 Vcore DVFS Handler...\n");
    md32_ipi_registration(IPI_VCORE_DVFS, vcore_dvfs_ipi_handler, "vcore_dvfs");
#endif   
    org_dram_data_rate = get_dram_data_rate();
    printk("[DRAMC Driver] Dram Data Rate = %d\n", org_dram_data_rate);
    
    if(dram_can_support_fh())
      printk("[DRAMC Driver] dram can support Frequency Hopping\n");
    else
      printk("[DRAMC Driver] dram can not support Frequency Hopping\n");
    
    return 0;
}

#ifdef CONFIG_OF_RESERVED_MEM
int dram_dummy_read_reserve_mem_of_init(struct reserved_mem *rmem)
{
	phys_addr_t rptr = 0;
	unsigned int rsize = 0;

	rptr = rmem->base;
	rsize = (unsigned int)rmem->size;

	if (strstr(DRAM_R0_DUMMY_READ_RESERVED_KEY, rmem->name)) {
		dram_base = rptr;
		dram_rank_num++;
		pr_debug("[dram_dummy_read_reserve_mem_of_init] dram_base = %pa, size = 0x%x\n",
				&dram_base, rsize);
	}

	if (strstr(DRAM_R1_DUMMY_READ_RESERVED_KEY, rmem->name)) {
		dram_add_rank0_base = rptr;
		dram_rank_num++;
		pr_debug("[dram_dummy_read_reserve_mem_of_init] dram_add_rank0_base = %pa, size = 0x%x\n",
				&dram_add_rank0_base, rsize);
	}

	return 0;
}
RESERVEDMEM_OF_DECLARE(dram_reserve_r0_dummy_read_init, DRAM_R0_DUMMY_READ_RESERVED_KEY,
			dram_dummy_read_reserve_mem_of_init);
RESERVEDMEM_OF_DECLARE(dram_reserve_r1_dummy_read_init, DRAM_R1_DUMMY_READ_RESERVED_KEY,
			dram_dummy_read_reserve_mem_of_init);
#endif

int DFS_APDMA_early_init(void)
{
    phys_addr_t max_dram_size = get_max_DRAM_size();
    phys_addr_t dummy_read_center_address;

    if(init_done == 0)
    {
        if(max_dram_size == 0x100000000ULL )  //dram size = 4GB
        {
          dummy_read_center_address = 0x80000000ULL; 
        }
        else if (max_dram_size <= 0xC0000000) //dram size <= 3GB   
        {
          dummy_read_center_address = DRAM_BASE+(max_dram_size >> 1); 
        }
        else
        {
          ASSERT(0);
        }   
        
        src_array_p = (volatile unsigned int)(dummy_read_center_address - (BUFF_LEN >> 1));
	dst_array_p = __pa(dfs_dummy_buffer);
	pr_alert("[DFS]dfs_dummy_buffer va: 0x%p, pa: 0x%llx, size: %d\n",
		(void *)dfs_dummy_buffer,
		(unsigned long long)dst_array_p,
		BUFF_LEN);
        
#ifdef APDMAREG_DUMP        
        src_array_v = ioremap(rounddown(src_array_p,IOREMAP_ALIGMENT),IOREMAP_ALIGMENT << 1)+IOREMAP_ALIGMENT-(BUFF_LEN >> 1);
        dst_array_v = src_array_v+BUFF_LEN;
#endif
        
        init_done = 1;
    }    

   return 1;
}
int DFS_APDMA_Init(void)
{
    writel(((~DMA_GSEC_EN_BIT)&readl(DMA_GSEC_EN)), DMA_GSEC_EN);
    return 1;
}

int DFS_APDMA_Enable(void)
{
#ifdef APDMAREG_DUMP    
    int i;
#endif

    while(readl(DMA_START)& 0x1);
#if 0
    writel(src_array_p, DMA_SRC);
    writel(dst_array_p, DMA_DST);
    writel(BUFF_LEN , DMA_LEN1);
    writel(DMA_CON_BURST_8BEAT, DMA_CON);    
#else
	if (dram_rank_num == DULE_RANK) {
		writel(dram_base, DMA_SRC);
		writel(dram_add_rank0_base, DMA_SRC2);
		writel(dst_array_p, DMA_DST);
		writel((BUFF_LEN >> 1), DMA_LEN1);
		writel((BUFF_LEN >> 1), DMA_LEN2);
		writel((DMA_CON_BURST_8BEAT | DMA_CON_WPEN), DMA_CON);
	}	else if (dram_rank_num == SINGLE_RANK) {
			writel(dram_base, DMA_SRC);
			writel(dst_array_p, DMA_DST);
			writel(BUFF_LEN, DMA_LEN1);
			writel(DMA_CON_BURST_8BEAT, DMA_CON);
		} else {
			pr_err("[DFS] error rank number = %x\n", dram_rank_num);
			return 1;
			}
#endif

#ifdef APDMAREG_DUMP
   printk("src_p=0x%x, dst_p=0x%x, src_v=0x%x, dst_v=0x%x, len=%d\n", src_array_p, dst_array_p, (unsigned int)src_array_v, (unsigned int)dst_array_v, BUFF_LEN);
   for (i=0;i<0x60;i+=4)
   {
     printk("[Before]addr:0x%x, value:%x\n",(unsigned int)(DMA_BASE+i),*((volatile int *)(DMA_BASE+i)));
   }                          
     
#ifdef APDMA_TEST
   for(i = 0; i < BUFF_LEN/sizeof(unsigned int); i++) {
                dst_array_v[i] = 0;
                src_array_v[i] = i;
        }
#endif
#endif

  mt_reg_sync_writel(0x1,DMA_START);
   
#ifdef APDMAREG_DUMP
   for (i=0;i<0x60;i+=4)
   {
     printk("[AFTER]addr:0x%x, value:%x\n",(unsigned int)(DMA_BASE+i),*((volatile int *)(DMA_BASE+i)));
   }

#ifdef APDMA_TEST
        for(i = 0; i < BUFF_LEN/sizeof(unsigned int); i++){
                if(dst_array_v[i] != src_array_v[i]){
                        printk("DMA ERROR at Address %x\n (i=%d, value=0x%x(should be 0x%x))", (unsigned int)&dst_array_v[i], i, dst_array_v[i], src_array_v[i]);
                        ASSERT(0);
                }
        }
        printk("Channe0 DFS DMA TEST PASS\n");
#endif
#endif
        return 1;     
}

int DFS_APDMA_END(void)
{
    while(readl(DMA_START));
    return 1 ;
}

void DFS_APDMA_dummy_read_preinit() 
{
    DFS_APDMA_early_init();    
}

void DFS_APDMA_dummy_read_deinit()
{
}

void dma_dummy_read_for_vcorefs(int loops)
{
    int i, count;
    unsigned long long start_time, end_time, duration;
    
    DFS_APDMA_early_init();    
    enable_clock(MT_CG_INFRA_GCE, "CQDMA");
    for(i=0; i<loops; i++)
    {
        count = 0;        
        start_time = sched_clock();
        do{
            DFS_APDMA_Enable();
            DFS_APDMA_END();
            end_time = sched_clock();
            duration = end_time - start_time;
            count++;
        }while(duration < 16000L);
        //printk("[DMA_dummy_read[%d], duration=%lld, count = %d\n", duration, count);   
    }        
    disable_clock(MT_CG_INFRA_GCE, "CQDMA");
}

/*
 * XXX: Reserved memory in low memory must be 1MB aligned.
 *     This is because the Linux kernel always use 1MB section to map low memory.
 *
 *    We Reserved the memory regien which could cross rank for APDMA to do dummy read.
 *    
 */

void DFS_Reserved_Memory(void)
{
  phys_addr_t high_memory_phys;
  phys_addr_t DFS_dummy_read_center_address;
  phys_addr_t max_dram_size = get_max_DRAM_size();

  high_memory_phys=virt_to_phys(high_memory);

  if(max_dram_size == 0x100000000ULL )  //dram size = 4GB
  {
    DFS_dummy_read_center_address = 0x80000000ULL; 
  }
  else if (max_dram_size <= 0xC0000000) //dram size <= 3GB   
  {
    DFS_dummy_read_center_address = DRAM_BASE+(max_dram_size >> 1);
  }
  else
  {
    ASSERT(0);
  }

  /*For DFS Purpose, we remove this memory block for Dummy read/write by APDMA.*/
  printk("[DFS Check]DRAM SIZE:0x%llx\n",(unsigned long long)max_dram_size);
  printk("[DFS Check]DRAM Dummy read from:0x%llx to 0x%llx\n",(unsigned long long)(DFS_dummy_read_center_address-(BUFF_LEN >> 1)),(unsigned long long)(DFS_dummy_read_center_address+(BUFF_LEN >> 1)));
  printk("[DFS Check]DRAM Dummy read center address:0x%llx\n",(unsigned long long)DFS_dummy_read_center_address);
  printk("[DFS Check]High Memory start address 0x%llx\n",(unsigned long long)high_memory_phys);
  
  if((DFS_dummy_read_center_address - SZ_4K) >= high_memory_phys){
    printk("[DFS Check]DFS Dummy read reserved 0x%llx to 0x%llx\n",(unsigned long long)(DFS_dummy_read_center_address-SZ_4K),(unsigned long long)(DFS_dummy_read_center_address+SZ_4K));
    memblock_reserve(DFS_dummy_read_center_address-SZ_4K, (SZ_4K << 1));
    memblock_free(DFS_dummy_read_center_address-SZ_4K, (SZ_4K << 1));
    memblock_remove(DFS_dummy_read_center_address-SZ_4K, (SZ_4K << 1));
  }
  else{
#ifndef CONFIG_ARM_LPAE    
    printk("[DFS Check]DFS Dummy read reserved 0x%llx to 0x%llx\n",(unsigned long long)(DFS_dummy_read_center_address-SZ_1M),(unsigned long long)(DFS_dummy_read_center_address+SZ_1M));
    memblock_reserve(DFS_dummy_read_center_address-SZ_1M, (SZ_1M << 1));
    memblock_free(DFS_dummy_read_center_address-SZ_1M, (SZ_1M << 1));
    memblock_remove(DFS_dummy_read_center_address-SZ_1M, (SZ_1M << 1));
#else
    printk("[DFS Check]DFS Dummy read reserved 0x%llx to 0x%llx\n",(unsigned long long)(DFS_dummy_read_center_address-SZ_2M),(unsigned long long)(DFS_dummy_read_center_address+SZ_2M));
    memblock_reserve(DFS_dummy_read_center_address-SZ_2M, (SZ_2M << 1));
    memblock_free(DFS_dummy_read_center_address-SZ_2M, (SZ_2M << 1));
    memblock_remove(DFS_dummy_read_center_address-SZ_2M, (SZ_2M << 1));
#endif    
  }
 
  return;
}

arch_initcall(dram_test_init);
