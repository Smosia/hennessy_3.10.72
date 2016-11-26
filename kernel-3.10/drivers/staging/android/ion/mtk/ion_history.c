#include <asm/page.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "ion_priv.h"
#include <linux/slab.h>
#include <linux/xlog.h>
#include <mach/m4u.h>
#include <linux/ion_drv.h>
#include <linux/mutex.h>
#include <linux/mmprofile.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include "ion_profile.h"
#include "ion_drv_priv.h"


//==============================================
//          history record
//==============================================

struct history_record{
	void *record;
	unsigned int record_num;
	unsigned int record_size;
	unsigned int top;
	unsigned int wrapped;
	spinlock_t lock;
	const char *name;
	struct dentry *debug_file;
	int (*show)(struct seq_file *seq, void *record, void *priv);
	int (*destory_record)(void *record, void *priv);
	void *private;
	unsigned long bitmap_busy[0];
};

static unsigned long inline history_record_test_busy(
							struct history_record *history_record, 
							unsigned int index)
{
	unsigned long *p = history_record->bitmap_busy + index / BITS_PER_LONG;
	int bit_mask = 1UL << (index % BITS_PER_LONG);
	return *p & bit_mask;
}

static void inline history_record_set_busy(
							struct history_record *history_record, 
							unsigned int index)
{
	unsigned long *p = history_record->bitmap_busy + index / BITS_PER_LONG;
	int bit_mask = 1UL << (index % BITS_PER_LONG);
	*p |= bit_mask;
}

static void inline history_record_clear_busy(
							struct history_record *history_record, 
							unsigned int index)
{
	unsigned long *p = history_record->bitmap_busy + index / BITS_PER_LONG;
	int bit_mask = 1UL << (index % BITS_PER_LONG);
	*p &= ~bit_mask;
}

static void inline history_record_dump_busy(struct seq_file *seq, 
						struct history_record *history_record)
{
	unsigned long longs = BITS_TO_LONGS(history_record->record_num);
	unsigned long i;
	for(i=0; i<longs; i++) {
		if(seq)
			seq_printf(seq, "0x%lx, ", history_record->bitmap_busy[i]);
		else
			printk("0x%lx, ", history_record->bitmap_busy[i]);
	}
	if(seq)
		seq_printf(seq, "\n");
	else
		printk("\n");
}

void* history_record_get_record(struct history_record *history_record)
{
	unsigned int index;
	void *record;
	
	spin_lock(&history_record->lock);
	
	index = history_record->top;
	history_record->top++;
	if(history_record->top >= history_record->record_num) {
		history_record->top = 0;
		history_record->wrapped = 1;
	}

	if(history_record_test_busy(history_record, index)) {
		IONMSG("%s: error to get record %d, bitmap is:\n", __FUNCTION__, index);
		history_record_dump_busy(NULL, history_record);
		spin_unlock(&history_record->lock);
		return NULL;
	}

	record = history_record->record + history_record->record_size * index;
	
	if(history_record->wrapped && history_record->destory_record)
		history_record->destory_record(record, history_record->private);
	
	history_record_set_busy(history_record, index);
	
	spin_unlock(&history_record->lock);

	return record;
}
void history_record_put_record(struct history_record *history_record, void* record)
{
	unsigned int index = (record - history_record->record)
						/ history_record->record_size;
	spin_lock(&history_record->lock);
	history_record_clear_busy(history_record, index);
	spin_unlock(&history_record->lock);
}


struct history_seq_priv {
	struct history_record * history_record;
	unsigned int start;
	unsigned int num;
};


int history_record_show(struct seq_file *seq, void *record)
{
	struct history_seq_priv *seq_priv = seq->private;
	struct history_record * history_record = seq_priv->history_record;
	unsigned int index = (record - history_record->record)
						/ history_record->record_size;
	spin_lock(&history_record->lock);
	
	if(history_record_test_busy(history_record, index)){
		spin_unlock(&history_record->lock);
		return 0;
	}

	history_record->show(seq, record, history_record->private);

	spin_unlock(&history_record->lock);
	return 0;
}

static void *history_seq_start(struct seq_file *p, loff_t *pos)
{
	struct history_seq_priv *seq_priv = p->private;
	struct history_record * history_record = seq_priv->history_record;
	unsigned int index;

	if(*pos == 0) {
		spin_lock(&history_record->lock);
		if(!history_record->wrapped) {
			seq_priv->start = 0;
			seq_priv->num = history_record->top;
		} else {
			seq_priv->start = history_record->top;
			seq_priv->num = history_record->record_num;
		}
		spin_unlock(&history_record->lock);
	} 
	
	if(*pos >= seq_priv->num)
		return NULL;

	index = seq_priv->start + *pos;
	if(index >= history_record->record_num)
		index -= history_record->record_num;

	return history_record->record + history_record->record_size * index;
}

static void *history_seq_next(struct seq_file *p, void *v, loff_t *pos)
{
	struct history_seq_priv *seq_priv = p->private;
	struct history_record * history_record = seq_priv->history_record;
	unsigned int index;
	
	++*pos;
	if(*pos >= seq_priv->num)
		return NULL;
	
	index = seq_priv->start + *pos;
	if(index >= history_record->record_num)
		index -= history_record->record_num;

	return history_record->record + history_record->record_size * index;
}

static void history_seq_stop(struct seq_file *p, void *v)
{
}

static struct seq_operations seq_op = {
	.start = history_seq_start,
	.next = history_seq_next,
	.stop = history_seq_stop,
	.show = history_record_show,
};
static int history_record_open(struct inode *inode, struct file *file)
{
	struct history_record * history_record = inode->i_private;
	struct history_seq_priv *seq_priv;
	int res = -ENOMEM;

	res = seq_open(file, &seq_op);
	if (res) {
		IONMSG("%s fail\n", __FUNCTION__);
		return res;
	}

	seq_priv = kzalloc(sizeof(*seq_priv), GFP_KERNEL);
	seq_priv->history_record = history_record;
	((struct seq_file *)file->private_data)->private = seq_priv;
	
	return 0;
}

static int history_record_release(struct inode *inode, struct file *file)
{
	struct history_seq_priv *seq_priv = 
		((struct seq_file *)file->private_data)->private;
	
	if(seq_priv)
		kfree(seq_priv);

	return seq_release(inode, file);
}

static const struct file_operations history_record_fops = {
	.open = history_record_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = history_record_release,
};


struct history_record * history_record_create(unsigned int record_num, 
				unsigned int record_size, 
				int (*show)(struct seq_file *seq, void *record, void *priv),
				int (*destory_record)(void *record, void *priv),
				void *priv,
				const char *name,
				struct dentry *debugfs_parent
				)
{
	struct history_record *history_record;
	int num_align;
	size_t size_align;
	int bitmap_bytes;

	/*	as vmalloc is page align. 
		we will enlarge record_num to num_align 
		to get the utmost of memory allocated
	*/
	size_align = record_num * record_size;
	size_align = ALIGN(size_align, PAGE_SIZE);
	num_align = size_align / record_size;

	bitmap_bytes = BITS_TO_LONGS(num_align) * sizeof(unsigned long);

	history_record = kzalloc(sizeof(struct history_record) + bitmap_bytes, GFP_KERNEL);
	if(!history_record) {
		IONMSG("%s error to kzalloc %zd.\n", __FUNCTION__, sizeof(struct history_record));
		return ERR_PTR(-ENOMEM);
	}

	history_record->record = vzalloc(size_align);
	if(!history_record->record) {
		IONMSG("%s error to valloc %zu.\n", __FUNCTION__, size_align);
		kfree(history_record);
		return ERR_PTR(-ENOMEM);
	}

	history_record->record_num = num_align;
	history_record->record_size = record_size;
	history_record->show = show;
	history_record->destory_record = destory_record;
	history_record->name = name;
	spin_lock_init(&history_record->lock);

	history_record->debug_file = debugfs_create_file(name, 0644, 
			debugfs_parent, history_record, &history_record_fops);
	
	return history_record;
}

void history_record_destroy(struct history_record *history_record)
{

	int busy;
	unsigned int i, bitmap_longs = BITS_TO_LONGS(history_record->record_num);
	unsigned int end;
	
	debugfs_remove(history_record->debug_file);

	/* wait untill no busy */
	do {
		busy = 0;
		spin_lock(&history_record->lock);
		for(i=0; i<bitmap_longs; i++) {
			if(history_record->bitmap_busy[i]) {
				/* busy ! */
				IONMSG("warning: %s when busy %d\n", __FUNCTION__, i);
				spin_unlock(&history_record->lock);
				busy = 1;
				cond_resched();
				break;
			}
		}
	}while(busy);
	/* we have history_record->lock locked here */

	if(!history_record->wrapped)
		end = history_record->top;
	else
		end = history_record->record_num;

	for(i=0; i<end; i++) {
		history_record->destory_record(
			history_record->record + i * history_record->record_num,
			history_record->private);
	}

	vfree(history_record->record);
	history_record->record = NULL;
	kfree(history_record);
	
	return ;
}


//====== string hash ==============
struct string_struct {
	unsigned int ref;
	struct hlist_node list;
	char str[0];
};

#define STR_HASH_BUCKET_NUM 32
static struct hlist_head ion_str_hash[STR_HASH_BUCKET_NUM];
DEFINE_SPINLOCK(ion_str_hash_lock);

/* as tested, simple add hash is better than RS_hash & BKDR_hash ! */
static unsigned int ADDHash(char* str, unsigned int len)
{
	unsigned int hash = 0, i;
	for(i=0; i<len; i++) {
		hash += str[i];
	}
	return hash % STR_HASH_BUCKET_NUM;
}

static struct string_struct * string_hash_get(const char* str)
{
	struct hlist_head *head;
	struct string_struct *string;
	unsigned int len = strlen(str);
	unsigned int hash;
	
	hash = ADDHash((char *)str, len);
	head = &ion_str_hash[hash];

	spin_lock(&ion_str_hash_lock);
	hlist_for_each_entry(string, head, list) {
		if(!strcmp(str, string->str)) 
			break;
	}
	
	if(string) {
		string->ref++;
	} else {
		/* add string */
		string = kzalloc(sizeof(*string) + len + 1, GFP_ATOMIC);
		if(!string) {
			IONMSG("%s: kzalloc fail size=%zd.\n", __FUNCTION__, 
					sizeof(*string) + len + 1);
			goto out;
		}
		string->ref = 1;
		INIT_HLIST_NODE(&string->list);
		memcpy(string->str, str, len);
		string->str[len] = '\0';
		hlist_add_head(&string->list, head);
	}

out:
	spin_unlock(&ion_str_hash_lock);
	return string;
	
}

static int string_hash_put(struct string_struct *string)
{
	spin_lock(&ion_str_hash_lock);

	if(!string->ref) {
		IONMSG("error %s string_ref is 0!!!\n", __FUNCTION__);
		spin_unlock(&ion_str_hash_lock);
		return -EINVAL;
	}
	string->ref--;

	if(!string->ref)
	{
		hlist_del(&string->list);
		kfree(string);
	}
	
	spin_unlock(&ion_str_hash_lock);
	return 0;
	
}

int string_hash_debug_show(struct seq_file *seq, void *unused)
{
	struct hlist_head *head;
	struct string_struct *string;
	unsigned int hash, num;
	
	spin_lock(&ion_str_hash_lock);
	
	for(hash=0; hash<STR_HASH_BUCKET_NUM; hash++) {
		head = &ion_str_hash[hash];
		num = 0;
		hlist_for_each_entry(string, head, list) {
			seq_printf(seq, "\t%s : %d\n", string->str, string->ref);
			num++;
		}
		seq_printf(seq, "hash %d : %d strings\n", hash, num);
	}
	
	spin_unlock(&ion_str_hash_lock);
	return 0;
	
}

static int string_hash_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, string_hash_debug_show, inode->i_private);
}

static const struct file_operations string_hash_debug_fops = {
	.open = string_hash_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


//===== ion client history  =======

struct ion_client_record {
	union {
		struct {
			struct string_struct *client_name;
			struct string_struct *dbg_name;
		};

		unsigned long long time;
	};
	size_t size;

	#define CLIENT_ADDRESS_TOTAL 	((void*)1)
	#define CLIENT_ADDRESS_ORPHAN 	((void*)2)
	#define CLIENT_ADDRESS_FLAG_MAX	((void*)0x1000)
	void *address;
};


static int ion_client_record_show(struct seq_file *seq, void *record, void *priv)
{
	struct ion_client_record *client_record = record;

	if(client_record->address > CLIENT_ADDRESS_FLAG_MAX) {
		char *client_name = NULL, *dbg_name = NULL;
		if(client_record->client_name)
			client_name = client_record->client_name->str;
		if(client_record->dbg_name)
			dbg_name = client_record->dbg_name->str;
		seq_printf(seq, "%16.s(%16.s) %16zu 0x%p\n",
				client_name,
				dbg_name,
				client_record->size, 
				client_record->address);
	} else {
		unsigned long long rem_ns, t;
		char *name;

		if(client_record->address == CLIENT_ADDRESS_TOTAL)
			name = "total";
		else if (client_record->address == CLIENT_ADDRESS_ORPHAN)
			name = "orphan";
		else
			name = "error";
		
		t = client_record->time;
		rem_ns = do_div(t, 1000000000ULL);
		seq_printf(seq, "time(%lld.%lld)\t%s %16zu\n",
				t, rem_ns, name, client_record->size);
	}
	
	return 0;	
}

static int ion_client_destory_record(void *record, void *priv)
{
	struct ion_client_record *client_record = record;

	if(client_record->address > CLIENT_ADDRESS_FLAG_MAX) {
		if(client_record->client_name)
			string_hash_put(client_record->client_name);
		if(client_record->dbg_name)
			string_hash_put(client_record->dbg_name);
	} 
		
	return 0;	
}

static int ion_client_write_record(
		struct history_record *client_history,
		const char *client_name,
		const char *dbg_name,
		size_t size, 
		void* address)
{
	struct ion_client_record *record;
	record = history_record_get_record(client_history);
	if(!record) 
		return -1;
	
	record->address = address;
	record->size = size;

	if(likely(address > CLIENT_ADDRESS_FLAG_MAX))	{
		record->client_name = string_hash_get(client_name);
		record->dbg_name = string_hash_get(dbg_name);
	} else {
		//total/orphan record: reuse name field
		record->time = local_clock();
	}
	
	history_record_put_record(client_history, record);
		
	return 0;
}

static struct history_record *g_client_history;
static struct history_record *g_buffer_history;
struct task_struct *ion_history_kthread;
#define ION_HISTORY_TIME_INTERVAL (HZ) //1s

static int write_mm_page_pool(int high, int order, int cache, size_t size)
{
	char name[50];

	snprintf(name, sizeof(name), "%smem order_%d pool", high ? "high" : "low", order);
	if(size)
		ion_client_write_record(g_client_history, name, 
			cache ? "cache" : "nocache", size, CLIENT_ADDRESS_FLAG_MAX+1);
	return 0;
}


static int ion_history_reocrd(void *data)
{
	struct ion_device *dev = g_ion_device;
	struct rb_node *n;

	while(1)
	{
		if(kthread_should_stop()) {
			IONMSG("stop ion history threak \n");
			break;
		}

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(ION_HISTORY_TIME_INTERVAL);
		if(fatal_signal_pending(current)) {
			IONMSG("ion history thread being killed\n");
			break;
		}

		//== client ==
		if(g_client_history)
		{
			down_read(&dev->lock);
			for (n = rb_first(&dev->clients); n; n = rb_next(n)) {
				struct ion_client *client = rb_entry(n, struct ion_client,
								     node);
				size_t size = 0;
				struct rb_node *nh;

				mutex_lock(&client->lock);
				for (nh = rb_first(&client->handles); nh; nh = rb_next(nh)) {
					struct ion_handle *handle = rb_entry(nh,
									     struct ion_handle,
									     node);
						size += handle->buffer->size;
				}
				mutex_unlock(&client->lock);
				
				if (!size)
					continue;
				if (client->task) {
					char task_comm[TASK_COMM_LEN];
					get_task_comm(task_comm, client->task);

					ion_client_write_record(g_client_history, task_comm,
							client->dbg_name, size, client);
				} else {
					ion_client_write_record(g_client_history, client->name,
							"kernel", size, client);
				}
			}
			up_read(&dev->lock);
		}


		if(g_client_history || g_buffer_history) {
			size_t total_size = 0;
			size_t total_orphaned_size = 0;
			
			mutex_lock(&dev->buffer_lock);
			for (n = rb_first(&dev->buffers); n; n = rb_next(n)) {
				struct ion_buffer *buffer = rb_entry(n, struct ion_buffer,
								     node);
				total_size += buffer->size;
				if (!buffer->handle_count) {
					total_orphaned_size += buffer->size;
				}

				if(g_buffer_history)
				{
					//record buffer here;
				}
				
			}
			mutex_unlock(&dev->buffer_lock);

			if(g_client_history) {
				/* record page pool info */
				ion_mm_heap_for_each_pool(write_mm_page_pool);
				
				if(total_orphaned_size)
					ion_client_write_record(g_client_history, NULL,
							NULL, total_orphaned_size, CLIENT_ADDRESS_ORPHAN);
				/* total size with time stamp */
				ion_client_write_record(g_client_history, NULL,
						NULL, total_size, CLIENT_ADDRESS_TOTAL);
			}
		}

	}
	
	return 0;
}


int ion_history_init(void)
{
	struct sched_param param = { .sched_priority = 0 };

	g_client_history = history_record_create(
			2048, 
			sizeof(struct ion_client_record), 
			ion_client_record_show,
			ion_client_destory_record,
			NULL,
			"client_history",
			g_ion_device->debug_root);

	if(IS_ERR_OR_NULL(g_client_history)) {
		IONMSG("create client history fail\n");
		return (long)g_client_history;
	}

	debugfs_create_file("string_hash", 644, g_ion_device->debug_root,
			NULL, &string_hash_debug_fops);
	
	ion_history_kthread = kthread_run(ion_history_reocrd, NULL, "%s", "ion_history");
	if (IS_ERR(ion_history_kthread)) {
		IONMSG("%s: creating thread for ion history\n", __func__);
		return PTR_RET(ion_history_kthread);
	}

	sched_setscheduler(ion_history_kthread, SCHED_IDLE, &param);

	return 0;
}




#if 0 /* test history record */

int ion_test_show(struct seq_file *seq, void *record, void *priv)
{
	seq_printf(seq, "%d\n", *(unsigned int*)record);
}


int ion_test_write(struct history_record *history_record, int data)
{
	int *record = history_record_get_record(history_record);
	*record = data;
	history_record_put_record(history_record, record);
	return 0;
}

static int debug_set(void *data, u64 val)
{
	struct history_record *history_record = data;
	int i;

	for(i=0; i<val; i++)
	{
		ion_test_write(history_record, i);
	}
	
	return 0;
}

static int debug_get(void *data, u64 *val)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_test_fops, debug_get,
			debug_set, "%llu\n");


int ion_test_history_init()
{
	struct history_record * history_record;
	int i;
	
	history_record = history_record_create(100, sizeof(int), 
			ion_test_show,
			NULL,
			NULL,
			"test",
			g_ion_device->debug_root );

	debugfs_create_file(
		"record", 0644, g_ion_device->debug_root , history_record,
		&debug_test_fops);
	return 0;
}

#endif


