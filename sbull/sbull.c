/*
 * Sample disk driver, from the beginning.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/sched.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/timer.h>
#include <linux/types.h>	/* size_t */
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/hdreg.h>	/* HDIO_GETGEO */
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>	/* invalidate_bdev */
#include <linux/bio.h>

MODULE_LICENSE("Dual BSD/GPL");

static int sbull_major = 0;
module_param(sbull_major, int, 0);
//扇区大小
static int hardsect_size = 512;
module_param(hardsect_size, int, 0);
//扇区个数
static int nsectors = 1024;	/* How big the drive is */
module_param(nsectors, int, 0);
static int ndevices = 4;
module_param(ndevices, int, 0);

/*
 * The different "request modes" we can use.
 */
enum {
	RM_SIMPLE  = 0,	/* The extra-simple request function */
	RM_FULL    = 1,	/* The full-blown version */
	RM_NOQUEUE = 2,	/* Use make_request */
};
static int request_mode = RM_SIMPLE;
module_param(request_mode, int, 0);

/*
 * Minor number and partition management.
 */
//SBULL_MINORS是每个sbull设备所支持的次设备号的数量
#define SBULL_MINORS	16
#define MINOR_SHIFT	4
#define DEVNUM(kdevnum)	(MINOR(kdev_t_to_nr(kdevnum)) >> MINOR_SHIFT

/*
 * We can tweak our hardware sector size, but the kernel talks to us
 * in terms of small sectors, always.
 */
#define KERNEL_SECTOR_SIZE	512

/*
 * After this much idle time, the driver will simulate a media change.
 */
//当应用层close掉块设备后,30s内没有再次打开,这个设备中的内容就无效了
#define INVALIDATE_DELAY	30*HZ

/*
 * The internal representation of our device.
 */
struct sbull_dev {
        int size;                       /* Device size in sectors */
        u8 *data;                       /* The data array */
        short users;                    /* How many users */
		//media是否改变的标志位
        short media_change;             /* Flag a media change? */
        spinlock_t lock;                /* For mutual exclusion */
        struct request_queue *queue;    /* The device request queue */
        struct gendisk *gd;             /* The gendisk structure */
        struct timer_list timer;        /* For simulated media changes */
};

static struct sbull_dev *Devices = NULL;

/*
 * Handle an I/O request.
 */
static void sbull_transfer(struct sbull_dev *dev, unsigned long sector,
		unsigned long nsect, char *buffer, int write)
{
	unsigned long offset = sector*KERNEL_SECTOR_SIZE;
	unsigned long nbytes = nsect*KERNEL_SECTOR_SIZE;

	if ((offset + nbytes) > dev->size) {
		printk (KERN_NOTICE "Beyond-end write (%ld %ld)\n", offset, nbytes);
		return;
	}
	if (write)
		//将sbull中的数据copy到请求队列的buffer中
		memcpy(dev->data + offset, buffer, nbytes);
	else
		//将请求队列中buffer中的数据copy到sbull中
		memcpy(buffer, dev->data + offset, nbytes);
}

/*
 * The simple form of the request function.
 */
static void sbull_request(struct request_queue *q)
{
	struct request *req;

	//从请求队列中依次取得请求
	while ((req = elv_next_request(q)) != NULL) {
		struct sbull_dev *dev = req->rq_disk->private_data;
		if (! blk_fs_request(req)) {
			//不是文件系统请求--移动数据块请求
			printk (KERN_NOTICE "Skip non-fs request\n");
			end_request(req, 0);
			continue;
		}
    //    	printk (KERN_NOTICE "Req dev %d dir %ld sec %ld, nr %d f %lx\n",
    //    			dev - Devices, rq_data_dir(req),
    //    			req->sector, req->current_nr_sectors,
    //    			req->flags);
    	//是文件系统请求
		sbull_transfer(dev, req->sector, req->current_nr_sectors,
				req->buffer, rq_data_dir(req));
		end_request(req, 1);
	}
}


/*
 * Transfer a single BIO.
 */
static int sbull_xfer_bio(struct sbull_dev *dev, struct bio *bio)
{
	int i;
	struct bio_vec *bvec;
	sector_t sector = bio->bi_sector;

	/* Do each segment independently. */
	bio_for_each_segment(bvec, bio, i) {
		char *buffer = __bio_kmap_atomic(bio, i, KM_USER0);
		sbull_transfer(dev, sector, bio_cur_sectors(bio),
				buffer, bio_data_dir(bio) == WRITE);
		sector += bio_cur_sectors(bio);
		__bio_kunmap_atomic(bio, KM_USER0);
	}
	return 0; /* Always "succeed" */
}

/*
 * Transfer a full request.
 */
static int sbull_xfer_request(struct sbull_dev *dev, struct request *req)
{
	struct bio *bio;
	int nsect = 0;
    
	__rq_for_each_bio(bio, req) {
		sbull_xfer_bio(dev, bio);
		nsect += bio->bi_size/KERNEL_SECTOR_SIZE;
	}
	return nsect;
}



/*
 * Smarter request function that "handles clustering".
 */
static void sbull_full_request(struct request_queue *q)
{
	struct request *req;
	int sectors_xferred;
	struct sbull_dev *dev = q->queuedata;

	while ((req = elv_next_request(q)) != NULL) {
		if (! blk_fs_request(req)) {
			printk (KERN_NOTICE "Skip non-fs request\n");
			end_request(req, 0);
			continue;
		}
		sectors_xferred = sbull_xfer_request(dev, req);
		if (!blk_end_request(req, 1, sectors_xferred)) {
			blkdev_dequeue_request(req);
		}
	}
}



/*
 * The direct make request version.
 */
static int sbull_make_request(struct request_queue *q, struct bio *bio)
{
	struct sbull_dev *dev = q->queuedata;
	int status;

	status = sbull_xfer_bio(dev, bio);
	bio_endio(bio, status);
	return 0;
}


/*
 * Open and close.
 */

static int sbull_open(struct block_device *bdev, fmode_t mode)
{
	//gendisk->private_data中保留的是sbull_dev指针
	struct sbull_dev *dev = bdev->bd_disk->private_data;

	//删除定时器
	del_timer_sync(&dev->timer);
	spin_lock(&dev->lock);
	if (! dev->users)
		//没有用户,即第一次打开时才会调用check_disk_change,使media有效
		check_disk_change(bdev);
	dev->users++;
	spin_unlock(&dev->lock);
	return 0;
}

//最后一个用户调用close时候,才会添加定时器
static int sbull_release(struct gendisk *disk, fmode_t mode)
{
	struct sbull_dev *dev = disk->private_data;

	spin_lock(&dev->lock);
	dev->users--;

	if (!dev->users) {
		/* 
		 * 没有用户了,添加定时器,超时的话就会添加定时器,当超过30s内没有打开这个设备,
		 * 就会执行定时器超时函数,add_timer添加的超时函数只会执行一次.
		*/
		dev->timer.expires = jiffies + INVALIDATE_DELAY;
		add_timer(&dev->timer);
	}
	spin_unlock(&dev->lock);

	return 0;
}

/*
 * Look for a (simulated) media change.
 */
int sbull_media_changed(struct gendisk *gd)
{
	struct sbull_dev *dev = gd->private_data;
	
	return dev->media_change;
}

/*
 * Revalidate.  WE DO NOT TAKE THE LOCK HERE, for fear of deadlocking
 * with open.  That needs to be reevaluated.
 */
int sbull_revalidate(struct gendisk *gd)
{
	struct sbull_dev *dev = gd->private_data;

   /*
	* 当前函数不能加上spin_lock,如果有自旋锁的话就会造成spin_lock递归调用.
	* 考虑下面的情况,当sbulla被open然后close,在INVALIDATE_DELAY过后,sbull_invalidate
	* 被调用,media_change被置位.我们接着再次open sbulla的时候check_disk_change中就会
	* 执行sbull_revalidate.注意的是open中调用check_disk_change之前已经上了锁,如果
	* sbull_revalidate中加锁,一定会造成死锁.
	*/
	//spin_lock(&dev->lock);
	if (dev->media_change) {
		//清除media_change的状态
		dev->media_change = 0;
		memset (dev->data, 0, dev->size);
	}
	//spin_unlock(&dev->lock);
	return 0;
}

/*
 * The "invalidate" function runs out of the device timer; it sets
 * a flag to simulate the removal of the media.
 */
//定时器超时函数,标志media中的数据被change了
void sbull_invalidate(unsigned long ldev)
{
	struct sbull_dev *dev = (struct sbull_dev *) ldev;

	spin_lock(&dev->lock);
	if (dev->users || !dev->data)
		//有用户,但是数据无效
		printk (KERN_WARNING "sbull: timer sanity check failed\n");
	else
		//有用户数据有效 || 无用户数据有效 || 无用户数据无效
		dev->media_change = 1;
	spin_unlock(&dev->lock);
}

/*
 * The ioctl() implementation
 */
int sbull_ioctl(struct block_device *bdev, fmode_t mode, unsigned cmd, unsigned long arg)
{
	long size;
	struct hd_geometry geo;
	struct sbull_dev *dev = bdev->bd_disk->private_data;

	switch(cmd) {
	    case HDIO_GETGEO:
        	/*
		 * Get geometry: since we are a virtual device, we have to make
		 * up something plausible.  So we claim 16 sectors, four heads,
		 * and calculate the corresponding number of cylinders.  We set the
		 * start of data at sector four.
		 */
		size = dev->size*(hardsect_size/KERNEL_SECTOR_SIZE);
		//heads * sectors = 2 ^ 6,所以cylinders须将size右移6位
		geo.cylinders = (size & ~0x3f) >> 6;
		geo.heads = 4;
		geo.sectors = 16;
		geo.start = 4;
		//将hd_geometry对象返回给fdisk使用
		if (copy_to_user((void __user *) arg, &geo, sizeof(geo)))
			return -EFAULT;
		return 0;
	}

	return -ENOTTY; /* unknown command */
}



/*
 * The device operations structure.
 */
static struct block_device_operations sbull_ops = {
	.owner           = THIS_MODULE,
	.open 	         = sbull_open,
	.release 	 = sbull_release,
	.media_changed   = sbull_media_changed,
	.revalidate_disk = sbull_revalidate,
	.ioctl	         = sbull_ioctl
};


/*
 * Set up our internal device.
 */
static void setup_device(struct sbull_dev *dev, int which)
{
	/*
	 * Get some memory.
	 */
	memset (dev, 0, sizeof (struct sbull_dev));
	//sbull是LBA地址
	dev->size = nsectors*hardsect_size;
	dev->data = vmalloc(dev->size);
	if (dev->data == NULL) {
		printk (KERN_NOTICE "vmalloc failure.\n");
		return;
	}
	spin_lock_init(&dev->lock);
	
	/*
	 * The timer which "invalidates" the device.
	 */
	init_timer(&dev->timer);
	dev->timer.data = (unsigned long) dev;
	dev->timer.function = sbull_invalidate;
	
	/*
	 * The I/O queue, depending on whether we are using our own
	 * make_request function or not.
	 */
	switch (request_mode) {
	    case RM_NOQUEUE:
		dev->queue = blk_alloc_queue(GFP_KERNEL);
		if (dev->queue == NULL)
			goto out_vfree;
		blk_queue_make_request(dev->queue, sbull_make_request);
		break;

	    case RM_FULL:
		dev->queue = blk_init_queue(sbull_full_request, &dev->lock);
		if (dev->queue == NULL)
			goto out_vfree;
		break;

	    default:
		printk(KERN_NOTICE "Bad request mode %d, using simple\n", request_mode);
        	/* fall into.. */
	
	    case RM_SIMPLE:
		//blk_init_queue中需要分配内存,故此函数可能会失败,在使用队列前一定要检查返回值
		dev->queue = blk_init_queue(sbull_request, &dev->lock);
		if (dev->queue == NULL)
			goto out_vfree;
		break;
	}
	blk_queue_hardsect_size(dev->queue, hardsect_size);
	dev->queue->queuedata = dev;
	/*
	 * And the gendisk structure.
	 */
	//每个块设备有16个次设备,如sbulla1,sbulla2 ... sbulla15
	dev->gd = alloc_disk(SBULL_MINORS);
	if (! dev->gd) {
		printk (KERN_NOTICE "alloc_disk failure\n");
		goto out_vfree;
	}
	dev->gd->major = sbull_major;
	dev->gd->first_minor = which*SBULL_MINORS;
	dev->gd->fops = &sbull_ops;
	dev->gd->queue = dev->queue;
	dev->gd->private_data = dev;
	//sbulla,sbullb...
	snprintf (dev->gd->disk_name, 32, "sbull%c", which + 'a');
	set_capacity(dev->gd, nsectors*(hardsect_size/KERNEL_SECTOR_SIZE));
	//调用add_disk后,磁盘设备将被激活
	add_disk(dev->gd);
	return;

  out_vfree:
	if (dev->data)
		vfree(dev->data);
}



static int __init sbull_init(void)
{
	int i;
	/*
	 * Get registered.
	 */
	sbull_major = register_blkdev(sbull_major, "sbull");
	if (sbull_major <= 0) {
		printk(KERN_WARNING "sbull: unable to get major number\n");
		return -EBUSY;
	}
	/*
	 * Allocate the device array, and initialize each one.
	 */
	//默认分配4个设备
	Devices = kmalloc(ndevices*sizeof (struct sbull_dev), GFP_KERNEL);
	if (Devices == NULL)
		goto out_unregister;
	for (i = 0; i < ndevices; i++) 
		setup_device(Devices + i, i);
    
	return 0;

  out_unregister:
	unregister_blkdev(sbull_major, "sbd");
	return -ENOMEM;
}

static void sbull_exit(void)
{
	int i;

	for (i = 0; i < ndevices; i++) {
		struct sbull_dev *dev = Devices + i;

		del_timer_sync(&dev->timer);
		if (dev->gd) {
			del_gendisk(dev->gd);
			put_disk(dev->gd);
		}
		if (dev->queue) {
			if (request_mode == RM_NOQUEUE)
				kobject_put(&dev->queue->kobj);
			else
				blk_cleanup_queue(dev->queue);
		}
		if (dev->data)
			vfree(dev->data);
	}
	unregister_blkdev(sbull_major, "sbull");
	kfree(Devices);
}
	
module_init(sbull_init);
module_exit(sbull_exit);
