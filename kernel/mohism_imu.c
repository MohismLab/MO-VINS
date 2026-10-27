// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/timekeeping.h>
#include "mohism_imu.h"
struct imu_device {
 struct spi_device *spi;
 struct miscdevice misc;
 DECLARE_KFIFO(fifo, struct mohism_imu_sample, 512);
 spinlock_t lock;
 wait_queue_head_t wait;
 struct mutex readers;
 struct kref refs;
 atomic_t opened;
 bool dead;
 u64 irq_ns, last_ns;
 u32 seq, pending_flags;
};
static int read_reg(struct spi_device *s, u8 r, u8 *data, size_t n)
{ r |= 0x80; return spi_write_then_read(s, &r, 1, data, n); }
static int write_reg(struct spi_device *s, u8 r, u8 v)
{ u8 b[2] = {r, v}; return spi_write(s, b, sizeof(b)); }
static void free_imu(struct kref *r)
{ kfree(container_of(r, struct imu_device, refs)); }
static irqreturn_t imu_irq(int irq, void *arg)
{
 struct imu_device *d = arg;
 d->irq_ns = ktime_get_ns();
 return IRQ_WAKE_THREAD;
}
static irqreturn_t imu_thread(int irq, void *arg)
{
 struct imu_device *d = arg;
 struct mohism_imu_sample s = {0};
 u8 raw[12], status;
 unsigned long flags;
 int i, ret;
 ret = read_reg(d->spi, 0x1e, &status, 1);
 if (ret) { d->pending_flags |= MOHISM_IMU_FLAG_SPI_ERROR; return IRQ_HANDLED; }
 s.timestamp_ns = d->irq_ns;
 s.sequence = d->seq++;
 ret = read_reg(d->spi, 0x22, raw, sizeof(raw));
 if (ret) { d->pending_flags |= MOHISM_IMU_FLAG_SPI_ERROR; return IRQ_HANDLED; }
 /* Always drain the outputs to deassert DRDY, including incomplete pairs. */
 if ((status & 3) != 3) { d->pending_flags |= MOHISM_IMU_FLAG_TIMING_GAP; return IRQ_HANDLED; }
 if (d->last_ns && s.timestamp_ns - d->last_ns > 4100000ULL)
  d->pending_flags |= MOHISM_IMU_FLAG_TIMING_GAP;
 d->last_ns = s.timestamp_ns;
 for (i = 0; i < 3; ++i) {
  s.gyro[i] = (s16)((u16)raw[i*2] | ((u16)raw[i*2+1] << 8));
  s.accel[i] = (s16)((u16)raw[6+i*2] | ((u16)raw[7+i*2] << 8));
 }
 spin_lock_irqsave(&d->lock, flags);
 if (kfifo_is_full(&d->fifo)) {
  struct mohism_imu_sample discarded;
  unsigned int count = kfifo_out(&d->fifo, &discarded, 1);
  if (count) d->pending_flags |= MOHISM_IMU_FLAG_OVERFLOW;
 }
 s.flags = d->pending_flags;
 d->pending_flags = 0;
 kfifo_in(&d->fifo, &s, 1);
 spin_unlock_irqrestore(&d->lock, flags);
 wake_up_interruptible(&d->wait);
 return IRQ_HANDLED;
}
static int imu_open(struct inode *inode, struct file *file)
{
 struct miscdevice *m = file->private_data;
 struct imu_device *d = container_of(m, struct imu_device, misc);
 if (READ_ONCE(d->dead)) return -ENODEV;
 if (atomic_cmpxchg(&d->opened, 0, 1)) return -EBUSY;
 kref_get(&d->refs);
 file->private_data = d;
 return nonseekable_open(inode, file);
}
static int imu_release(struct inode *inode, struct file *file)
{
 struct imu_device *d = file->private_data;
 atomic_set(&d->opened, 0);kref_put(&d->refs, free_imu);return 0;
}
static ssize_t imu_read(struct file *file, char __user *buf, size_t n, loff_t *pos)
{
 struct imu_device *d = file->private_data;
 struct mohism_imu_sample s;
 unsigned long flags;
 size_t done = 0;
 int ret;
 if (n < sizeof(s) || n % sizeof(s)) return -EINVAL;
 if (mutex_lock_interruptible(&d->readers)) return -ERESTARTSYS;
 while (done + sizeof(s) <= n) {
  spin_lock_irqsave(&d->lock, flags);
  ret = kfifo_out(&d->fifo, &s, 1);
  spin_unlock_irqrestore(&d->lock, flags);
  if (!ret) {
   if (done) break;
   if (READ_ONCE(d->dead)) { ret = -ENODEV; goto out; }
   if (file->f_flags & O_NONBLOCK) { ret = -EAGAIN; goto out; }
   ret = wait_event_interruptible(d->wait, !kfifo_is_empty(&d->fifo) || READ_ONCE(d->dead));
   if (ret) goto out;
   continue;
  }
  if (copy_to_user(buf + done, &s, sizeof(s))) { ret = -EFAULT; goto out; }
  done += sizeof(s);
 }
 ret = 0;
out:
 mutex_unlock(&d->readers);
 return done ? done : ret;
}
static __poll_t imu_poll(struct file *file, poll_table *wait)
{
 struct imu_device *d = file->private_data;
 __poll_t mask = 0;
 poll_wait(file, &d->wait, wait);
 if (!kfifo_is_empty(&d->fifo)) mask |= EPOLLIN | EPOLLRDNORM;
 if (READ_ONCE(d->dead)) mask |= EPOLLHUP | EPOLLERR;
 return mask;
}
static const struct file_operations imu_fops = {
 .owner = THIS_MODULE, .open = imu_open, .release = imu_release,
 .read = imu_read, .poll = imu_poll, .llseek = no_llseek,
};
static int imu_probe(struct spi_device *spi)
{
 struct imu_device *d;
 u8 who, reset;
 int ret, i;
 BUILD_BUG_ON(sizeof(struct mohism_imu_sample) != 32);
 if (spi->irq <= 0) return -EINVAL;
 spi->mode = SPI_MODE_3;spi->bits_per_word = 8;
 spi->max_speed_hz = min(spi->max_speed_hz, 1000000U);
 ret = spi_setup(spi);if (ret) return ret;
 ret = read_reg(spi, 0x0f, &who, 1);
 if (ret) return ret;
 if (who != 0x6c) return -ENODEV;
 ret = write_reg(spi, 0x12, 1);if (ret) return ret;
 for (i = 0; i < 100; ++i) {
  usleep_range(1000, 2000);
  ret = read_reg(spi, 0x12, &reset, 1);if (ret) return ret;
  if (!(reset & 1)) break;
 }
 if (i == 100) return -ETIMEDOUT;
 d = kzalloc(sizeof(*d), GFP_KERNEL);if (!d) return -ENOMEM;
 d->spi = spi;spin_lock_init(&d->lock);mutex_init(&d->readers);
 INIT_KFIFO(d->fifo);init_waitqueue_head(&d->wait);kref_init(&d->refs);atomic_set(&d->opened, 0);
 d->misc.minor = MISC_DYNAMIC_MINOR;d->misc.name = "imu0";
 d->misc.fops = &imu_fops;d->misc.parent = &spi->dev;d->misc.mode = 0660;
 spi_set_drvdata(spi, d);
 ret = write_reg(spi, 0x12, 0x44);if (ret) goto poweroff;
 ret = write_reg(spi, 0x10, 0x60);if (ret) goto poweroff;
 ret = write_reg(spi, 0x11, 0x60);if (ret) goto poweroff;
 /* Keep only timestamp capture in hard IRQ context under PREEMPT_RT.
  * NO_THREAD prevents forced threading of this explicit primary handler;
  * the SPI transfer still runs in the separately requested IRQ thread. */
 ret = request_threaded_irq(spi->irq, imu_irq, imu_thread,
   IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_THREAD, "mohism-imu", d);
 if (ret) goto poweroff;
 ret = misc_register(&d->misc);if (ret) goto free_irq;
 /* Gyro DRDY only: OR'ing accel and gyro DRDY can leave INT1 asserted
  * between asynchronous update edges. One burst drains both output banks. */
 ret = write_reg(spi, 0x0d, 0x02);if (ret) goto deregister;
 dev_info(&spi->dev, "/dev/imu0 ABI v1, 416Hz monotonic IRQ timestamps\n");
 return 0;
deregister:
 WRITE_ONCE(d->dead, true);
 misc_deregister(&d->misc);
 wake_up_interruptible(&d->wait);
free_irq:
 free_irq(spi->irq, d);
poweroff:
 write_reg(spi, 0x10, 0);write_reg(spi, 0x11, 0);
 kref_put(&d->refs, free_imu);return ret;
}
static int imu_remove(struct spi_device *spi)
{
 struct imu_device *d = spi_get_drvdata(spi);
 WRITE_ONCE(d->dead, true);
 misc_deregister(&d->misc);
 write_reg(spi, 0x0d, 0);
 free_irq(spi->irq, d);
 write_reg(spi, 0x10, 0);write_reg(spi, 0x11, 0);
 wake_up_interruptible(&d->wait);
 kref_put(&d->refs, free_imu);return 0;
}
static const struct of_device_id imu_of_match[] = {
 { .compatible = "mohism,lsm6dso-imu" }, {}
};
MODULE_DEVICE_TABLE(of, imu_of_match);
static const struct spi_device_id imu_ids[] = { {"mohism-imu", 0}, {} };
MODULE_DEVICE_TABLE(spi, imu_ids);
static struct spi_driver imu_driver = {
 .driver = { .name = "mohism-imu", .of_match_table = imu_of_match },
 .probe = imu_probe, .remove = imu_remove, .id_table = imu_ids,
};
module_spi_driver(imu_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MOHISM LSM6DSO timestamped /dev/imu0 stream");
