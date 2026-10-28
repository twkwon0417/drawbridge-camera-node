#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/ktime.h>
#include <linux/uaccess.h>

MODULE_LICENSE("GPL");

#define GPIO_UP         12
#define GPIO_DOWN        5
#define DEBOUNCE_NS     (30 * NSEC_PER_MSEC)

static int irq_up, irq_down;
static ktime_t last_irq_up, last_irq_down;
static int ready;

static DECLARE_KFIFO(sw_fifo, char, 16);
static DECLARE_WAIT_QUEUE_HEAD(sw_wq);

static irqreturn_t up_isr(int irq, void *dev_id)
{
	ktime_t now = ktime_get();

	if (!ready)
		return IRQ_HANDLED;
	if (ktime_to_ns(ktime_sub(now, last_irq_up)) < DEBOUNCE_NS)
		return IRQ_HANDLED;
	last_irq_up = now;

	kfifo_put(&sw_fifo, 'U');
	wake_up_interruptible(&sw_wq);
	printk(KERN_INFO "manual_switch: MANUAL_UP 눌림 (GPIO %d)\n", GPIO_UP);
	return IRQ_HANDLED;
}

static irqreturn_t down_isr(int irq, void *dev_id)
{
	ktime_t now = ktime_get();

	if (!ready)
		return IRQ_HANDLED;
	if (ktime_to_ns(ktime_sub(now, last_irq_down)) < DEBOUNCE_NS)
		return IRQ_HANDLED;
	last_irq_down = now;

	kfifo_put(&sw_fifo, 'D');
	wake_up_interruptible(&sw_wq);
	printk(KERN_INFO "manual_switch: MANUAL_DOWN 눌림 (GPIO %d)\n", GPIO_DOWN);
	return IRQ_HANDLED;
}

static ssize_t sw_read(struct file *f, char __user *buf, size_t count, loff_t *pos)
{
	char c;
	int ret;

	if (count < 1)
		return -EINVAL;

	ret = wait_event_interruptible(sw_wq, !kfifo_is_empty(&sw_fifo));
	if (ret)
		return ret;

	if (!kfifo_get(&sw_fifo, &c))
		return 0;

	if (copy_to_user(buf, &c, 1))
		return -EFAULT;

	return 1;
}

static unsigned int sw_poll(struct file *f, poll_table *wait)
{
	poll_wait(f, &sw_wq, wait);
	return kfifo_is_empty(&sw_fifo) ? 0 : (POLLIN | POLLRDNORM);
}

static const struct file_operations sw_fops = {
	.owner = THIS_MODULE,
	.read  = sw_read,
	.poll  = sw_poll,
};

static struct miscdevice sw_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "manual_sw",
	.fops  = &sw_fops,
	.mode  = 0666,
};

static int __init manual_switch_init(void)
{
	int ret;

	INIT_KFIFO(sw_fifo);
	last_irq_up   = ktime_set(0, 0);
	last_irq_down = ktime_set(0, 0);

	printk(KERN_INFO "manual_switch: 초기화 중 GPIO_UP=%d GPIO_DOWN=%d\n",
	       GPIO_UP, GPIO_DOWN);

	ret = gpio_request_one(GPIO_UP, GPIOF_IN, "manual_up");
	if (ret) {
		printk(KERN_ERR "manual_switch: GPIO %d 요청 실패 (%d)\n", GPIO_UP, ret);
		return ret;
	}

	ret = gpio_request_one(GPIO_DOWN, GPIOF_IN, "manual_down");
	if (ret) {
		printk(KERN_ERR "manual_switch: GPIO %d 요청 실패 (%d)\n", GPIO_DOWN, ret);
		goto err_free_up;
	}

	irq_up = gpio_to_irq(GPIO_UP);
	if (irq_up < 0) {
		printk(KERN_ERR "manual_switch: gpio_to_irq(%d) 실패\n", GPIO_UP);
		ret = irq_up;
		goto err_free_down;
	}

	irq_down = gpio_to_irq(GPIO_DOWN);
	if (irq_down < 0) {
		printk(KERN_ERR "manual_switch: gpio_to_irq(%d) 실패\n", GPIO_DOWN);
		ret = irq_down;
		goto err_free_down;
	}

	ret = request_irq(irq_up, up_isr, IRQF_TRIGGER_RISING, "manual_up", NULL);
	if (ret) {
		printk(KERN_ERR "manual_switch: UP IRQ 등록 실패 (%d)\n", ret);
		goto err_free_down;
	}

	ret = request_irq(irq_down, down_isr, IRQF_TRIGGER_RISING, "manual_down", NULL);
	if (ret) {
		printk(KERN_ERR "manual_switch: DOWN IRQ 등록 실패 (%d)\n", ret);
		goto err_free_irq_up;
	}

	ret = misc_register(&sw_misc);
	if (ret) {
		printk(KERN_ERR "manual_switch: misc_register 실패 (%d)\n", ret);
		goto err_free_irqs;
	}

	kfifo_reset(&sw_fifo);
	smp_wmb();
	ready = 1;

	printk(KERN_INFO "manual_switch: 준비 완료 — /dev/manual_sw  (UP=GPIO%d DOWN=GPIO%d)\n",
	       GPIO_UP, GPIO_DOWN);
	return 0;

err_free_irqs:
	free_irq(irq_down, NULL);
err_free_irq_up:
	free_irq(irq_up, NULL);
err_free_down:
	gpio_free(GPIO_DOWN);
err_free_up:
	gpio_free(GPIO_UP);
	return ret;
}

static void __exit manual_switch_exit(void)
{
	ready = 0;
	misc_deregister(&sw_misc);
	free_irq(irq_down, NULL);
	free_irq(irq_up,   NULL);
	gpio_free(GPIO_DOWN);
	gpio_free(GPIO_UP);
	printk(KERN_INFO "manual_switch: 종료\n");
}

module_init(manual_switch_init);
module_exit(manual_switch_exit);
