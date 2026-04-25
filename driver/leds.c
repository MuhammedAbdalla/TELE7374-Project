/*
Hardware:
P1 -> GPIO17 (interrupt)
P2 -> GPIO27 (interrupt)
L1 -> GPIO22 (PWM via hwtimer)
L2 -> GPIO23 (PWM via hwtimer)



Kernel Module:
- hwtimer for PWM (10ms period)
- interrupts for button press detection
- speed calculation (presses per 10 seconds)
- /dev/myleds for userspace communication
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("TELE7374");
MODULE_DESCRIPTION("PWM LED speed indicator");

// ----------------------------------------------------------------
// pin definitions
// global = 512 + gpio number (RPi 3B kernel 6.12)
// ----------------------------------------------------------------
#define GPIO_BASE       512

#define P1_GPIO         17          // button 1
#define P2_GPIO         27          // button 2
#define L1_GPIO         18          // LED 1 - PWM channel 0
#define L2_GPIO         19          // LED 2 - PWM channel 1

#define P1              (GPIO_BASE + P1_GPIO)
#define P2              (GPIO_BASE + P2_GPIO)

// ----------------------------------------------------------------
// hardware addresses - RPi 3B (BCM2837)
// VC bus 0x7Exxxxxx -> ARM physical 0x3Fxxxxxx
// ----------------------------------------------------------------
#define GPIO_PHYS       0x3F200000
#define PWM_PHYS        0x3F20C000
#define CLK_PHYS        0x3F101000

// GPIO function select - 3 bits per pin, 10 pins per register
// GPIO18 and GPIO19 are in GPFSEL1
// GPIO18 = bits 26-24, GPIO19 = bits 29-27
#define GPFSEL1         0x04
#define ALT5            0x2         // alt function 5 = PWM for GPIO18/19

// PWM registers
#define PWM_CTL         0x00        // control
#define PWM_RNG1        0x10        // channel 1 range (period)
#define PWM_DAT1        0x14        // channel 1 data  (duty)
#define PWM_RNG2        0x20        // channel 2 range
#define PWM_DAT2        0x24        // channel 2 data

// PWM control bits
#define PWEN1           (1 << 0)    // enable channel 1
#define MSEN1           (1 << 7)    // mark-space mode channel 1
#define PWEN2           (1 << 8)    // enable channel 2
#define MSEN2           (1 << 15)   // mark-space mode channel 2

// clock manager
#define CLK_CTL         0xA0        // PWM clock control
#define CLK_DIV         0xA4        // PWM clock divisor
#define CLK_PASSWD      0x5A000000  // must OR with all clock writes
#define CLK_OSC         1           // 19.2 MHz oscillator source
#define CLK_ENAB        (1 << 4)    // enable bit
#define CLK_BUSY        (1 << 7)    // busy bit

// PWM frequency:
// oscillator = 19.2 MHz
// divisor = 192 -> PWM clock = 100 kHz
// range   = 1000 -> period = 1000/100000 = 10ms = 100Hz
#define CLK_DIV_VAL     192
#define PWM_RANGE       1000        // 0-1000 duty cycle steps

// ----------------------------------------------------------------
// speed measurement
// ----------------------------------------------------------------
#define WINDOW_NS       10000000000ULL  // 10 second window
#define MAX_PRESS       256

// ----------------------------------------------------------------
// device
// ----------------------------------------------------------------
#define DEVICE_NAME     "myleds"
#define BUF_LEN         64

// ----------------------------------------------------------------
// state
// ----------------------------------------------------------------
static int            major;
static struct class  *cls;

static void __iomem  *gpio_mem;
static void __iomem  *pwm_mem;
static void __iomem  *clk_mem;

static int irq_p1 = -1;
static int irq_p2 = -1;

static int duty_l1 = 0;            // 0-100 percent
static int duty_l2 = 0;

static ktime_t    press_log[MAX_PRESS];
static int        press_cnt  = 0;
static int        last_btn   = 0;
static int        speed      = 0;  // presses per 10 seconds
static spinlock_t lock;

static ktime_t last_press_time = 0;
#define DEBOUNCE_NS  200000000ULL   // 200ms

// ----------------------------------------------------------------
// set GPIO alt function
// ----------------------------------------------------------------
static void set_alt(int gpio, int alt)
{
    int reg   = gpio / 10;          // GPFSEL register index
    int shift = (gpio % 10) * 3;   // bit position
    u32 val;

    val  = readl(gpio_mem + reg * 4);
    val &= ~(0x7 << shift);         // clear 3 bits
    val |=  (alt << shift);         // set alt function
    writel(val, gpio_mem + reg * 4);
}

// ----------------------------------------------------------------
// setup PWM clock
// ----------------------------------------------------------------
static void setup_clock(void)
{
    // stop clock
    writel(CLK_PASSWD | CLK_OSC, clk_mem + CLK_CTL);
    udelay(10);
    while (readl(clk_mem + CLK_CTL) & CLK_BUSY)
        udelay(1);

    // set divisor - integer part in bits 23:12
    writel(CLK_PASSWD | (CLK_DIV_VAL << 12), clk_mem + CLK_DIV);

    // start clock
    writel(CLK_PASSWD | CLK_OSC | CLK_ENAB, clk_mem + CLK_CTL);
    udelay(10);
    while (!(readl(clk_mem + CLK_CTL) & CLK_BUSY))
        udelay(1);
}

// ----------------------------------------------------------------
// setup PWM peripheral
// ----------------------------------------------------------------
static void setup_pwm(void)
{
    writel(0, pwm_mem + PWM_CTL);       // disable
    udelay(10);

    writel(PWM_RANGE, pwm_mem + PWM_RNG1);
    writel(PWM_RANGE, pwm_mem + PWM_RNG2);
    writel(0,         pwm_mem + PWM_DAT1);
    writel(0,         pwm_mem + PWM_DAT2);

    // enable both channels in mark-space mode
    writel(PWEN1 | MSEN1 | PWEN2 | MSEN2, pwm_mem + PWM_CTL);
}

// ----------------------------------------------------------------
// write duty cycle to PWM hardware
// percent = 0 to 100
// ----------------------------------------------------------------
static void write_duty(int channel, int percent)
{
    u32 dat = (percent * PWM_RANGE) / 100;

    if (channel == 1)
        writel(dat, pwm_mem + PWM_DAT1);
    else
        writel(dat, pwm_mem + PWM_DAT2);
}

// ----------------------------------------------------------------
// speed calculation - count presses in last 10 seconds
// ----------------------------------------------------------------
static void update_speed(void)
{
    ktime_t now    = ktime_get();
    ktime_t cutoff = ktime_sub_ns(now, WINDOW_NS);
    int i, count   = 0;
    unsigned long flags;

    spin_lock_irqsave(&lock, flags);
    for (i = 0; i < press_cnt; i++)
        if (ktime_compare(press_log[i], cutoff) >= 0)
            count++;
    speed = count;
    spin_unlock_irqrestore(&lock, flags);

    pr_info("myleds: speed=%d\n", speed);
}

static void record_press(int btn)
{
    unsigned long flags;
    ktime_t now = ktime_get();

    // ignore bounces within 200ms
    if (ktime_to_ns(ktime_sub(now, last_press_time)) < DEBOUNCE_NS)
        return;

    last_press_time = now;

    // only count alternating presses
    if (btn == last_btn)
        return;

    last_btn = btn;

    spin_lock_irqsave(&lock, flags);
    if (press_cnt < MAX_PRESS) {
        press_log[press_cnt++] = now;
    } else {
        int i;
        for (i = 0; i < MAX_PRESS - 1; i++)
            press_log[i] = press_log[i + 1];
        press_log[MAX_PRESS - 1] = now;
    }
    spin_unlock_irqrestore(&lock, flags);

    update_speed();
}

// ----------------------------------------------------------------
// IRQ handlers
// ----------------------------------------------------------------
static irqreturn_t p1_isr(int irq, void *data)
{
    record_press(1);
    return IRQ_HANDLED;
}

static irqreturn_t p2_isr(int irq, void *data)
{
    record_press(2);
    return IRQ_HANDLED;
}

// ----------------------------------------------------------------
// fops
// ----------------------------------------------------------------
static int dev_open(struct inode *i, struct file *f)    { return 0; }
static int dev_release(struct inode *i, struct file *f) { return 0; }

// read: "speed=N duty_l1=N duty_l2=N\n"
static ssize_t dev_read(struct file *f, char __user *buf,
                        size_t len, loff_t *off)
{
    char msg[BUF_LEN];
    int  n = snprintf(msg, BUF_LEN,
                      "speed=%d duty_l1=%d duty_l2=%d\n",
                      speed, duty_l1, duty_l2);

    if (*off >= n) { *off = 0; return 0; }
    if (len > n - *off) len = n - *off;
    if (copy_to_user(buf, msg + *off, len)) return -EFAULT;

    *off += len;
    return len;
}

// write: "duty_l1=N duty_l2=N"
static ssize_t dev_write(struct file *f, const char __user *buf,
                         size_t len, loff_t *off)
{
    char msg[BUF_LEN];
    int  n = len < BUF_LEN - 1 ? len : BUF_LEN - 1;
    int  l1, l2;

    if (copy_from_user(msg, buf, n)) return -EFAULT;
    msg[n] = '\0';
    if (n > 0 && msg[n-1] == '\n') msg[n-1] = '\0';

    if (sscanf(msg, "duty_l1=%d duty_l2=%d", &l1, &l2) == 2) {
        duty_l1 = l1 < 0 ? 0 : l1 > 100 ? 100 : l1;
        duty_l2 = l2 < 0 ? 0 : l2 > 100 ? 100 : l2;
        write_duty(1, duty_l1);
        write_duty(2, duty_l2);
        pr_info("myleds: duty_l1=%d duty_l2=%d\n", duty_l1, duty_l2);
    } else {
        pr_warn("myleds: use 'duty_l1=N duty_l2=N'\n");
        return -EINVAL;
    }

    return len;
}

static struct file_operations fops = {
    .owner   = THIS_MODULE,
    .open    = dev_open,
    .release = dev_release,
    .read    = dev_read,
    .write   = dev_write,
};

// ----------------------------------------------------------------
// init
// ----------------------------------------------------------------
static int __init leds_init(void)
{
    int ret;

    spin_lock_init(&lock);

    // character device
    major = register_chrdev(0, DEVICE_NAME, &fops);
    if (major < 0) return major;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    cls = class_create(DEVICE_NAME);
#else
    cls = class_create(THIS_MODULE, DEVICE_NAME);
#endif
    if (IS_ERR(cls)) {
        unregister_chrdev(major, DEVICE_NAME);
        return PTR_ERR(cls);
    }

    device_create(cls, NULL, MKDEV(major, 0), NULL, DEVICE_NAME);
    pr_info("myleds: /dev/%s created\n", DEVICE_NAME);

    // ioremap
    gpio_mem = ioremap(GPIO_PHYS, 0x100);
    pwm_mem  = ioremap(PWM_PHYS,  0x30);
    clk_mem  = ioremap(CLK_PHYS,  0x100);

    if (!gpio_mem || !pwm_mem || !clk_mem) {
        pr_err("myleds: ioremap failed\n");
        ret = -ENOMEM;
        goto err_unmap;
    }

    // GPIO18 and GPIO19 -> Alt5 (PWM)
    set_alt(L1_GPIO, ALT5);
    set_alt(L2_GPIO, ALT5);
    pr_info("myleds: GPIO18/19 set to Alt5 (PWM)\n");

    // start clock and PWM
    setup_clock();
    setup_pwm();
    pr_info("myleds: PWM ready (period=10ms range=%d)\n", PWM_RANGE);

    // buttons
    ret = gpio_request(P1, "p1");
    if (ret) { pr_err("myleds: P1 request failed\n"); goto err_unmap; }

    ret = gpio_request(P2, "p2");
    if (ret) { pr_err("myleds: P2 request failed\n"); goto err_p1; }

    gpio_direction_input(P1);
    gpio_direction_input(P2);

    irq_p1 = gpio_to_irq(P1);
    irq_p2 = gpio_to_irq(P2);

    ret = request_irq(irq_p1, p1_isr, IRQF_TRIGGER_FALLING, "p1", NULL);
    if (ret) { pr_err("myleds: IRQ P1 failed\n"); goto err_p2; }

    ret = request_irq(irq_p2, p2_isr, IRQF_TRIGGER_FALLING, "p2", NULL);
    if (ret) { pr_err("myleds: IRQ P2 failed\n"); goto err_irq1; }

    pr_info("myleds: buttons P1=GPIO%d P2=GPIO%d\n", P1_GPIO, P2_GPIO);
    pr_info("myleds: myleds    L1=GPIO%d L2=GPIO%d\n", L1_GPIO, L2_GPIO);
    pr_info("myleds: ready\n");
    return 0;

err_irq1:  free_irq(irq_p1, NULL);
err_p2:    gpio_free(P2);
err_p1:    gpio_free(P1);
err_unmap:
    if (gpio_mem) iounmap(gpio_mem);
    if (pwm_mem)  iounmap(pwm_mem);
    if (clk_mem)  iounmap(clk_mem);
    device_destroy(cls, MKDEV(major, 0));
    class_destroy(cls);
    unregister_chrdev(major, DEVICE_NAME);
    return ret;
}

// ----------------------------------------------------------------
// exit
// ----------------------------------------------------------------
static void __exit leds_exit(void)
{
    // stop PWM and myleds off
    writel(0, pwm_mem + PWM_CTL);
    writel(0, pwm_mem + PWM_DAT1);
    writel(0, pwm_mem + PWM_DAT2);

    free_irq(irq_p1, NULL);
    free_irq(irq_p2, NULL);
    gpio_free(P1);
    gpio_free(P2);

    iounmap(clk_mem);
    iounmap(pwm_mem);
    iounmap(gpio_mem);

    device_destroy(cls, MKDEV(major, 0));
    class_destroy(cls);
    unregister_chrdev(major, DEVICE_NAME);
    pr_info("myleds: removed\n");
}

module_init(leds_init);
module_exit(leds_exit);