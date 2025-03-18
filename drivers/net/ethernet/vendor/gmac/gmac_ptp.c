#include <linux/ktime.h>
#include <linux/proc_fs.h>

#include "gmac.h"

static struct gmac_netdev_local *gmac_priv = NULL;

static int gmac_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
    return 0;
}

static int gmac_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
    struct timespec64 ts;
    struct timespec64 new_ts;
    ktime_t kt;

    kt = ktime_get_real();
    ts = ktime_to_timespec64(kt);

    new_ts.tv_sec = ts.tv_sec;
    new_ts.tv_nsec = ts.tv_nsec + delta;

    while (new_ts.tv_nsec >= 1000000000) {
        new_ts.tv_nsec -= 1000000000;
        new_ts.tv_sec++;
    }

    while (new_ts.tv_nsec < 0) {
        new_ts.tv_nsec += 1000000000;
        new_ts.tv_sec--;
    }

    kt = timespec64_to_ktime(new_ts);
    timekeeping_inject_sleeptime64(&new_ts);

    return 0;
}

static int gmac_gettime64(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
    ktime_t kt = ktime_get_real();
    *ts = ktime_to_timespec64(kt);
    return 0;
}

static int gmac_settime64(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
    return 0;
}

static int gmac_enable(struct ptp_clock_info *ptp, struct ptp_clock_request *rq, int on)
{
    return 0;
}

struct ptp_clock_info gmac_ptp_clock_ops = {
    .owner      = THIS_MODULE,
    .name       = "gmac_ptp_clock",
    .max_adj    = 1000000000, // 1 second
    .adjfine    = gmac_adjfine,
    .adjtime    = gmac_adjtime,
    .gettime64  = gmac_gettime64,
    .settime64  = gmac_settime64,
    .enable     = gmac_enable,
};

static int gmac_proc_show(struct seq_file *m, void *v)
{
    ptp_clock_unregister(gmac_priv->ptp_clock);
    gmac_priv->ptp_clock = ptp_clock_register(&gmac_priv->ptp_clock_ops,
                                         gmac_priv->dev);

    return PTR_ERR_OR_ZERO(gmac_priv->ptp_clock);
}

static int gmac_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, gmac_proc_show, PDE_DATA(inode));
}

static const struct proc_ops gmac_ptp_ops = {
    .proc_open      = gmac_proc_open,
    .proc_read      = seq_read,
    .proc_lseek     = seq_lseek,
    .proc_release   = single_release,
};

void gmac_ptp_register(struct gmac_netdev_local *priv)
{
    struct proc_dir_entry *p = NULL;
    
    spin_lock_init(&priv->ptp_lock);
    priv->ptp_clock_ops = gmac_ptp_clock_ops;

    gmac_priv = priv;

    priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops,
                                         priv->dev);
    if (IS_ERR(priv->ptp_clock)) {
        netdev_err(priv->netdev, "ptp_clock_register failed\n");
        priv->ptp_clock = NULL;
    } else if (priv->ptp_clock) {
        netdev_info(priv->netdev, "registered PTP clock\n");
    }

    p = proc_create_data("software_ptp", S_IRUGO, NULL, &gmac_ptp_ops, NULL);
    if (NULL == p) {
        netdev_err(priv->netdev, "software ptp creat failed\n");
    }
}

void gmac_ptp_unregister(struct gmac_netdev_local *priv)
{
    if (priv->ptp_clock) {
        ptp_clock_unregister(priv->ptp_clock);
        priv->ptp_clock = NULL;
        pr_debug("Removed PTP HW clock successfully on %s\n",
                 priv->netdev->name);
    }
}
