Software ptp is in directory drivers/net/ethernet/vendor/gmac, 
and gmac_ptp.c is the driver.

Hardware ptp has been tested on Realtek 8211FS(I)-VS phy device, 
and the driver is in directory drivers/net/phy/realtek/.

The software and hardware ptp is determined by the interrupt pin 
from RTL8211, and the ptp function has been tested on Hi3519DV500.
In realtek_ptp.c, gpio7_2 is used as interrupt pin which is connected 
to INTB of RTL8211. Gpio7_2 is put into GPIO_IN mode, we believe 
it is hardware ptp if the voltage level of gpio7_2 is high.

This kernel can be compiled using toolchain aarch64-v01c01-linux-gnu-gcc,
which is contained in Hi3519DV500_SDK_V2.0.1.1, or you don't need to 
compile the kernel, realtek_ptp.c can be put into any version of kernel.

Please remember download linuxptp from https://github.com/richardcochran/linuxptp.git,
then cross compile linuxptp, you can get ptp4l command, and run this command like,

./ptp4l -H -i eth0 -m -s
or
./ptp4l -S -i eth0 -m -s

-H means hardware ptp, and -S means software ptp, and -s means slave mode.

If the network is not much better, the ptp synchronization can be blocked,
you can "cat /proc/hardware_ptp" or "cat /proc/software_ptp" to reset rtl8211
in this case. However, if you don't want to reboot phy rtl8211, you have to
modify the function rtl8211f_proc_show() in realtek_ptp.c. First, remove 
rtl8211f_ptp_exit and rtl8211f_ptp_init, and put the code below into rtl8211f_proc_show(),


static int rtl8211f_proc_show(struct seq_file *m, void *v)

{

	struct rtl8211f_ptp *ptp = container_of(rtl8211f_clk_info, struct rtl8211f_ptp, caps);
        struct phy_device *phydev = ptp->phydev;

	ptp_clock_unregister(ptp->ptp_clock);
	ptp->ptp_clock = ptp_clock_register(&ptp->caps, &phydev->mdio.dev);

	return PTR_ERR_OR_ZERO(ptp->ptp_clock);

}

rtl8211f_proc_show() above won't reset rtl8211, and it will not lead to link down and up
of the network.
