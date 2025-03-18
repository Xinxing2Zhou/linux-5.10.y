Software ptp is in directory drivers/net/ethernet/vendor/gmac, 
and gmac_ptp.c is the driver.

Hardware ptp has been tested on Realtek 8211FS(I)-VS phy device, 
and the driver is in directory drivers/net/phy/realtek/.

The software and hardware ptp is determined by the interrupt pin 
from RTL8211, and the ptp function has been tested on Hi3519DV500.
In realtek_ptp.c, gpio7_2 is used as interrupt pin which is connected 
to INTB of RTL8211. Gpio7_2 is put into GPIO_IN mode, we believe 
it is hardware ptp if the voltage level of gpio7_2 is high.
