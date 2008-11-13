
# Final objects
sfc_o = sfc.o
sfc_mtd_o = sfc_mtd.o

# Constituent objects
sfc_elements_o :=
sfc_elements_o += efx.o
sfc_elements_o += falcon.o
sfc_elements_o += tx.o
sfc_elements_o += rx.o
sfc_elements_o += mentormac.o
sfc_elements_o += falcon_gmac.o
sfc_elements_o += falcon_xmac.o
sfc_elements_o += alaska.o
sfc_elements_o += i2c-direct.o
sfc_elements_o += selftest.o
sfc_elements_o += driverlink.o
ifeq ($(CONFIG_SFC_DEBUGFS),y)
sfc_elements_o += debugfs.o
endif
sfc_elements_o += ethtool.o
sfc_elements_o += xfp_phy.o
sfc_elements_o += mdio_10g.o
sfc_elements_o += txc43128_phy.o
sfc_elements_o += tenxpress.o
sfc_elements_o += lm87_support.o
sfc_elements_o += boards.o
sfc_elements_o += sfe4001.o
sfc_elements_o += pm8358_phy.o
sfc_elements_o += null_phy.o
sfc_elements_o += phy.o
sfc_elements_o += kernel_compat.o

sfc_mtd_elements_o := mtd.o

obj-$(CONFIG_SFC) += $(sfc_o)
obj-$(CONFIG_SFC_MTD) += $(sfc_mtd_o)

sfc-objs = $(sfc_elements_o)
sfc_mtd-objs = $(sfc_mtd_elements_o)

obj-$(CONFIG_SFC_RESOURCE) += sfc_resource/
