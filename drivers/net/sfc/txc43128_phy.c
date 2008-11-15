/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2006-2008: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed by Solarflare Communications <linux-net-drivers@solarflare.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation, incorporated herein by reference.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 ****************************************************************************
 */
/*
 * Driver for Transwitch/Mysticom CX4 retimer
 * see www.transwitch.com, part is TXC-43128
 */

#include <linux/delay.h>
#include <linux/seq_file.h>
#include "efx.h"
#include "debugfs.h"
#include "gmii.h"
#include "mdio_10g.h"
#include "xenpack.h"
#include "phy.h"
#include "lm87_support.h"
#include "falcon.h"
#include "workarounds.h"

/* We expect these MMDs to be in the package */
#define TXC_REQUIRED_DEVS (MDIO_MMDREG_DEVS0_PCS |	\
			   MDIO_MMDREG_DEVS0_PMAPMD |	\
			   MDIO_MMDREG_DEVS0_PHYXS)

#define TXC_LOOPBACKS ((1 << LOOPBACK_PCS) |		\
		       (1 << LOOPBACK_PMAPMD) |		\
		       (1 << LOOPBACK_NETWORK))

/**************************************************************************
 *
 * Compile-time config
 *
 **************************************************************************
 */
#define TXCNAME "TXC43128"
/* Total length of time we'll wait for the PHY to come out of reset */
#define TXC_MAX_RESET_TIME 500
/* Interval between checks */
#define TXC_RESET_WAIT 10
/* How long to run BIST: At 10Gbps 50 microseconds should be plenty to get
 * some stats */
#define TXC_BIST_DURATION (50)

#define BER_INTERVAL (10 * efx_monitor_interval)

/**************************************************************************
 *
 * Register definitions
 *
 **************************************************************************
 */
#define XAUI_NUM_LANES (4)

/*** Global register bank */
/* Silicon ID register */
#define TXC_GLRGS_SLID		(0xc000)
#define TXC_GLRGS_SLID_MASK	(0x1f)

/* Command register */
#define TXC_GLRGS_GLCMD		(0xc004)
/* Useful bits in command register */
/* Lane power-down */
#define TXC_GLCMD_L01PD_LBN	(5)
#define TXC_GLCMD_L23PD_LBN	(6)
/* Limited SW reset: preserves configuration but
 * initiates a logic reset. Self-clearing */
#define TXC_GLCMD_LMTSWRST_LBN	(14)

/* Signal Quality Control */
#define TXC_GLRGS_GSGQLCTL	(0xc01a)
/* Enable bit */
#define TXC_GSGQLCT_SGQLEN_LBN	(15)
/* Lane selection */
#define TXC_GSGQLCT_LNSL_LBN	(13)
#define TXC_GSGQLCT_LNSL_WIDTH	(2)

/* Signal Quality Input */
#define TXC_GLRGS_GSGQLIN	(0xc01b)
/*  Signal Quality Grade */
#define TXC_GLRGS_GSGQLGRD	(0xc01c)
/* Drift sign */
#define TXC_GSGQLGRD_DRFTSGN_LBN (15)
/* Grade valid flag */
#define TXC_GSGQLGRD_GRDVAL_LBN	(14)
/* Remaining bits are the actual grade */
#define TXC_GSGQLGRD_GRADE_LBN	(0)
#define TXC_GSGQLGRD_GRADE_WIDTH (14)

/*  Signal Quality Drift: 16-bit drift value */
#define TXC_GLRGS_GSGQLDRFT	(0xc01d)

/**** Analog register bank */
#define TXC_ALRGS_ATXCTL	(0xc040)
/* Lane power-down */
#define TXC_ATXCTL_TXPD3_LBN	(15)
#define TXC_ATXCTL_TXPD2_LBN	(14)
#define TXC_ATXCTL_TXPD1_LBN	(13)
#define TXC_ATXCTL_TXPD0_LBN	(12)

/* Amplitude on lanes 0, 1 */
#define  TXC_ALRGS_ATXAMP0	(0xc041)
/* Amplitude on lanes 2, 3 */
#define  TXC_ALRGS_ATXAMP1	(0xc042)
/* Bit position of value for lane 0 (or 2) */
#define TXC_ATXAMP_LANE02_LBN	(3)
/* Bit position of value for lane 1 (or 3) */
#define TXC_ATXAMP_LANE13_LBN	(11)

#define TXC_ATXAMP_1280_mV	(0)
#define TXC_ATXAMP_1200_mV	(8)
#define TXC_ATXAMP_1120_mV	(12)
#define TXC_ATXAMP_1060_mV	(14)
#define TXC_ATXAMP_0820_mV	(25)
#define TXC_ATXAMP_0720_mV	(26)
#define TXC_ATXAMP_0580_mV	(27)
#define TXC_ATXAMP_0440_mV	(28)

#define TXC_ATXAMP_0820_BOTH					\
	((TXC_ATXAMP_0820_mV << TXC_ATXAMP_LANE02_LBN)		\
	 | (TXC_ATXAMP_0820_mV << TXC_ATXAMP_LANE13_LBN))

#define TXC_ATXAMP_DEFAULT	(0x6060) /* From databook */

/* Preemphasis on lanes 0, 1 */
#define  TXC_ALRGS_ATXPRE0	(0xc043)
/* Preemphasis on lanes 2, 3 */
#define  TXC_ALRGS_ATXPRE1	(0xc044)

#define TXC_ATXPRE_NONE (0)
#define TXC_ATXPRE_DEFAULT	(0x1010) /* From databook */

#define TXC_ALRGS_ARXCTL	(0xc045)
/* Lane power-down */
#define TXC_ARXCTL_RXPD3_LBN	(15)
#define TXC_ARXCTL_RXPD2_LBN	(14)
#define TXC_ARXCTL_RXPD1_LBN	(13)
#define TXC_ARXCTL_RXPD0_LBN	(12)

/*** receiver control registers: Bit Error Rate measurement */
/* Per lane BER timers */
#define TXC_RXCTL_BERTMR0	(0xc0d4)
#define TXC_RXCTL_BERTMR1	(0xc154)
#define TXC_RXCTL_BERTMR2	(0xc1d4)
#define TXC_RXCTL_BERTMR3	(0xc254)
/* Per lane BER counters */
#define TXC_RXCTL_BERCNT0	(0xc0d5)
#define TXC_RXCTL_BERCNT1	(0xc155)
#define TXC_RXCTL_BERCNT2	(0xc1d5)
#define TXC_RXCTL_BERCNT3	(0xc255)

#define BER_REG_SPACING	(TXC_RXCTL_BERTMR1 - TXC_RXCTL_BERTMR0)

/*** Main user-defined register set */
/* Main control */
#define TXC_MRGS_CTL		(0xc340)
/* Bits in main control */
#define TXC_MCTL_RESET_LBN	(15)	/* Self clear */
#define TXC_MCTL_TXLED_LBN	(14)	/* 1 to show align status */
#define TXC_MCTL_RXLED_LBN	(13)	/* 1 to show align status */

/* GPIO output */
#define TXC_GPIO_OUTPUT		(0xc346)
#define TXC_GPIO_DIR		(0xc348)

/*** Vendor-specific BIST registers */
#define TXC_BIST_CTL		(0xc280)
#define TXC_BIST_TXFRMCNT	(0xc281)
#define TXC_BIST_RX0FRMCNT	(0xc282)
#define TXC_BIST_RX1FRMCNT	(0xc283)
#define TXC_BIST_RX2FRMCNT	(0xc284)
#define TXC_BIST_RX3FRMCNT	(0xc285)
#define TXC_BIST_RX0ERRCNT	(0xc286)
#define TXC_BIST_RX1ERRCNT	(0xc287)
#define TXC_BIST_RX2ERRCNT	(0xc288)
#define TXC_BIST_RX3ERRCNT	(0xc289)

/*** BIST control bits */
/* BIST type (controls bit patter in test) */
#define TXC_BIST_CTRL_TYPE_LBN	(10)
#define	TXC_BIST_CTRL_TYPE_TSD	(0)	/* TranSwitch Deterministic */
#define TXC_BIST_CTRL_TYPE_CRP	(1)	/* CRPAT standard */
#define TXC_BIST_CTRL_TYPE_CJP	(2)	/* CJPAT standard */
#define TXC_BIST_CTRL_TYPE_TSR	(3)	/* TranSwitch pseudo-random */
/* Set this to 1 for 10 bit and 0 for 8 bit */
#define TXC_BIST_CTRL_B10EN_LBN	(12)
/* Enable BIST (write 0 to disable) */
#define	TXC_BIST_CTRL_ENAB_LBN	(13)
/*Stop BIST (self-clears when stop complete) */
#define  TXC_BIST_CTRL_STOP_LBN	(14)
/* Start BIST (cleared by writing 1 to STOP) */
#define  TXC_BIST_CTRL_STRT_LBN	(15)

/* Mt. Diablo test configuration */
#define TXC_MTDIABLO_CTRL	(0xc34f)
#define TXC_MTDIABLO_CTRL_PMA_LOOP_LBN	(10)

struct txc43128_data {
#ifdef CONFIG_SFC_DEBUGFS
	/* BER stats update from check_hw. Note that this is in errors/second,
	 * converting it to errors/bit is left as an exercise for user-space.
	 */
	unsigned phy_ber_pcs[4];
	unsigned phy_ber_phyxs[4];
#endif
	unsigned bug10934_timer;
	int phy_powered;
	int tx_disabled;
	enum efx_loopback_mode loopback_mode;
};

/* Perform the bug 10934 workaround every 5s */
#define BUG10934_RESET_INTERVAL (5 * HZ)


/* Perform a reset that doesn't clear configuration changes */
static void txc_reset_logic(struct efx_nic *efx);

/* Set the output value of a gpio */
void txc_set_gpio_val(struct efx_nic *efx, int pin, int on)
{
	int outputs;

	outputs = mdio_clause45_read(efx, efx->mii.phy_id,
					MDIO_MMD_PHYXS, TXC_GPIO_OUTPUT);

	outputs = (outputs & ~(1 << pin)) | (on << pin);

	mdio_clause45_write(efx, efx->mii.phy_id,
					MDIO_MMD_PHYXS, TXC_GPIO_OUTPUT,
					outputs);
}

/* Set up the GPIO direction register */
void txc_set_gpio_dir(struct efx_nic *efx, int pin, int dir)
{
	int dirs;

	if (efx->board_info.minor < 3 &&
		   efx->board_info.major == 0)
		return;

	dirs = mdio_clause45_read(efx, efx->mii.phy_id,
			    MDIO_MMD_PHYXS, TXC_GPIO_DIR);
	dirs = (dir & ~(1 << pin)) | (dir << pin);
	mdio_clause45_write(efx, efx->mii.phy_id,
			    MDIO_MMD_PHYXS, TXC_GPIO_DIR, dirs);

}

/* Reset the PMA/PMD MMD. The documentation is explicit that this does a
 * global reset (it's less clear what reset of other MMDs does).*/
static int txc_reset_phy(struct efx_nic *efx)
{
	int rc = mdio_clause45_reset_mmd(efx, MDIO_MMD_PMAPMD,
					 TXC_MAX_RESET_TIME / TXC_RESET_WAIT,
					 TXC_RESET_WAIT);
	if (rc < 0)
		goto fail;

	/* Check that all the MMDs we expect are present and responding. We
	 * expect faults on some if the link is down, but not on the PHY XS */
	rc = mdio_clause45_check_mmds(efx, TXC_REQUIRED_DEVS, 0);
	if (rc < 0)
		goto fail;

	return 0;

 fail:
	EFX_ERR(efx, TXCNAME ": reset timed out!\n");
	return rc;
}

/* Run a single BIST on one MMD*/
static int txc_bist_one(struct efx_nic *efx, int mmd, int test)
{
	int phy = efx->mii.phy_id;
	int ctrl, bctl;
	int lane;
	int rc = 0;

	EFX_INFO(efx, "" TXCNAME ": running BIST on %s MMD\n",
		 mdio_clause45_mmd_name(mmd));

	/* Set PMA to test into loopback using Mt Diablo reg as per app note */
	ctrl = mdio_clause45_read(efx, phy, MDIO_MMD_PCS,
				  TXC_MTDIABLO_CTRL);
	ctrl |= (1 << TXC_MTDIABLO_CTRL_PMA_LOOP_LBN);
	mdio_clause45_write(efx, phy, MDIO_MMD_PCS,
			    TXC_MTDIABLO_CTRL, ctrl);


	/* The BIST app. note lists these  as 3 distinct steps. */
	/* Set the BIST type */
	bctl = (test << TXC_BIST_CTRL_TYPE_LBN);
	mdio_clause45_write(efx, phy, mmd, TXC_BIST_CTL, bctl);

	/* Set the BSTEN bit in the BIST Control register to enable */
	bctl |= (1 << TXC_BIST_CTRL_ENAB_LBN);
	mdio_clause45_write(efx, phy, mmd, TXC_BIST_CTL, bctl);

	/* Set the BSTRT bit in the BIST Control register */
	mdio_clause45_write(efx, phy, mmd, TXC_BIST_CTL, bctl |
			    (1 << TXC_BIST_CTRL_STRT_LBN));

	/* Wait. */
	udelay(TXC_BIST_DURATION);

	/* Set the BSTOP bit in the BIST Control register */
	bctl |= (1 << TXC_BIST_CTRL_STOP_LBN);
	mdio_clause45_write(efx, phy, mmd, TXC_BIST_CTL, bctl);

	/* The STOP bit should go off when things have stopped */
	while (bctl & (1 << TXC_BIST_CTRL_STOP_LBN))
		bctl = mdio_clause45_read(efx, phy, mmd, TXC_BIST_CTL);

	/* Check all the error counts are 0 and all the frame counts are
	   non-zero */
	for (lane = 0; lane < 4; lane++) {
		int count = mdio_clause45_read(efx, phy, mmd,
					       TXC_BIST_RX0ERRCNT + lane);
		if (count != 0) {
			EFX_ERR(efx, ""TXCNAME": BIST error. "
				"Lane %d had %d errs\n", lane, count);
			rc = -EIO;
		}
		count = mdio_clause45_read(efx, phy, mmd,
					   TXC_BIST_RX0FRMCNT + lane);
		if (count == 0) {
			EFX_ERR(efx, ""TXCNAME": BIST error. "
				"Lane %d got 0 frames\n", lane);
			rc = -EIO;
		}
	}

	if (rc == 0)
		EFX_INFO(efx, ""TXCNAME": BIST pass\n");

	/* Disable BIST */
	mdio_clause45_write(efx, phy, mmd, TXC_BIST_CTL, 0);

	/* Turn off loopback */
	ctrl &= ~(1 << TXC_MTDIABLO_CTRL_PMA_LOOP_LBN);
	mdio_clause45_write(efx, phy, MDIO_MMD_PCS,
			    TXC_MTDIABLO_CTRL, ctrl);

	return rc;
}

/* Run all the desired BIST tests for the PHY */
static int txc_bist(struct efx_nic *efx)
{
	int rc;
	/*!\todo: experiment with running more of the BIST patterns to
	 * see if it actually shows up more problems. */
	rc = txc_bist_one(efx, MDIO_MMD_PCS, TXC_BIST_CTRL_TYPE_TSD);
	return rc;
}

#ifdef CONFIG_SFC_DEBUGFS

/* debugfs entries for this PHY */
static struct efx_debugfs_parameter debug_entries[] = {
	EFX_PER_LANE_PARAMETER("phy_ber_lane", "_pcs",
			       struct txc43128_data, phy_ber_pcs,
			       unsigned, efx_debugfs_read_uint),
	EFX_PER_LANE_PARAMETER("phy_ber_lane", "_phyxs",
			       struct txc43128_data, phy_ber_phyxs,
			       unsigned, efx_debugfs_read_uint),
	EFX_INT_PARAMETER(struct txc43128_data, phy_powered),
	{NULL}
};

#endif /* CONFIG_SFC_DEBUGFS */

/* Push the non-configurable defaults into the PHY. This must be
 * done after every full reset */
static void txc_apply_defaults(struct efx_nic *efx)
{
	int mctrl;

	/* Turn amplitude down and preemphasis off on the host side
	 * (PHY<->MAC) as this is believed less likely to upset Falcon
	 * and no adverse effects have been noted. It probably also
	 * saves a picowatt or two */

	/* Turn off preemphasis */
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PHYXS,
			    TXC_ALRGS_ATXPRE0, TXC_ATXPRE_NONE);
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PHYXS,
			    TXC_ALRGS_ATXPRE1, TXC_ATXPRE_NONE);

	/* Turn down the amplitude */
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PHYXS,
			    TXC_ALRGS_ATXAMP0, TXC_ATXAMP_0820_BOTH);
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PHYXS,
			    TXC_ALRGS_ATXAMP1, TXC_ATXAMP_0820_BOTH);

	/* Set the line side amplitude and preemphasis to the databook
	 * defaults as an erratum causes them to be 0 on at least some
	 * PHY rev.s */
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PMAPMD,
			    TXC_ALRGS_ATXPRE0, TXC_ATXPRE_DEFAULT);
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PMAPMD,
			    TXC_ALRGS_ATXPRE1, TXC_ATXPRE_DEFAULT);
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PMAPMD,
			    TXC_ALRGS_ATXAMP0, TXC_ATXAMP_DEFAULT);
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PMAPMD,
			    TXC_ALRGS_ATXAMP1, TXC_ATXAMP_DEFAULT);

	/* Set up the LEDs  */
	mctrl = mdio_clause45_read(efx, efx->mii.phy_id,
				   MDIO_MMD_PHYXS, TXC_MRGS_CTL);

	/* Set the Green and Red LEDs to their default modes */
	mctrl &= ~((1 << TXC_MCTL_TXLED_LBN) | (1 << TXC_MCTL_RXLED_LBN));
	mdio_clause45_write(efx, efx->mii.phy_id,
			    MDIO_MMD_PHYXS, TXC_MRGS_CTL, mctrl);

	/* Databook recommends doing this after configuration changes */
	txc_reset_logic(efx);

	efx->board_info.init_leds(efx);
}

/* Initialisation entry point for this PHY driver */
static int txc43128_phy_init(struct efx_nic *efx)
{
	u32 devid;
	int rc = 0;
	struct txc43128_data *phy_data;

	devid = mdio_clause45_read_id(efx, MDIO_MMD_PHYXS);

	phy_data = kzalloc(sizeof(*phy_data), GFP_KERNEL);
	efx->phy_data = phy_data;

	/* This is the default after reset */
	phy_data->phy_powered = efx->phy_powered;
	phy_data->tx_disabled = efx->tx_disabled;

#ifdef CONFIG_SFC_DEBUGFS
	rc = efx_extend_debugfs_port(efx, phy_data, debug_entries);
	if (rc < 0)
		goto fail1;
#endif
	EFX_INFO(efx, ""TXCNAME ": PHY ID reg %x (OUI %x model %x "
		 "revision %x)\n", devid, MDIO_ID_OUI(devid),
		 MDIO_ID_MODEL(devid), MDIO_ID_REV(devid));

	EFX_INFO(efx, ""TXCNAME ": Silicon ID %x\n",
		 mdio_clause45_read(efx, efx->mii.phy_id,
				    MDIO_MMD_PHYXS, TXC_GLRGS_SLID) &
					TXC_GLRGS_SLID_MASK);

	rc = txc_reset_phy(efx);
	if (rc < 0)
		goto fail2;

	rc = txc_bist(efx);
	if (rc < 0)
		goto fail2;

	txc_apply_defaults(efx);

	return 0;

 fail2:
#ifdef CONFIG_SFC_DEBUGFS
	efx_trim_debugfs_port(efx, debug_entries);
	/* fall-thru */
 fail1:
#endif
	kfree(efx->phy_data);
	efx->phy_data = NULL;
	return rc;
}

/* Set the lane power down state in the global registers */
static void txc_glrgs_lane_power(struct efx_nic *efx, int mmd)
{
	int pd = (1 << TXC_GLCMD_L01PD_LBN) | (1 << TXC_GLCMD_L23PD_LBN);
	int ctl = mdio_clause45_read(efx, efx->mii.phy_id,
				     mmd, TXC_GLRGS_GLCMD);

	if (efx->phy_powered)
		ctl &= ~pd;
	else
		ctl |= pd;

	mdio_clause45_write(efx, efx->mii.phy_id,
			    mmd, TXC_GLRGS_GLCMD, ctl);
}

/* Set the lane power down state in the analog control registers */
static void txc_analog_lane_power(struct efx_nic *efx, int mmd)
{
	int txpd = (1 << TXC_ATXCTL_TXPD3_LBN) | (1 << TXC_ATXCTL_TXPD2_LBN)
		| (1 << TXC_ATXCTL_TXPD1_LBN) | (1 << TXC_ATXCTL_TXPD0_LBN);

	int rxpd = (1 << TXC_ATXCTL_TXPD3_LBN) | (1 << TXC_ATXCTL_TXPD2_LBN)
		| (1 << TXC_ATXCTL_TXPD1_LBN) | (1 << TXC_ATXCTL_TXPD0_LBN);

	int txctl = mdio_clause45_read(efx, efx->mii.phy_id,
				       mmd, TXC_ALRGS_ATXCTL);
	int rxctl = mdio_clause45_read(efx, efx->mii.phy_id,
				       mmd, TXC_ALRGS_ARXCTL);

	if (efx->phy_powered) {
		txctl &= ~txpd;
		rxctl &= ~rxpd;
	} else {
		txctl |= txpd;
		rxctl |= rxpd;
	}

	mdio_clause45_write(efx, efx->mii.phy_id,
			    mmd, TXC_ALRGS_ATXCTL, txctl);
	mdio_clause45_write(efx, efx->mii.phy_id,
			    mmd, TXC_ALRGS_ARXCTL, rxctl);
}

static void txc_set_power(struct efx_nic *efx)
{
	/* According to the data book, all the MMDs can do low power */
	mdio_clause45_set_mmds_lpower(efx, !efx->phy_powered,
				      TXC_REQUIRED_DEVS);

	/* Global register bank is in PCS, PHY XS. These control the host
	 * side and line side settings respectively. */
	txc_glrgs_lane_power(efx, MDIO_MMD_PCS);
	txc_glrgs_lane_power(efx, MDIO_MMD_PHYXS);

	/* Analog register bank in PMA/PMD, PHY XS */
	txc_analog_lane_power(efx, MDIO_MMD_PMAPMD);
	txc_analog_lane_power(efx, MDIO_MMD_PHYXS);
}


static void txc_reset_logic_mmd(struct efx_nic *efx, int mmd)
{
	int portid = efx->mii.phy_id;
	int val = mdio_clause45_read(efx, portid, mmd, TXC_GLRGS_GLCMD);
	int tries = 50;
	val |= (1 << TXC_GLCMD_LMTSWRST_LBN);
	mdio_clause45_write(efx, portid, mmd, TXC_GLRGS_GLCMD, val);
	while (tries--) {
		val = mdio_clause45_read(efx, portid, mmd,
					 TXC_GLRGS_GLCMD);
		if (!(val & (1 << TXC_GLCMD_LMTSWRST_LBN)))
			break;
		udelay(1);
	}
	if (!tries)
		EFX_INFO(efx, TXCNAME " Logic reset timed out!\n");
}


/* Perform a logic reset. This preserves the configuration registers
 * and is needed for some configuration changes to take effect */
static void txc_reset_logic(struct efx_nic *efx)
{
	/* The data sheet claims we can do the logic reset on either the
	 * PCS or the PHYXS and the result is a reset of both host- and
	 * line-side logic. */
	txc_reset_logic_mmd(efx, MDIO_MMD_PCS);
}

static int txc43128_phy_read_link(struct efx_nic *efx)
{
	return mdio_clause45_links_ok(efx, TXC_REQUIRED_DEVS);
}

static void txc43128_phy_reconfigure(struct efx_nic *efx)
{
	struct txc43128_data *phy_data = efx->phy_data;
	int power_change = (efx->phy_powered != phy_data->phy_powered);
	int loop_change = LOOPBACK_CHANGED(phy_data, efx, TXC_LOOPBACKS);
	int disable_change = (efx->tx_disabled != phy_data->tx_disabled);

	if (!phy_data->tx_disabled && efx->tx_disabled) {
		txc_reset_phy(efx);
		txc_apply_defaults(efx);
		falcon_reset_xaui(efx);
		disable_change = 0;
	}

	mdio_clause45_transmit_disable(efx, efx->tx_disabled);
	mdio_clause45_phy_reconfigure(efx);
	if (power_change)
		txc_set_power(efx);

	/* The data sheet claims this is required after every reconfiguration
	 * (note at end of 7.1), but we mustn't do it when nothing changes as
	 * it glitches the link, and reconfigure gets called on link change,
	 * so we get an IRQ storm on link up. */
	if (loop_change || power_change || disable_change)
		txc_reset_logic(efx);

	phy_data->phy_powered = efx->phy_powered;
	phy_data->loopback_mode = efx->loopback_mode;
	phy_data->tx_disabled = efx->tx_disabled;
	efx->link_up = txc43128_phy_read_link(efx);
	efx->link_options = GM_LPA_10000FULL;
}

static void txc43128_phy_fini(struct efx_nic *efx)
{
	efx->board_info.blink(efx, 0);

	/* Disable link events */
	xenpack_disable_lasi_irqs(efx);

#ifdef CONFIG_SFC_DEBUGFS
	/* Remove the extra debug entries and free data */
	efx_trim_debugfs_port(efx, debug_entries);
#endif
	kfree(efx->phy_data);
	efx->phy_data = NULL;
}

/* Periodic callback: this exists mainly to poll link status as we currently
 * don't use LASI interrupts. Also update the BER counters and poll the lm87 */
static int txc43128_phy_check_hw(struct efx_nic *efx)
{
	struct txc43128_data *data = efx->phy_data;
#ifdef CONFIG_SFC_DEBUGFS
	int phy = efx->mii.phy_id;
	int timer, count, i, mmd;
#endif
	int rc = 0;
	int link_up = txc43128_phy_read_link(efx);

	/* Simulate a PHY event if link state has changed */
	if (link_up != efx->link_up) {
		efx->link_up = link_up;
		efx->mac_op->fake_phy_event(efx);
	} else if (EFX_WORKAROUND_10934(efx)) {
		if (link_up || (efx->loopback_mode != LOOPBACK_NONE))
			data->bug10934_timer = jiffies;
		else {
			int delta = jiffies - data->bug10934_timer;
			if (delta >= BUG10934_RESET_INTERVAL) {
				data->bug10934_timer = jiffies;
				txc_reset_logic(efx);
			}
		}
	}

	rc = efx->board_info.monitor(efx);
	if (rc) {
		EFX_ERR(efx, "" TXCNAME
			": sensor alert! Putting PHY into low power.\n");
		efx->phy_powered = 0;
		txc_set_power(efx);
	}

#ifdef CONFIG_SFC_DEBUGFS
	/* There are 2 MMDs with RX BER counters: PCS and PHY XS,
	 * which happen to be consecutively numbered */
	for (mmd = MDIO_MMD_PCS; mmd <= MDIO_MMD_PHYXS; mmd++) {
		for (i = 0; i < XAUI_NUM_LANES; i++) {
			timer = mdio_clause45_read(efx, phy, mmd,
						   TXC_RXCTL_BERTMR0 +
						   i * BER_REG_SPACING);
			count = mdio_clause45_read(efx, phy, mmd,
						   TXC_RXCTL_BERCNT0 +
						   i * BER_REG_SPACING);
			/* The BER timer counts down in seconds. If it would
			 * expire before the next check_hw, update the stats &
			 * restart the timer (clears the count) */
			if (timer * HZ < efx_monitor_interval) {
				/* Record count, allowing for the fact that the
				 * timer may not have reached zero */
				unsigned ber = (count * BER_INTERVAL) /
					(BER_INTERVAL - timer * HZ);
				if (mmd == MDIO_MMD_PCS)
					data->phy_ber_pcs[i] = ber;
				else
					data->phy_ber_phyxs[i] = ber;
				/* Reprogram the timer */
				mdio_clause45_write(efx, phy, mmd,
						    TXC_RXCTL_BERTMR0 +
						    i * BER_REG_SPACING,
						    BER_INTERVAL / HZ);
			}
		}
		mmd = (mmd == MDIO_MMD_PCS) ? MDIO_MMD_PHYXS : 0;
	}
#endif /* CONFIG_SFC_DEBUGFS */
	return rc;
}

struct efx_phy_operations falcon_txc_phy_ops = {
	.init             = txc43128_phy_init,
	.reconfigure      = txc43128_phy_reconfigure,
	.check_hw         = txc43128_phy_check_hw,
	.fini             = txc43128_phy_fini,
	.clear_interrupt  = efx_port_dummy_op_void,
	.reset_xaui       = efx_port_dummy_op_void,
	.mmds             = TXC_REQUIRED_DEVS,
	.loopbacks        = TXC_LOOPBACKS,
	.startup_loopback = LOOPBACK_PMAPMD,
};
