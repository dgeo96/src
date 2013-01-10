/*
 * Ethernet driver for the NPX LPC22XX
 *
 *  Copyright (C) 2007 Siemens Bulding Technologies (mailto:philippe.goetz@siemens.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <asm/arch/board.h>

#include "lpc22xx_eth.h"

// XXX - Ivan 
//#define DEBUG_ETH	1
#undef DEBUG_ETH


#define DRV_NAME	"lpc22xx-eth"
#define DRV_VERSION	"1.0"

#define LINK_POLL_INTERVAL	(HZ)

/* DMA Memory organization - ETH_RAM_BASE (0x7FE00000) ETH_RAM_SIZE (0x4000) */
#define ETH_BLOCK_SIZE         0x600
#define ETH_TX_DESCRIPTOR_NUM  4
#define ETH_RX_DESCRIPTOR_NUM  4
/* descriptors and status are placed at the end of the emac address space */
#define RX_STATUS_SIZE         8
#define RX_STATUS_ADDR         (ETH_RAM_BASE + ETH_RAM_SIZE - (RX_STATUS_SIZE * ETH_RX_DESCRIPTOR_NUM))
#define RX_DESCRIPTOR_SIZE     8
#define RX_DESCRIPTOR_ADDR    (RX_STATUS_ADDR - (RX_DESCRIPTOR_SIZE * ETH_RX_DESCRIPTOR_NUM))
#define TX_STATUS_SIZE         4
#define TX_STATUS_ADDR         (RX_DESCRIPTOR_ADDR - (TX_STATUS_SIZE * ETH_TX_DESCRIPTOR_NUM))
#define TX_DESCRIPTOR_SIZE     8
#define TX_DESCRIPTOR_ADDR    (TX_STATUS_ADDR - (TX_DESCRIPTOR_SIZE * ETH_TX_DESCRIPTOR_NUM))

/* ........................... PHY INTERFACE ........................... */

/*
 * Enable the MDIO bit in MAC control register
 * When not called from an interrupt-handler, access to the PHY must be protected by a spinlock.
 */
static void enable_mdi(void)
{
}

/*
 * Disable the MDIO bit in the MAC control register
 */
static void disable_mdi(void)
{
}

/*
 * Initialize the access to the PHY
 */
static void init_phy(void) {
  /* MII Mgmt. Configuration Register */ 
  REG_ETH_MCFG = 0x0018; /* [4:2]=0b110 CLOCK SELECT -> Host Clock divided by 20. */
}

/*
 * Wait until the PHY operation is complete.
 */
static inline void wait_phy(void) {
	unsigned long timeout = jiffies + 2;

	while ((REG_ETH_MIND & 0x1)!=0) {
		if (time_after(jiffies, timeout)) {
			printk("lpc22xx_eth: MIO timeout\n");
			break;
		}
		cpu_relax();
	}
}

/*
 * Write value to the a PHY register
 * Note: MDI interface is assumed to already have been enabled.
 */
static void write_phy(unsigned char phy_addr, unsigned char address, unsigned int value)
{
//printk(KERN_INFO "%s(%d, %d, %d)\n", __FUNCTION__, phy_addr, address, value);
	REG_ETH_MCMD = 0;
	REG_ETH_MADR = ((phy_addr & 0x1f) << 8) | address;
	REG_ETH_MWTD = value;
	/* Wait until BUSY bit in MMI Mgmt Indicators Register (MIND) is cleared */
	wait_phy();
}

/*
 * Read value stored in a PHY register.
 * Note: MDI interface is assumed to already have been enabled.
 */
static void read_phy(unsigned char phy_addr, unsigned char address, unsigned int *value)
{
//printk(KERN_INFO "%s(%d, %d)", __FUNCTION__, phy_addr, address);
	REG_ETH_MCMD = 1;
	REG_ETH_MADR = ((phy_addr & 0x1f) << 8) | address;
	/* Wait until BUSY bit in MMI Mgmt Indicators Register (MIND) is cleared */
	wait_phy();
	REG_ETH_MCMD = 0;
	*value = (unsigned int)REG_ETH_MRDD;
//printk("=%u\n", *value);
}

/* ........................... PHY MANAGEMENT .......................... */

/*
 * Reset the PHY
 */
static void reset_phy(struct net_device *dev)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	unsigned int bmcr;

	spin_lock_irq(&lp->lock);
	enable_mdi();

	/* Perform PHY reset */
	write_phy(lp->phy_address, MII_BMCR, BMCR_RESET);

	/* Wait until PHY reset is complete */
	do {
		read_phy(lp->phy_address, MII_BMCR, &bmcr);
	} while (!(bmcr && BMCR_RESET));

	disable_mdi();
	spin_unlock_irq(&lp->lock);
}

/*
 * Access the PHY to determine the current link speed and mode, and update the
 * MAC accordingly.
 * If no link or auto-negotiation is busy, then no changes are made.
 */
static void update_linkspeed(struct net_device *dev, int silent)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	unsigned int bmsr, bmcr, lpa;
	unsigned int speed, duplex;

	if (!mii_link_ok(&lp->mii)) {		/* no link */
		netif_carrier_off(dev);
		if (!silent)
			printk(KERN_INFO "%s: Link down.\n", dev->name);
		return;
	}

	/* Link up, or auto-negotiation still in progress */
	read_phy(lp->phy_address, MII_BMSR, &bmsr);
	read_phy(lp->phy_address, MII_BMCR, &bmcr);
	if (bmcr & BMCR_ANENABLE) {				/* AutoNegotiation is enabled */
		if (!(bmsr & BMSR_ANEGCOMPLETE))
			return;			/* Do nothing - another interrupt generated when negotiation complete */

		read_phy(lp->phy_address, MII_LPA, &lpa);
		if ((lpa & LPA_100FULL) || (lpa & LPA_100HALF)) speed = SPEED_100;
		else speed = SPEED_10;
		if ((lpa & LPA_100FULL) || (lpa & LPA_10FULL)) duplex = DUPLEX_FULL;
		else duplex = DUPLEX_HALF;
	} else {
		speed = (bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10;
		duplex = (bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
	}

	/* Update the MAC */
	REG_ETH_COMMAND |= 0x0040;
	if (speed == SPEED_100) {
		REG_ETH_SUPP |= 0x100;
		if (duplex == DUPLEX_FULL) {		/* 100 Full Duplex */
			REG_ETH_MAC2 = 0x31;
			REG_ETH_COMMAND |= 0x0400;
			REG_ETH_IPGT = 0x15;
		}
		else {					/* 100 Half Duplex */
			REG_ETH_MAC2 = 0x30;
			REG_ETH_COMMAND &= ~0x0400;
			REG_ETH_IPGT = 0x12;
		}
	} else {
		REG_ETH_SUPP &= ~0x100;
		if (duplex == DUPLEX_FULL) {		/* 10 Full Duplex */
			REG_ETH_MAC2 = 0x31;
			REG_ETH_COMMAND |= 0x0400;
			REG_ETH_IPGT = 0x15;
		}
		else {					/* 10 Half Duplex */
			REG_ETH_MAC2 = 0x30;
			REG_ETH_COMMAND &= ~0x0400;
			REG_ETH_IPGT = 0x12;
		}
	}

	if (!silent)
		printk(KERN_INFO "%s: Link now %i-%s\n", dev->name, speed, (duplex == DUPLEX_FULL) ? "FullDuplex" : "HalfDuplex");
	netif_carrier_on(dev);
}

/*
 * Handle interrupts from the PHY
 */
static irqreturn_t lpc22xx_eth_phy_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct lpc22xx_private *lp = netdev_priv(dev);
	unsigned int phy;

	/*
	 * This hander is triggered on both edges, but the PHY chips expect
	 * level-triggering.  We therefore have to check if the PHY actually has
	 * an IRQ pending.
	 */
	enable_mdi();
	// MII_MICREL PHY */
	read_phy(lp->phy_address, MII_TPISTATUS, &phy);		/* ack interrupt in Micrel PHY */
	if (!(phy & ((1 << 2) | 1)))
		goto done;

	update_linkspeed(dev, 0);

done:
	disable_mdi();

	return IRQ_HANDLED;
}

/*
 * Initialize and enable the PHY interrupt for link-state changes
 */
static void enable_phyirq(struct net_device *dev)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	unsigned int dsintr, irq_number;
	int status;

	irq_number = 0; // lp->board_data.phy_irq_pin;
	if (!irq_number) {
		/*
		 * PHY doesn't have an IRQ pin (RTL8201, DP83847, AC101L),
		 * or board does not have it connected.
		 */
		mod_timer(&lp->check_timer, jiffies + LINK_POLL_INTERVAL);
		return;
	}

	status = request_irq(irq_number, lpc22xx_eth_phy_interrupt, 0, dev->name, dev);
	if (status) {
		printk(KERN_ERR "lpc22xx_eth: PHY IRQ %d request failed - status %d!\n", irq_number, status);
		return;
	}
/*
	spin_lock_irq(&lp->lock);
	enable_mdi();

	// MII_MICREL PHY
	dsintr = (1 << 10) | ( 1 << 8);
	write_phy(lp->phy_address, MII_TPISTATUS, dsintr);

	disable_mdi();
	spin_unlock_irq(&lp->lock);
*/
}

/*
 * Disable the PHY interrupt
 */
static void disable_phyirq(struct net_device *dev)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	unsigned int dsintr;
	unsigned int irq_number;

	irq_number = 0; // lp->board_data.phy_irq_pin;
	if (!irq_number) {
		del_timer_sync(&lp->check_timer);
		return;
	}

/*
	spin_lock_irq(&lp->lock);
	enable_mdi();

	// MII_MICREL PHY
	read_phy(lp->phy_address, MII_TPISTATUS, &dsintr);
	dsintr = ~((1 << 10) | (1 << 8));
	write_phy(lp->phy_address, MII_TPISTATUS, dsintr);

	disable_mdi();
	spin_unlock_irq(&lp->lock);
*/
	free_irq(irq_number, dev);			/* Free interrupt handler */
}

static void lpc22xx_eth_check_link(unsigned long dev_id)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct lpc22xx_private *lp = netdev_priv(dev);

	spin_lock_irq(&lp->lock);
	enable_mdi();
	update_linkspeed(dev, 1);
	disable_mdi();
	spin_unlock_irq(&lp->lock);

	mod_timer(&lp->check_timer, jiffies + LINK_POLL_INTERVAL);
}

/* ......................... ADDRESS MANAGEMENT ........................ */

static char chToUpper(char ch)
{
  if((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9'))
  {
    return ch;
  }
  else
  {
    return ch - ('a'-'A');
  }   
}

static int strToInt(const char* pStr, int len, int base)
{
  //                      0,1,2,3,4,5,6,7,8,9,:,;,<,=,>,?,@,A ,B ,C ,D ,E ,F
  static const int v[] = {0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,0,10,11,12,13,14,15}; 
  int i   = 0;
  int val = 0;
  int dec = 1;
  int idx = 0; 

  for(i = len; i > 0; i--)
  {
    idx = chToUpper(pStr[i-1]) - '0';    

    if(idx > sizeof(v)/sizeof(int))
    {
      printk("strToInt: illegal character %c\n", pStr[i-1]);
      continue;
    }
    
    val += (v[idx]) * dec;
    dec *= base;
  }

  return val; 
}

/*
 * Set the ethernet MAC address in dev->dev_addr
 */
static void __init get_mac_address(struct net_device *dev)
{

	dev->dev_addr[0] = 0x00;	
	dev->dev_addr[1] = 0x1A;
	dev->dev_addr[2] = 0xF1;
	dev->dev_addr[3] = 0x00;
	dev->dev_addr[4] = 0x00;
	dev->dev_addr[5] = 0xF6;
{
  unsigned char *pEnv    = (unsigned char *)0x7C004;
  unsigned char *pEndEnv = (unsigned char *)0x7FFF0;
  unsigned char  found   = 0;
  while((0 == found) && ((unsigned long)pEnv < (unsigned long)pEndEnv))
  {
    if (0 == strncmp(pEnv, "ethaddr=", 8))
    {
printk("%s %s\n", __FUNCTION__, pEnv);
      dev->dev_addr[0] = strToInt(pEnv+8, 2, 16);
      dev->dev_addr[1] = strToInt(pEnv+11, 2, 16);
      dev->dev_addr[2] = strToInt(pEnv+14, 2, 16);
      dev->dev_addr[3] = strToInt(pEnv+17, 2, 16);
      dev->dev_addr[4] = strToInt(pEnv+20, 2, 16);
      dev->dev_addr[5] = strToInt(pEnv+23, 2, 16);
      found = 1;
    }
    //move to next string
    if (0 == found)
    {
      while(((unsigned long)pEnv < (unsigned long)pEndEnv) && (*pEnv != '\0'))
        pEnv++;
      pEnv++;
    }
  }
}
/*
	printk("MAC address got %02x:%02x:%02x:%02x:%02x:%02x\n",
		dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
		dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);
*/
}

/*
 * Program the hardware MAC address from dev->dev_addr.
 */
static void update_mac_address(struct net_device *dev)
{
	REG_ETH_SA0 = (dev->dev_addr[5] << 8 | dev->dev_addr[4]);
	REG_ETH_SA1 = (dev->dev_addr[3] << 8 | dev->dev_addr[2]);
	REG_ETH_SA2 = (dev->dev_addr[1] << 8 | dev->dev_addr[0]);
}

/*
 * Store the new hardware address in dev->dev_addr, and update the MAC.
 */
static int set_mac_address(struct net_device *dev, void* addr)
{
	struct sockaddr *address = addr;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	update_mac_address(dev);

	printk("%s: Setting MAC address to %02x:%02x:%02x:%02x:%02x:%02x\n", dev->name,
		dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
		dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	return 0;
}

// TODO...

/* ......................... ETHTOOL SUPPORT ........................... */

static int mdio_read(struct net_device *dev, int phy_id, int location)
{
	unsigned int value;

	read_phy(phy_id, location, &value);
	return value;
}

static void mdio_write(struct net_device *dev, int phy_id, int location, int value)
{
	write_phy(phy_id, location, value);
}

static int lpc22xx_eth_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	int ret;

printk(KERN_INFO "%s\n", __FUNCTION__);
	spin_lock_irq(&lp->lock);
	enable_mdi();

	ret = mii_ethtool_gset(&lp->mii, cmd);

	disable_mdi();
	spin_unlock_irq(&lp->lock);

	return ret;
}

static int lpc22xx_eth_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	int ret;

printk(KERN_INFO "%s\n", __FUNCTION__);
	spin_lock_irq(&lp->lock);
	enable_mdi();

	ret = mii_ethtool_sset(&lp->mii, cmd);

	disable_mdi();
	spin_unlock_irq(&lp->lock);

	return ret;
}

static int lpc22xx_eth_nwayreset(struct net_device *dev)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	int ret;

printk(KERN_INFO "%s\n", __FUNCTION__);
	spin_lock_irq(&lp->lock);
	enable_mdi();

	ret = mii_nway_restart(&lp->mii);

	disable_mdi();
	spin_unlock_irq(&lp->lock);

	return ret;
}

static void lpc22xx_eth_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, dev->dev.parent->bus_id, sizeof(info->bus_info));
}

static const struct ethtool_ops lpc22xx_eth_ethtool_ops = {
	.get_settings	= lpc22xx_eth_get_settings,
	.set_settings	= lpc22xx_eth_set_settings,
	.get_drvinfo	= lpc22xx_eth_get_drvinfo,
	.nway_reset	= lpc22xx_eth_nwayreset,
	.get_link	= ethtool_op_get_link,
};

static int lpc22xx_eth_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	int res;

printk(KERN_INFO "%s\n", __FUNCTION__);
	if (!netif_running(dev))
		return -EINVAL;

	spin_lock_irq(&lp->lock);
	enable_mdi();
	res = generic_mii_ioctl(&lp->mii, if_mii(rq), cmd, NULL);
	disable_mdi();
	spin_unlock_irq(&lp->lock);

	return res;
}

/* ................................ MAC ................................ */

/*
 * Open the ethernet interface
 */
static int lpc22xx_eth_open(struct net_device *dev)
{
	struct lpc22xx_private *lp = netdev_priv(dev);
	unsigned long ctl;

	/* Update the MAC address (incase user has changed it) */
	update_mac_address(dev);

	/* Enable PHY interrupt */
	enable_phyirq(dev);

	/* Enable MAC interrupts */
  	REG_ETH_INTCLEAR = 0xFFFF;
  	REG_ETH_INTENABLE = 0x00FF; /*not SoftInt and WoL*/

	/* Determine current link speed */
	spin_lock_irq(&lp->lock);
	enable_mdi();
	update_linkspeed(dev, 0);
	disable_mdi();
	spin_unlock_irq(&lp->lock);

	/* Enable Receiver and Transmitter */
  	REG_ETH_COMMAND |= 3; // (KREG_ETH_COMMAND_RX_ENABLE | KREG_ETH_COMMAND_TX_ENABLE);
	REG_ETH_MAC1 |= 1;

	netif_start_queue(dev);

	return 0;
}

static int lpc22xx_eth_close(struct net_device *dev)
{
	/* Disable Receiver and Transmitter */
 	REG_ETH_COMMAND &= ~3; // (KREG_ETH_COMMAND_RX_ENABLE | KREG_ETH_COMMAND_TX_ENABLE);
	REG_ETH_MAC1 &= ~1;

	/* Disable PHY interrupt */
	disable_phyirq(dev);

	netif_stop_queue(dev);

	return 0;
}

static int lpc22xx_eth_tx(struct sk_buff *skb, struct net_device *dev)
{
  struct lpc22xx_private *lp = netdev_priv(dev);

  u32 txProduceIndex = 0;
  u32 txConsumeIndex = 0;
  u8* pData          = 0;
  u32 len            = skb->len;
  u32 sendLen        = 0;
  u32* tx_desc_addr   = 0;

  spin_lock_irq(&lp->lock);

  txProduceIndex = REG_ETH_TXPRODUCEINDEX;
  txConsumeIndex = REG_ETH_TXCONSUMEINDEX;

  if (txConsumeIndex != txProduceIndex)
  {
    // TODO why return here? This just means that the transmit array isn't empty
    printk("%s: emac_tx transmit array isn't empty\n", __FUNCTION__);
    spin_unlock_irq(&lp->lock);
    return -1;
  }

  if (txProduceIndex == ETH_TX_DESCRIPTOR_NUM)
  {
    // should never happen
    // TODO remove
    printk("%s: emac_tx produce index == count\n", __FUNCTION__);
    spin_unlock_irq(&lp->lock);
    return -1;
  }

  if (len > 0)
  {
    // TODO pData must be copied to MAC_TX_BUFFER_ADDR
    pData = skb->data;

    do
    {
      tx_desc_addr = (u32*) (TX_DESCRIPTOR_ADDR + txProduceIndex * TX_DESCRIPTOR_SIZE);

      sendLen = len;
      if (sendLen > ETH_BLOCK_SIZE)
      {
        sendLen = ETH_BLOCK_SIZE;
      }
      else
      {
        // last fragment
        sendLen |= KREG_ETH_TXCONTROL_LAST;
      }

      *tx_desc_addr = (u32)pData;
      *(tx_desc_addr+1) = (u32)(KREG_ETH_TXCONTROL_INT | (sendLen -1));

      txProduceIndex++;
      if (txProduceIndex == ETH_TX_DESCRIPTOR_NUM)
      {
        txProduceIndex = 0;
      }

      REG_ETH_TXPRODUCEINDEX = txProduceIndex;

      len   -= (sendLen & ~KREG_ETH_TXCONTROL_LAST);
      pData += (sendLen & ~KREG_ETH_TXCONTROL_LAST);

    } while (len > 0);

    dev->trans_start = jiffies;

    lp->skb = skb;

    // stop upper layer from sending more packages until we are done
    netif_stop_queue(dev);

  }

  spin_unlock_irq(&lp->lock);

  return 0;
}

static struct net_device_stats *lpc22xx_eth_stats(struct net_device *dev)
{
  struct lpc22xx_private *lp = netdev_priv(dev);

  return &lp->stats;
}

static void lpc22xx_eth_rx(struct net_device *dev)
{
  struct lpc22xx_private *lp = netdev_priv(dev);
  struct sk_buff *skb;

  u32 rxProduceIndex = 0;
  u32 rxConsumeIndex = 0;
  volatile unsigned long* rxStatusAddr  = 0;
  u32 statusInfo     = 0;
  volatile unsigned long* recvAddr      = 0;
  u32 recvSize       = 0;
  u32 i = 0;

//printk("%s %s\n", __FUNCTION__, dev->name);
  spin_lock(&lp->lock);

  rxProduceIndex = REG_ETH_RXPRODUCEINDEX;
  rxConsumeIndex = REG_ETH_RXCONSUMEINDEX;
/*
printk("%s START rxConsumeIndex=%d %p status=%08lx rxProduceIndex=%d %p status=%08lx\n", __FUNCTION__, rxConsumeIndex, (volatile unsigned long*)(RX_STATUS_ADDR + rxConsumeIndex * RX_STATUS_SIZE), *(volatile unsigned long*)(RX_STATUS_ADDR + rxConsumeIndex * RX_STATUS_SIZE), rxProduceIndex, (volatile unsigned long*)(RX_STATUS_ADDR + rxProduceIndex * RX_STATUS_SIZE), *(volatile unsigned long*)(RX_STATUS_ADDR + rxProduceIndex * RX_STATUS_SIZE));
*/
  // consume the received packets
  while (rxConsumeIndex != rxProduceIndex)
  {
    // Evaluate the recvSize
    i = rxConsumeIndex; recvSize = 0; statusInfo = 0;
    while(i!=rxProduceIndex) {
      rxStatusAddr = (volatile unsigned long*)(RX_STATUS_ADDR + i * RX_STATUS_SIZE);
      statusInfo = *rxStatusAddr;
      if(statusInfo==0) {
	unsigned long addr;
        printk("LOST SYNC at %d %p=%08lx",i,rxStatusAddr,statusInfo); 
        for(addr = TX_DESCRIPTOR_ADDR; addr<(ETH_RAM_BASE+ETH_RAM_SIZE); addr+=4) {
          if((addr%16)==0) printk("\n%p", (volatile unsigned long *)addr);
          printk(" %08lX", *(volatile unsigned long *)addr);
        }
        printk("\n");
      }
      recvSize += (statusInfo & KREG_ETH_RXSTATUS_SIZE_MASK) + 1;
      if ((statusInfo & KREG_ETH_RXSTATUS_LAST) != 0) break;
      i++;
      if (i == ETH_RX_DESCRIPTOR_NUM)
      {
        i = 0;
      }
    }
    if ((statusInfo & KREG_ETH_RXSTATUS_LAST) == 0) {
      i = rxProduceIndex + 1;
      if (i == ETH_RX_DESCRIPTOR_NUM) i = 0;
      if(i == rxConsumeIndex) {
      printk("%s ERROR OVERRRUN rxConsumeIndex=%d rxProduceIndex=%d i=%d\n", __FUNCTION__, rxConsumeIndex, rxProduceIndex, i);
        rxConsumeIndex = rxProduceIndex;
      }
      break;
    }
//printk("%s rxConsumeIndex=%u lastIndex=%u rxProduceIndex=%u recvSize=%u\n", __FUNCTION__, rxConsumeIndex, i, rxProduceIndex, recvSize);
    // Allocate the skb
    skb = dev_alloc_skb(recvSize + 2); 
    if (skb == NULL)
    {
      printk("%s: dev_alloc_failed\n", __FUNCTION__);
      lp->stats.rx_dropped++;
    }
    else
    {
      skb->dev = dev;
      skb_reserve(skb, 2);  /* Force 16 byte alignment. Ethernet
                   frame header is 14 bytes, so we
                   add two extra bytes in the headers */
    }
    // Copy packets in the skb
    while(rxConsumeIndex!=rxProduceIndex) {
      rxStatusAddr = (volatile unsigned long*)(RX_STATUS_ADDR + rxConsumeIndex * RX_STATUS_SIZE);
      statusInfo = *rxStatusAddr;
      recvSize = (statusInfo & KREG_ETH_RXSTATUS_SIZE_MASK) + 1;
      recvAddr = (volatile unsigned long*)(RX_DESCRIPTOR_ADDR + rxConsumeIndex * RX_DESCRIPTOR_SIZE);
      if(skb != NULL) {
        memcpy(skb_put(skb, recvSize), (void *)*recvAddr, recvSize);
      }
      *rxStatusAddr = 0;
      rxConsumeIndex++;
      if (rxConsumeIndex == ETH_RX_DESCRIPTOR_NUM)
      {
        rxConsumeIndex = 0;
      }
      if ((statusInfo & KREG_ETH_RXSTATUS_LAST) != 0) break;
    }
    /* stats rx_packets */
    lp->stats.rx_packets++;
    if(skb != NULL) {
/*
printk("recv %08lx [%ld]=%ld %p %p", statusInfo, lp->stats.rx_packets, skb->len, recvAddr, (void*)*recvAddr);
for(i=0; i<skb->len; i++) {
  if((i%16)==0) printk("\n");
  if((i%16)==8) printk(" ");
  printk("%02X", skb->data[i]);
}
printk("\n");
*/
      skb->protocol = eth_type_trans(skb, dev);
      /* stats rx_bytes */
      lp->stats.rx_bytes += skb->len;
      netif_rx(skb);
      dev->last_rx = jiffies;
    }
    REG_ETH_RXCONSUMEINDEX = rxConsumeIndex;
    rxProduceIndex = REG_ETH_RXPRODUCEINDEX;
  }

  spin_unlock(&lp->lock);

}

/*
 * MAC interrupt handler
 */
static irqreturn_t lpc22xx_eth_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *) dev_id;
	struct lpc22xx_private *lp = netdev_priv(dev);
	unsigned long intstatus, ctl;

	/* MAC Interrupt Status register indicates what interrupts are pending.
	   It is automatically cleared once read. */
	intstatus = REG_ETH_INTSTATUS;

	do {
	  if (intstatus & KREG_ETH_INT_RXOVERRUN)
    	  {
      	     REG_ETH_INTCLEAR = KREG_ETH_INT_RXOVERRUN;
             lp->stats.rx_over_errors++;
	     break;
          }

          if (intstatus & KREG_ETH_INT_RXERROR)
          {
      	     REG_ETH_INTCLEAR = KREG_ETH_INT_RXERROR;
//             lp->stats.rx_errors++;
             break;   
          }

          if (intstatus & KREG_ETH_INT_RXFINISHED)
          {
      	     REG_ETH_INTCLEAR = KREG_ETH_INT_RXFINISHED;
//             lp->stats.rx_errors++;   
//           while ( REG_ETH_RXPRODUCEINDEX != (REG_ETH_RXCONSUMEINDEX - 1) );
          }

          if (intstatus & KREG_ETH_INT_RXDONE)
          {
      	     REG_ETH_INTCLEAR = KREG_ETH_INT_RXDONE;
             lpc22xx_eth_rx(dev);   
          }

          if (intstatus & KREG_ETH_INT_TXUNDERRUN)
          {
      	     REG_ETH_INTCLEAR = KREG_ETH_INT_TXUNDERRUN;
//             lp->stats.tx_errors++;
             break;  
          }

          if (intstatus & KREG_ETH_INT_TXERROR)
          {
      	     REG_ETH_INTCLEAR = KREG_ETH_INT_TXERROR;
//             lp->stats.tx_errors++;
             break;  
          }

          if (intstatus & KREG_ETH_INT_TXFINISHED)
          {
      	     REG_ETH_INTCLEAR = KREG_ETH_INT_TXFINISHED;
          }

          if (intstatus & KREG_ETH_INT_TXDONE)
          {
      	     REG_ETH_INTCLEAR = KREG_ETH_INT_TXDONE;
  //spin_lock(&lock);

  /* stats tx_packets */
  lp->stats.tx_packets++;

  if (lp->skb == NULL)
  {
    printk("lpc22_eth_tx: lp->skb is NULL\n");
  }
  else
  {
    /* stats tx_bytes */
    lp->stats.tx_bytes += lp->skb->len;
    dev_kfree_skb_irq(lp->skb);
    lp->skb = NULL;
  }

  if (netif_queue_stopped(dev))
  {
    netif_wake_queue(dev);
  }

  //spin_unlock(&lock);
          }
	  
	} while(0);

	return IRQ_HANDLED;
}

/*
 * Initialize and start the Receiver and Transmit subsystems
 */
static void init_dma(struct net_device *dev)
{
	int i;
	volatile unsigned long* addr;
	unsigned long buffer_addr = ETH_RAM_BASE;

	memset((void *)ETH_RAM_BASE, 0, ETH_RAM_SIZE);
	/* Initialize the TX descriptors and status */
	REG_ETH_TXDESCRIPTOR = TX_DESCRIPTOR_ADDR;
	REG_ETH_TXDESCRIPTORNUMBER = ETH_TX_DESCRIPTOR_NUM - 1;
/* TX_DESCRIPTOR is directly set in main memory - see lpc22xx_eth_tx.
	for(addr = (volatile unsigned long*)TX_DESCRIPTOR_ADDR, i=0; i < ETH_TX_DESCRIPTOR_NUM; i++) {
	    // TX packet  - base address of the buffer containing the data
    	    *addr++ = buffer_addr; buffer_addr += ETH_BLOCK_SIZE;
	    // TX control field in descriptor
    	    *addr++ = (KREG_ETH_TXCONTROL_INT | ((ETH_BLOCK_SIZE-1) & KREG_ETH_TXCONTROL_SIZE_MASK));
	}
*/
	REG_ETH_TXSTATUS = TX_STATUS_ADDR;
	for(addr = (volatile unsigned long*)TX_STATUS_ADDR, i=0; i < ETH_TX_DESCRIPTOR_NUM; i++) {
    	    *addr++ = 0;
	}
	REG_ETH_TXPRODUCEINDEX = 0;

	/* Initialize the RX descriptors and status */
	REG_ETH_RXDESCRIPTOR = RX_DESCRIPTOR_ADDR;
	REG_ETH_RXDESCRIPTORNUMBER = ETH_RX_DESCRIPTOR_NUM - 1;
	for(addr = (volatile unsigned long*)RX_DESCRIPTOR_ADDR, i=0; i < ETH_RX_DESCRIPTOR_NUM; i++) {
	    // RX packet  - base address of the buffer containing the data
    	    *addr++ = buffer_addr; buffer_addr += ETH_BLOCK_SIZE;
	    // RX control field in descriptor
    	    *addr++ = (KREG_ETH_RXCONTROL_INT | ((ETH_BLOCK_SIZE-1) & KREG_ETH_RXCONTROL_SIZE_MASK));
	}
	REG_ETH_RXSTATUS = RX_STATUS_ADDR;
	for(addr = (volatile unsigned long*)RX_STATUS_ADDR, i=0; i < ETH_RX_DESCRIPTOR_NUM; i++) {
    	    *addr++ = 0;
    	    *addr++ = 0;
	}
	REG_ETH_RXCONSUMEINDEX = 0;

}

/*
 * Initialize the ethernet interface
 */
static int __init lpc22xx_eth_setup(unsigned long phy_type, unsigned short phy_address,
			struct platform_device *pdev)
{
	struct lpc22xx_eth_data *board_data = pdev->dev.platform_data;
	struct net_device *dev;
	struct lpc22xx_private *lp;
	int res;

printk(KERN_INFO "%s \n", __FUNCTION__);
	dev = alloc_etherdev(sizeof(struct lpc22xx_private));
	if (!dev)
		return -ENOMEM;

	dev->base_addr = APH_ETH_BASE;
	dev->irq = LPC22xx_INTERRUPT_ETH;
	/*SET_MODULE_OWNER(dev);*/

	/* Install the interrupt handler */
	if (request_irq(dev->irq, lpc22xx_eth_interrupt, 0, dev->name, dev)) {
		free_netdev(dev);
		return -EBUSY;
	}

	lp = netdev_priv(dev);
	lp->board_data = *board_data;
	platform_set_drvdata(pdev, dev);

	spin_lock_init(&lp->lock);

	ether_setup(dev);
	dev->open = lpc22xx_eth_open;
	dev->stop = lpc22xx_eth_close;
	dev->hard_start_xmit = lpc22xx_eth_tx;
	dev->get_stats = lpc22xx_eth_stats;
	//dev->set_multicast_list = lpc22xx_eth_set_rx_mode;
	dev->set_mac_address = set_mac_address;
	dev->ethtool_ops = &lpc22xx_eth_ethtool_ops;
	dev->do_ioctl = lpc22xx_eth_ioctl;
#ifdef CONFIG_NET_POLL_CONTROLLER
	//dev->poll_controller = lpc22xx_eth_poll_controller;
#endif

	SET_NETDEV_DEV(dev, &pdev->dev);

	get_mac_address(dev);		/* Get ethernet address and store it in dev->dev_addr */
	update_mac_address(dev);	/* Program ethernet address into MAC */

	lp->mii.dev = dev;		/* Support for ethtool */
	lp->mii.mdio_read = mdio_read;
	lp->mii.mdio_write = mdio_write;
	lp->mii.phy_id = phy_address;
	lp->mii.phy_id_mask = 0x1f;
	lp->mii.reg_num_mask = 0x1f;

	lp->phy_type = phy_type;	/* Type of PHY connected */
	lp->phy_address = phy_address;	/* MDI address of PHY */

	/* Register the network interface */
	res = register_netdev(dev);
	if (res) {
		free_irq(dev->irq, dev);
		free_netdev(dev);
		return res;
	}

	/* Determine current link speed */
	spin_lock_irq(&lp->lock);
	enable_mdi();
	reset_phy(dev);
	write_phy(lp->phy_address, MII_BMCR, (BMCR_ANENABLE | BMCR_ANRESTART));
	update_linkspeed(dev, 0);
	disable_mdi();
	spin_unlock_irq(&lp->lock);
	netif_carrier_off(dev);		/* will be enabled in open() */

	/* If board has no PHY IRQ, use a timer to poll the PHY */
	if (1) {
		init_timer(&lp->check_timer);
		lp->check_timer.data = (unsigned long)dev;
		lp->check_timer.function = lpc22xx_eth_check_link;
	}

	init_dma(dev);

	// pass all receive frames
	REG_ETH_MAC1 |= 0x0002;

	// set up the Rx filter 
	// [0]-AllUnicast, [1]-AllBroadCast, [2]-AllMulticast, [3]-UnicastHash
	// [4]-MulticastHash, [5]-Perfect, [12]-MagicPacketEnWoL, [13]-RxFilterEnWoL
	REG_ETH_RXFILTERCTRL =  0x0022;

	// clear all interrupts
	REG_ETH_INTCLEAR = 0xFFFF;

	/* Display ethernet banner */
	printk(KERN_INFO "%s: LPC22XX ethernet at 0x%08x int=%d %s%s (%02x:%02x:%02x:%02x:%02x:%02x)\n",
		dev->name, (uint) dev->base_addr, dev->irq,
		REG_ETH_SUPP & 0x80 ? "100-" : "10-",
		REG_ETH_MAC2 & 0x01 ? "FullDuplex" : "HalfDuplex",
		dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
		dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);
	switch (phy_type & 0xFFFFFFF0L) {
	case MII_MICREL21_ID: printk(KERN_INFO "%s: Micrel PHY at %d\n", dev->name, phy_address); break;
	//case MII_BROADCOMxx_ID: printk(KERN_INFO "%s: Broadcom PHY at %d\n", dev->name, phy_address); break;
	}

	return 0;
}

/*
 * Detect MAC and PHY and perform initialization
 * NOTE: The gpio shall be setup before probing the Ethernet.
 * RMII => P1.0=1 ,P1.1=1 ,P1.4=1 ,P1.6=1 ,P1.8=1 ,P1.9=1 ,P1.10=1, P1.14=1
 *         P1.15=1 , P1.16=1, P1.17=1
 * NOTE: ENET_TX_CLK (P1.6=1) must be enabled even not connected.
 */
static int __init lpc22xx_eth_probe(struct platform_device *pdev)
{
	unsigned int phyid1, phyid2;
	int detected = -1;
	unsigned long mac_id;
	unsigned long phy_id;
	unsigned short phy_address = 0;

printk(KERN_INFO "%s \n", __FUNCTION__);
/* Check MAC module ID
 XXX - Ivan
 the MAC modules on different chips have their own IDs, so mask out the 
 offending part	
*/
 
	mac_id = REG_ETH_MODULEID;
	if ((mac_id&KREG_ETH_MODULEID_MASK) != (KREG_ETH_MODULEID_NXP&KREG_ETH_MODULEID_MASK))
  	{
    		printk("lpc22xx_eth: module ID check failed (%lx != %lx)\n", mac_id, KREG_ETH_MODULEID_NXP);
		return -ENODEV;
  	}

#if defined(DEBUG_ETH)	
	printk("lpc22xx_eth: module ID check succeeded (mac_id= %lx)\n", mac_id);
#endif

// Reset and initialize the MAC
	// MAC configuration register 1
	// [8]=1 RESET TX; [9]=1 RESET MCS / TX; [10]=1 RESET RX; [11]=1 RESET MCS / RX
	// [14]=1 SIMULATION RESET; [15]=1 SOFT RESET
  	REG_ETH_MAC1 = 0xCF00; 
		#if defined(DEBUG_ETH)	
			printk("lpc22xx_eth: REG_ETH_MAC1 = 0xCF00; succeeded\n");
		#endif
  	// Command Register
	// [3]=1 RegReset; [4]=1 TxReset; [5]=1 RxReset
  	REG_ETH_COMMAND = 0x0038;
		#if defined(DEBUG_ETH)	
			printk("lpc22xx_eth: REG_ETH_COMMAND = 0x0038; succeeded\n");
		#endif
  	// Short delay after reset
		udelay(10);	
		#if defined(DEBUG_ETH)	
			printk("lpc22xx_eth: udelay(10); succeeded\n");
		#endif
	// MAC configuration register 1
  	REG_ETH_MAC1 = 0;
		#if defined(DEBUG_ETH)	
			printk("lpc22xx_eth: REG_ETH_MAC1 = 0; succeeded\n");
		#endif

#ifndef CONFIG_LPC22XX_ETH_MMI
	// Command Register
	REG_ETH_COMMAND |= 0x0200; /* [9]=1 RMII */
	#if defined(DEBUG_ETH)
		printk("lpc22xx_eth: RMII mode selected");
	#endif
#else
	#if defined(DEBUG_ETH)
		printk("lpc22xx_eth: MII mode selected");
	#endif
#endif

	REG_ETH_SUPP = 0x0900;  /* [8]=1 SPEED -> 0=10 Mbps mode, 1=100 Mbps mode */
	udelay(50);
	REG_ETH_SUPP = 0x0100; /* [8]=1 SPEED -> 0=10 Mbps mode, 1=100 Mbps mode */

	REG_ETH_IPGR = 0x0C12;
	REG_ETH_CLRT = 0x370F;

	init_phy();

	#if defined(DEBUG_ETH)
	printk("lpc22xx_eth: phy_init: detected: %d, phy_address: %d\n", detected , phy_address);
	#endif

	while ((detected != 0) && (phy_address < 32)) {
		/* Read the PHY ID registers */
		read_phy(phy_address, MII_PHYSID1, &phyid1);
		read_phy(phy_address, MII_PHYSID2, &phyid2);
		phy_id = (phyid1 << 16) | phyid2;
		#if defined(DEBUG_ETH) 
			printk(KERN_INFO "%s phy_address=%d phy_id=0x%08lx\n", __FUNCTION__, phy_address, phy_id);
		#endif
		switch (phy_id & 0xFFFFFFF0L) {
		case MII_MICREL21_ID: return lpc22xx_eth_setup(phy_id, phy_address, pdev);
		//case MII_BROADCOMxx_ID: return lpc22xx_eth_setup(phy_id, phy_address, pdev);
		}
		phy_address++;
	}

	return -ENODEV;
}


static int lpc22xx_eth_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

printk(KERN_INFO "%s \n", __FUNCTION__);
	unregister_netdev(dev);
	free_irq(dev->irq, dev); // LPC22xx_INTERRUPT_ETH
	//			 // LPC22xx_INTERRUPT_EINT1
	free_netdev(dev);
	return 0;
}

/* TODO Power Management */
#ifdef CONFIG_PM
#else
#define lpc22xx_eth_suspend	NULL
#define lpc22xx_eth_resume	NULL
#endif
static struct platform_driver lpc22xx_eth_driver = {
	.probe		= lpc22xx_eth_probe,
	.remove		= lpc22xx_eth_remove,
	.suspend	= lpc22xx_eth_suspend,
	.resume		= lpc22xx_eth_resume,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init lpc22xx_eth_init(void)
{
	return platform_driver_register(&lpc22xx_eth_driver);
}

static void __exit lpc22xx_eth_exit(void)
{
	platform_driver_unregister(&lpc22xx_eth_driver);
}

module_init(lpc22xx_eth_init)
module_exit(lpc22xx_eth_exit)

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LPC22XX EMAC Ethernet driver");
MODULE_AUTHOR("Philippe Goetz");
