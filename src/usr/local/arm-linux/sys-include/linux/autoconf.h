/*
 * Automatically generated C config: don't edit
 */
#define AUTOCONF_INCLUDED
#define CONFIG_ARM 1
#undef  CONFIG_EISA
#undef  CONFIG_SBUS
#undef  CONFIG_MCA
#define CONFIG_UID16 1
#define CONFIG_RWSEM_GENERIC_SPINLOCK 1
#undef  CONFIG_RWSEM_XCHGADD_ALGORITHM
#undef  CONFIG_GENERIC_BUST_SPINLOCK
#undef  CONFIG_GENERIC_ISA_DMA

/*
 * Code maturity level options
 */
#define CONFIG_EXPERIMENTAL 1
#undef  CONFIG_ADVANCED_OPTIONS
#undef  CONFIG_OBSOLETE

/*
 * Loadable module support
 */
#define CONFIG_MODULES 1
#undef  CONFIG_MODVERSIONS
#undef  CONFIG_KMOD

/*
 * System Type
 */
#undef  CONFIG_ARCH_ADIFCC
#undef  CONFIG_ARCH_ANAKIN
#undef  CONFIG_ARCH_ARCA5K
#undef  CONFIG_ARCH_CLPS7500
#undef  CONFIG_ARCH_CLPS711X
#undef  CONFIG_ARCH_CO285
#undef  CONFIG_ARCH_EBSA110
#undef  CONFIG_ARCH_CAMELOT
#undef  CONFIG_ARCH_FOOTBRIDGE
#undef  CONFIG_ARCH_INTEGRATOR
#undef  CONFIG_ARCH_KS8695
#undef  CONFIG_ARCH_IOP3XX
#undef  CONFIG_ARCH_IXP1200
#undef  CONFIG_ARCH_IXP2000
#define CONFIG_ARCH_IXP425 1
#undef  CONFIG_ARCH_OMAHA
#undef  CONFIG_ARCH_L7200
#undef  CONFIG_ARCH_MX1ADS
#undef  CONFIG_ARCH_EP93XX
#undef  CONFIG_ARCH_RPC
#undef  CONFIG_ARCH_RISCSTATION
#undef  CONFIG_ARCH_SA1100
#undef  CONFIG_ARCH_SHARK
#undef  CONFIG_ARCH_AT91RM9200

/*
 * Archimedes/A5000 Implementations
 */

/*
 * Archimedes/A5000 Implementations (select only ONE)
 */
#undef  CONFIG_ARCH_ARC
#undef  CONFIG_ARCH_A5K

/*
 * Footbridge Implementations
 */
#undef  CONFIG_ARCH_CATS
#undef  CONFIG_ARCH_PERSONAL_SERVER
#undef  CONFIG_ARCH_EBSA285_ADDIN
#undef  CONFIG_ARCH_EBSA285_HOST
#undef  CONFIG_ARCH_NETWINDER

/*
 * SA11x0 Implementations
 */
#undef  CONFIG_SA1100_ACCELENT
#undef  CONFIG_SA1100_ASSABET
#undef  CONFIG_ASSABET_NEPONSET
#undef  CONFIG_SA1100_ADSAGC
#undef  CONFIG_SA1100_ADSBITSY
#undef  CONFIG_SA1100_ADSBITSYPLUS
#undef  CONFIG_SA1100_BRUTUS
#undef  CONFIG_SA1100_CEP
#undef  CONFIG_SA1100_CERF
#undef  CONFIG_SA1100_H3100
#undef  CONFIG_SA1100_H3600
#undef  CONFIG_SA1100_H3800
#undef  CONFIG_SA1100_H3XXX
#undef  CONFIG_H3600_SLEEVE
#undef  CONFIG_SA1100_EXTENEX1
#undef  CONFIG_SA1100_FLEXANET
#undef  CONFIG_SA1100_FREEBIRD
#undef  CONFIG_SA1100_FRODO
#undef  CONFIG_SA1100_GRAPHICSCLIENT
#undef  CONFIG_SA1100_GRAPHICSMASTER
#undef  CONFIG_SA1100_HACKKIT
#undef  CONFIG_SA1100_BADGE4
#undef  CONFIG_SA1100_JORNADA720
#undef  CONFIG_SA1100_HUW_WEBPANEL
#undef  CONFIG_SA1100_ITSY
#undef  CONFIG_SA1100_LART
#undef  CONFIG_SA1100_NANOENGINE
#undef  CONFIG_SA1100_OMNIMETER
#undef  CONFIG_SA1100_PANGOLIN
#undef  CONFIG_SA1100_PLEB
#undef  CONFIG_SA1100_PT_SYSTEM3
#undef  CONFIG_SA1100_SHANNON
#undef  CONFIG_SA1100_SHERMAN
#undef  CONFIG_SA1100_SIMPAD
#undef  CONFIG_SA1100_SIMPUTER
#undef  CONFIG_SA1100_PFS168
#undef  CONFIG_SA1100_VICTOR
#undef  CONFIG_SA1100_XP860
#undef  CONFIG_SA1100_YOPY
#undef  CONFIG_SA1100_USB
#undef  CONFIG_SA1100_USB_NETLINK
#undef  CONFIG_SA1100_USB_CHAR
#undef  CONFIG_SA1100_SSP

/*
 * IXP4xx Implementation Options
 */
#define CONFIG_ARCH_IXDP425 1
#undef  CONFIG_MACH_IXDP465
#undef  CONFIG_MACH_MONTEJADE
#undef  CONFIG_ARCH_IXCDP1100
#undef  CONFIG_ARCH_PRPMC1100
#undef  CONFIG_ARCH_ADI_COYOTE
#undef  CONFIG_ARCH_SE4000
#undef  CONFIG_MACH_SG560
#undef  CONFIG_MACH_SG565
#undef  CONFIG_MACH_SHIVA1100
#undef  CONFIG_MACH_SG580
#undef  CONFIG_MACH_ESS710
#undef  CONFIG_MACH_IVPN
#undef  CONFIG_MACH_SE5100

/*
 * AT91RM9200 Implementations
 */
#undef  CONFIG_ARCH_AT91RM9200DK
#undef  CONFIG_MACH_CSB337

/*
 * CLPS711X/EP721X Implementations
 */
#undef  CONFIG_ARCH_AUTCPU12
#undef  CONFIG_ARCH_CDB89712
#undef  CONFIG_ARCH_CLEP7312
#undef  CONFIG_ARCH_EDB7211
#undef  CONFIG_ARCH_EDB7312
#undef  CONFIG_ARCH_FORTUNET
#undef  CONFIG_ARCH_GUIDEA07
#undef  CONFIG_ARCH_P720T
#undef  CONFIG_ARCH_EP7211
#undef  CONFIG_ARCH_EP7212
#undef  CONFIG_ARCH_EP7312
#undef  CONFIG_ARCH_EP9301
#undef  CONFIG_ARCH_EP9302
#undef  CONFIG_ARCH_EP9312
#undef  CONFIG_ARCH_EP9315
#undef  CONFIG_ARCH_EDB9301
#undef  CONFIG_ARCH_EDB9302
#undef  CONFIG_ARCH_EDB9312
#undef  CONFIG_ARCH_EDB9315
#undef  CONFIG_ARCH_ACORN
#undef  CONFIG_FOOTBRIDGE
#undef  CONFIG_FOOTBRIDGE_HOST
#undef  CONFIG_FOOTBRIDGE_ADDIN
#undef  CONFIG_EP93XX_GRAPHICS
#undef  CONFIG_EP93XX_CRUNCH

/*
 * Processor Type
 */
#define CONFIG_CPU_32 1
#undef  CONFIG_CPU_26
#undef  CONFIG_CPU_ARM610
#undef  CONFIG_CPU_ARM710
#undef  CONFIG_CPU_ARM720T
#undef  CONFIG_CPU_ARM920T
#undef  CONFIG_CPU_ARM922T
#undef  CONFIG_PLD
#undef  CONFIG_CPU_ARM926T
#undef  CONFIG_CPU_ARM1020
#undef  CONFIG_CPU_ARM1026
#undef  CONFIG_CPU_SA110
#undef  CONFIG_CPU_SA1100
#undef  CONFIG_CPU_32v3
#undef  CONFIG_CPU_32v4
#define CONFIG_CPU_32v5 1
#define CONFIG_CPU_XSCALE 1
#define CONFIG_ARM_THUMB 1

/*
 * Processor Features
 */
#undef  CONFIG_XSCALE_PMU_TIMER
#undef  CONFIG_XSCALE_CACHE_ERRATA
#undef  CONFIG_XSCALE_BDI2000
#undef  CONFIG_ARM_FASS
#undef  CONFIG_DISCONTIGMEM
#define CONFIG_CPU_BIG_ENDIAN 1

/*
 * General setup
 */
#define CONFIG_PCI 1
#define CONFIG_PCI_AUTOCONFIG 1
#undef  CONFIG_ISA
#undef  CONFIG_ISA_DMA
#define CONFIG_KERNEL_START 0xc0000000
#undef  CONFIG_ZBOOT_ROM
#define CONFIG_ZBOOT_ROM_TEXT 0x0
#define CONFIG_ZBOOT_ROM_BSS 0x0
#define CONFIG_PCI_NAMES 1
#undef  CONFIG_HOTPLUG
#undef  CONFIG_PCMCIA
#define CONFIG_NET 1
#define CONFIG_SYSVIPC 1
#undef  CONFIG_BSD_PROCESS_ACCT
#define CONFIG_SYSCTL 1

/*
 * At least one math emulation must be selected
 */
#define CONFIG_FPE_NWFPE 1
#undef  CONFIG_FPE_NWFPE_XP
#undef  CONFIG_FPE_FASTFPE
#define CONFIG_KCORE_ELF 1
#undef  CONFIG_KCORE_AOUT
#define CONFIG_BINFMT_AOUT 1
#define CONFIG_BINFMT_ELF 1
#undef  CONFIG_BINFMT_MISC
#undef  CONFIG_OOM_KILLER
#undef  CONFIG_PM
#undef  CONFIG_ARTHUR
#define CONFIG_CMDLINE_BOOL 1
#undef  CONFIG_CMDLINE_FORCE
#define CONFIG_CMDLINE "console=ttyS0,115200 root=/dev/ram0 initrd=0x00800000,8M mem=64M@0x00000000"
#define CONFIG_ALIGNMENT_TRAP 1

/*
 * Parallel port support
 */
#undef  CONFIG_PARPORT

/*
 * Memory Technology Devices (MTD)
 */
#define CONFIG_MTD 1
#undef  CONFIG_MTD_DEBUG
#define CONFIG_MTD_PARTITIONS 1
#undef  CONFIG_MTD_CONCAT
#define CONFIG_MTD_CONCAT_MODULE 1
#define CONFIG_MTD_REDBOOT_PARTS 1
#undef  CONFIG_MTD_REDBOOT_PARTS_UNALLOCATED
#undef  CONFIG_MTD_REDBOOT_PARTS_READONLY
#undef  CONFIG_MTD_CMDLINE_PARTS
#undef  CONFIG_MTD_AFS_PARTS

/*
 * User Modules And Translation Layers
 */
#define CONFIG_MTD_CHAR 1
#define CONFIG_MTD_BLOCK 1
#undef  CONFIG_FTL
#undef  CONFIG_NFTL
#undef  CONFIG_INFTL

/*
 * RAM/ROM/Flash chip drivers
 */
#define CONFIG_MTD_CFI 1
#undef  CONFIG_MTD_JEDECPROBE
#define CONFIG_MTD_GEN_PROBE 1
#undef  CONFIG_MTD_CFI_ADV_OPTIONS
#define CONFIG_MTD_CFI_INTELEXT 1
#undef  CONFIG_MTD_CFI_AMDSTD
#undef  CONFIG_MTD_CFI_STAA
#undef  CONFIG_MTD_RAM
#undef  CONFIG_MTD_ROM
#undef  CONFIG_MTD_ABSENT
#undef  CONFIG_MTD_OBSOLETE_CHIPS
#undef  CONFIG_MTD_AMDSTD
#undef  CONFIG_MTD_SHARP
#undef  CONFIG_MTD_JEDEC
#undef  CONFIG_MTD_PSD4256G

/*
 * Mapping drivers for chip access
 */
#undef  CONFIG_MTD_PHYSMAP
#undef  CONFIG_MTD_DRAGONIX
#undef  CONFIG_MTD_NETtel
#undef  CONFIG_MTD_SNAPGEODE
#undef  CONFIG_MTD_NETteluC
#undef  CONFIG_MTD_MBVANILLA
#undef  CONFIG_MTD_ML401
#undef  CONFIG_MTD_SUZAKU
#undef  CONFIG_MTD_KeyTechnology
#undef  CONFIG_MTD_SED_SIOSIII
#undef  CONFIG_MTD_NORA
#undef  CONFIG_MTD_ARM_INTEGRATOR
#undef  CONFIG_MTD_CDB89712
#undef  CONFIG_MTD_SA1100
#undef  CONFIG_MTD_DC21285
#undef  CONFIG_MTD_IQ80310
#undef  CONFIG_MTD_EPXA10DB
#undef  CONFIG_MTD_FORTUNET
#undef  CONFIG_MTD_AUTCPU12
#undef  CONFIG_MTD_IXP425
#undef  CONFIG_MTD_SE4000
#undef  CONFIG_MTD_SNAPARM
#undef  CONFIG_MTD_DM270
#undef  CONFIG_MTD_CLPS7500
#undef  CONFIG_MTD_EDB7312
#undef  CONFIG_MTD_EDB9301
#undef  CONFIG_MTD_EDB9302
#undef  CONFIG_MTD_EDB9312
#undef  CONFIG_MTD_EDB9315
#undef  CONFIG_MTD_IMPA7
#undef  CONFIG_MTD_CEIVA
#undef  CONFIG_MTD_VC547X
#undef  CONFIG_MTD_TIBURON
#undef  CONFIG_MTD_PCI
#undef  CONFIG_MTD_PCMCIA

/*
 * Self-contained MTD device drivers
 */
#undef  CONFIG_MTD_PMC551
#undef  CONFIG_MTD_SLRAM
#undef  CONFIG_MTD_MTDRAM
#undef  CONFIG_MTD_MTDCNXT
#undef  CONFIG_MTD_BLKMTD

/*
 * Disk-On-Chip Device Drivers
 */
#undef  CONFIG_MTD_DOC1000
#undef  CONFIG_MTD_DOC2000
#undef  CONFIG_MTD_DOC2001
#undef  CONFIG_MTD_DOC2001PLUS
#undef  CONFIG_MTD_DOCPROBE

/*
 * NAND Flash Device Drivers
 */
#undef  CONFIG_MTD_NAND

/*
 * Plug and Play configuration
 */
#undef  CONFIG_PNP
#undef  CONFIG_ISAPNP

/*
 * Block devices
 */
#undef  CONFIG_BLK_DEV_FD
#undef  CONFIG_BLK_DEV_XD
#undef  CONFIG_PARIDE
#undef  CONFIG_BLK_CPQ_DA
#undef  CONFIG_BLK_CPQ_CISS_DA
#undef  CONFIG_CISS_SCSI_TAPE
#undef  CONFIG_CISS_MONITOR_THREAD
#undef  CONFIG_BLK_DEV_DAC960
#undef  CONFIG_BLK_DEV_UMEM
#undef  CONFIG_BLK_DEV_SX8
#undef  CONFIG_BLK_DEV_LOOP
#undef  CONFIG_BLK_DEV_NBD
#define CONFIG_BLK_DEV_RAM 1
#define CONFIG_BLK_DEV_RAM_SIZE (8192)
#define CONFIG_BLK_DEV_INITRD 1
#undef  CONFIG_BLK_DEV_RAMDISK_DATA
#undef  CONFIG_BLK_DEV_BLKMEM
#undef  CONFIG_BLK_STATS

/*
 * Multi-device support (RAID and LVM)
 */
#undef  CONFIG_MD
#undef  CONFIG_BLK_DEV_MD
#undef  CONFIG_MD_LINEAR
#undef  CONFIG_MD_RAID0
#undef  CONFIG_MD_RAID1
#undef  CONFIG_MD_RAID5
#undef  CONFIG_MD_MULTIPATH
#undef  CONFIG_BLK_DEV_LVM

/*
 * Networking options
 */
#define CONFIG_PACKET 1
#undef  CONFIG_PACKET_MMAP
#undef  CONFIG_NETLINK_DEV
#undef  CONFIG_NETFILTER
#undef  CONFIG_FILTER
#undef  CONFIG_NET_NEIGH_DEBUG
#undef  CONFIG_NET_RESTRICTED_REUSE
#define CONFIG_UNIX 1
#define CONFIG_INET 1
#undef  CONFIG_IP_MULTICAST
#undef  CONFIG_IP_ADVANCED_ROUTER
#undef  CONFIG_IP_PNP
#undef  CONFIG_NET_ARP_LIMIT
#undef  CONFIG_NET_IPIP
#undef  CONFIG_NET_IPGRE
#undef  CONFIG_ARPD
#undef  CONFIG_INET_ECN
#undef  CONFIG_SYN_COOKIES
#undef  CONFIG_IPV6
#undef  CONFIG_KHTTPD

/*
 *    SCTP Configuration (EXPERIMENTAL)
 */
#undef  CONFIG_IP_SCTP
#undef  CONFIG_ATM
#undef  CONFIG_VLAN_8021Q

/*
 *  
 */
#undef  CONFIG_IPX
#undef  CONFIG_ATALK
#undef  CONFIG_DECNET
#undef  CONFIG_BRIDGE
#undef  CONFIG_X25
#undef  CONFIG_LAPB
#undef  CONFIG_LLC
#undef  CONFIG_NET_DIVERT
#undef  CONFIG_ECONET
#undef  CONFIG_WAN_ROUTER
#undef  CONFIG_NET_FASTROUTE
#undef  CONFIG_NET_HW_FLOWCONTROL

/*
 * QoS and/or fair queueing
 */
#undef  CONFIG_NET_SCHED
#undef  CONFIG_IPSEC
#undef  CONFIG_KLIPS

/*
 * Network testing
 */
#undef  CONFIG_NET_PKTGEN

/*
 * Network device support
 */
#define CONFIG_NETDEVICES 1

/*
 * ARCnet devices
 */
#undef  CONFIG_ARCNET
#undef  CONFIG_DUMMY
#undef  CONFIG_BONDING
#undef  CONFIG_EQUALIZER

/*
 * IMQ needs CONFIG_NETFILTER enabled
 */
#undef  CONFIG_TUN
#undef  CONFIG_ETHERTAP

/*
 * Ethernet (10 or 100Mbit)
 */
#define CONFIG_NET_ETHERNET 1
#undef  CONFIG_ARM_AM79C961A
#undef  CONFIG_ARM_CIRRUS
#undef  CONFIG_SUNLANCE
#undef  CONFIG_HAPPYMEAL
#undef  CONFIG_SUNBMAC
#undef  CONFIG_SUNQE
#undef  CONFIG_SUNGEM
#undef  CONFIG_NET_VENDOR_3COM
#undef  CONFIG_LANCE
#undef  CONFIG_NET_VENDOR_SMC
#undef  CONFIG_NET_VENDOR_RACAL
#undef  CONFIG_HP100
#undef  CONFIG_NET_ISA
#define CONFIG_NET_PCI 1
#undef  CONFIG_PCNET32
#undef  CONFIG_PCNET32_VMWARE
#undef  CONFIG_AMD8111_ETH
#undef  CONFIG_ADAPTEC_STARFIRE
#undef  CONFIG_APRICOT
#undef  CONFIG_B44
#undef  CONFIG_CS89x0
#undef  CONFIG_TULIP
#undef  CONFIG_DE4X5
#undef  CONFIG_DGRS
#undef  CONFIG_DM9102
#undef  CONFIG_EEPRO100
#define CONFIG_EEPRO100_MODULE 1
#undef  CONFIG_EEPRO100_PIO
#undef  CONFIG_E100
#undef  CONFIG_LNE390
#undef  CONFIG_FEALNX
#undef  CONFIG_NATSEMI
#undef  CONFIG_NE2K_PCI
#undef  CONFIG_FORCEDETH
#undef  CONFIG_NE3210
#undef  CONFIG_ES3210
#undef  CONFIG_8139CP
#undef  CONFIG_8139TOO
#undef  CONFIG_8139TOO_PIO
#undef  CONFIG_8139TOO_TUNE_TWISTER
#undef  CONFIG_8139TOO_8129
#undef  CONFIG_8139_OLD_RX_RESET
#undef  CONFIG_8139TOO_EXTERNAL_PHY
#undef  CONFIG_RTL8139
#undef  CONFIG_SIS900
#undef  CONFIG_EPIC100
#undef  CONFIG_SUNDANCE
#undef  CONFIG_SUNDANCE_MMIO
#undef  CONFIG_TLAN
#undef  CONFIG_VIA_RHINE
#undef  CONFIG_VIA_RHINE_FET
#undef  CONFIG_VIA_RHINE_MMIO
#undef  CONFIG_WINBOND_840
#undef  CONFIG_NET_POCKET
#undef  CONFIG_CNXT_EMAC
#undef  CONFIG_FEC
#undef  CONFIG_CS89x0
#undef  CONFIG_UCCS8900

/*
 * Ethernet (1000 Mbit)
 */
#undef  CONFIG_ACENIC
#undef  CONFIG_DL2K
#undef  CONFIG_E1000
#define CONFIG_E1000_MODULE 1
#undef  CONFIG_E1000_NAPI
#undef  CONFIG_MYRI_SBUS
#undef  CONFIG_NS83820
#undef  CONFIG_HAMACHI
#undef  CONFIG_YELLOWFIN
#undef  CONFIG_R8169
#undef  CONFIG_SK98LIN
#undef  CONFIG_TIGON3
#undef  CONFIG_FDDI
#undef  CONFIG_HIPPI
#undef  CONFIG_PLIP
#undef  CONFIG_PPP
#undef  CONFIG_SLIP

/*
 * Wireless LAN (non-hamradio)
 */
#undef  CONFIG_NET_RADIO

/*
 * Token Ring devices
 */
#undef  CONFIG_TR
#undef  CONFIG_NET_FC
#undef  CONFIG_RCPCI
#undef  CONFIG_SHAPER

/*
 * Wan interfaces
 */
#undef  CONFIG_WAN

/*
 * Amateur Radio support
 */
#undef  CONFIG_HAMRADIO

/*
 * IrDA (infrared) support
 */
#undef  CONFIG_IRDA

/*
 * ATA/ATAPI/MFM/RLL support
 */
#undef  CONFIG_IDE
#undef  CONFIG_BLK_DEV_HD

/*
 * SCSI support
 */
#undef  CONFIG_SCSI

/*
 * IEEE 1394 (FireWire) support (EXPERIMENTAL)
 */
#undef  CONFIG_IEEE1394

/*
 * I2O device support
 */
#undef  CONFIG_I2O
#undef  CONFIG_I2O_PCI
#undef  CONFIG_I2O_BLOCK
#undef  CONFIG_I2O_LAN
#undef  CONFIG_I2O_SCSI
#undef  CONFIG_I2O_PROC

/*
 * ISDN subsystem
 */
#undef  CONFIG_ISDN

/*
 * Input core support
 */
#undef  CONFIG_INPUT
#undef  CONFIG_INPUT_KEYBDEV
#undef  CONFIG_INPUT_MOUSEDEV
#undef  CONFIG_INPUT_JOYDEV
#undef  CONFIG_INPUT_EVDEV
#undef  CONFIG_INPUT_UINPUT

/*
 * Character devices
 */
#undef  CONFIG_LEDMAN
#undef  CONFIG_SNAPDOG
#undef  CONFIG_FAST_TIMER
#undef  CONFIG_DS1302
#undef  CONFIG_M41T11M6
#undef  CONFIG_VT
#define CONFIG_SERIAL 1
#define CONFIG_SERIAL_CONSOLE 1
#undef  CONFIG_SERIAL_EXTENDED
#undef  CONFIG_SERIAL_NONSTANDARD

/*
 * Serial drivers
 */
#undef  CONFIG_SERIAL_ANAKIN
#undef  CONFIG_SERIAL_ANAKIN_CONSOLE
#undef  CONFIG_SERIAL_AMBA
#undef  CONFIG_SERIAL_AMBA_CONSOLE
#undef  CONFIG_SERIAL_CLPS711X
#undef  CONFIG_SERIAL_CLPS711X_CONSOLE
#undef  CONFIG_SERIAL_21285
#undef  CONFIG_SERIAL_21285_OLD
#undef  CONFIG_SERIAL_21285_CONSOLE
#undef  CONFIG_SERIAL_UART00
#undef  CONFIG_SERIAL_UART00_CONSOLE
#undef  CONFIG_SERIAL_SA1100
#undef  CONFIG_SERIAL_SA1100_CONSOLE
#undef  CONFIG_SERIAL_OMAHA
#undef  CONFIG_SERIAL_OMAHA_CONSOLE
#undef  CONFIG_SERIAL_AT91
#undef  CONFIG_SERIAL_AT91_CONSOLE
#undef  CONFIG_SERIAL_8250
#undef  CONFIG_SERIAL_8250_CONSOLE
#undef  CONFIG_SERIAL_8250_EXTENDED
#undef  CONFIG_SERIAL_8250_MANY_PORTS
#undef  CONFIG_SERIAL_8250_SHARE_IRQ
#undef  CONFIG_SERIAL_8250_DETECT_IRQ
#undef  CONFIG_SERIAL_8250_MULTIPORT
#undef  CONFIG_SERIAL_8250_HUB6
#define CONFIG_UNIX98_PTYS 1
#define CONFIG_UNIX98_PTY_COUNT (256)

/*
 * I2C support
 */
#undef  CONFIG_I2C

/*
 * Mice
 */
#undef  CONFIG_BUSMOUSE
#define CONFIG_MOUSE 1
#define CONFIG_PSMOUSE 1
#undef  CONFIG_82C710_MOUSE
#undef  CONFIG_PC110_PAD
#undef  CONFIG_MK712_MOUSE

/*
 * Joysticks
 */
#undef  CONFIG_INPUT_GAMEPORT

/*
 * Input core support is needed for gameports
 */

/*
 * Input core support is needed for joysticks
 */
#undef  CONFIG_QIC02_TAPE
#undef  CONFIG_IPMI_HANDLER
#undef  CONFIG_IPMI_PANIC_EVENT
#undef  CONFIG_IPMI_DEVICE_INTERFACE
#undef  CONFIG_IPMI_KCS
#undef  CONFIG_IPMI_WATCHDOG

/*
 * Controller Area Network Cards/Chips
 */
#undef  CONFIG_CAN4LINUX

/*
 * Watchdog Cards
 */
#undef  CONFIG_WATCHDOG
#undef  CONFIG_SCx200
#undef  CONFIG_SCx200_GPIO
#undef  CONFIG_AMD_PM768
#undef  CONFIG_NVRAM
#undef  CONFIG_RTC
#undef  CONFIG_DTLK
#undef  CONFIG_R3964
#undef  CONFIG_APPLICOM

/*
 * Ftape, the floppy tape device driver
 */
#undef  CONFIG_FTAPE
#undef  CONFIG_AGP

/*
 * Direct Rendering Manager (XFree86 DRI support)
 */
#undef  CONFIG_DRM

/*
 * Multimedia devices
 */
#undef  CONFIG_VIDEO_DEV

/*
 * File systems
 */
#undef  CONFIG_QUOTA
#undef  CONFIG_QFMT_V2
#undef  CONFIG_AUTOFS_FS
#undef  CONFIG_AUTOFS4_FS
#undef  CONFIG_REISERFS_FS
#undef  CONFIG_REISERFS_CHECK
#undef  CONFIG_REISERFS_PROC_INFO
#undef  CONFIG_ADFS_FS
#undef  CONFIG_ADFS_FS_RW
#undef  CONFIG_AFFS_FS
#undef  CONFIG_HFS_FS
#undef  CONFIG_HFSPLUS_FS
#undef  CONFIG_BEFS_FS
#undef  CONFIG_BEFS_DEBUG
#undef  CONFIG_BFS_FS
#undef  CONFIG_EXT3_FS
#undef  CONFIG_JBD
#undef  CONFIG_JBD_DEBUG
#undef  CONFIG_FAT_FS
#undef  CONFIG_MSDOS_FS
#undef  CONFIG_UMSDOS_FS
#undef  CONFIG_VFAT_FS
#undef  CONFIG_EFS_FS
#undef  CONFIG_JFFS_FS
#define CONFIG_JFFS2_FS 1
#define CONFIG_JFFS2_FS_DEBUG (0)
#undef  CONFIG_CRAMFS
#undef  CONFIG_TMPFS
#define CONFIG_RAMFS 1
#undef  CONFIG_ISO9660_FS
#undef  CONFIG_JOLIET
#undef  CONFIG_ZISOFS
#undef  CONFIG_JFS_FS
#undef  CONFIG_JFS_DEBUG
#undef  CONFIG_JFS_STATISTICS
#undef  CONFIG_MINIX_FS
#undef  CONFIG_VXFS_FS
#undef  CONFIG_NTFS_FS
#undef  CONFIG_NTFS_RW
#undef  CONFIG_HPFS_FS
#define CONFIG_PROC_FS 1
#undef  CONFIG_DEVFS_FS
#undef  CONFIG_DEVFS_MOUNT
#undef  CONFIG_DEVFS_DEBUG
#define CONFIG_DEVPTS_FS 1
#undef  CONFIG_QNX4FS_FS
#undef  CONFIG_QNX4FS_RW
#undef  CONFIG_ROMFS_FS
#define CONFIG_EXT2_FS 1
#undef  CONFIG_SYSV_FS
#undef  CONFIG_UDF_FS
#undef  CONFIG_UDF_RW
#undef  CONFIG_UFS_FS
#undef  CONFIG_UFS_FS_WRITE
#undef  CONFIG_XFS_FS
#undef  CONFIG_XFS_QUOTA
#undef  CONFIG_XFS_RT
#undef  CONFIG_XFS_TRACE
#undef  CONFIG_XFS_DEBUG

/*
 * Network File Systems
 */
#undef  CONFIG_CODA_FS
#undef  CONFIG_INTERMEZZO_FS
#define CONFIG_NFS_FS 1
#undef  CONFIG_NFS_V3
#undef  CONFIG_NFS_DIRECTIO
#undef  CONFIG_ROOT_NFS
#undef  CONFIG_NFSD
#undef  CONFIG_NFSD_V3
#undef  CONFIG_NFSD_TCP
#define CONFIG_SUNRPC 1
#define CONFIG_LOCKD 1
#undef  CONFIG_SMB_FS
#undef  CONFIG_NCP_FS
#undef  CONFIG_NCPFS_PACKET_SIGNING
#undef  CONFIG_NCPFS_IOCTL_LOCKING
#undef  CONFIG_NCPFS_STRONG
#undef  CONFIG_NCPFS_NFS_NS
#undef  CONFIG_NCPFS_OS2_NS
#undef  CONFIG_NCPFS_SMALLDOS
#undef  CONFIG_NCPFS_NLS
#undef  CONFIG_NCPFS_EXTRAS
#undef  CONFIG_ZISOFS_FS
#undef  CONFIG_COREDUMP_PRINTK

/*
 * Partition Types
 */
#define CONFIG_PARTITION_ADVANCED 1
#undef  CONFIG_ACORN_PARTITION
#undef  CONFIG_OSF_PARTITION
#undef  CONFIG_AMIGA_PARTITION
#undef  CONFIG_ATARI_PARTITION
#undef  CONFIG_MAC_PARTITION
#define CONFIG_MSDOS_PARTITION 1
#define CONFIG_BSD_DISKLABEL 1
#define CONFIG_MINIX_SUBPARTITION 1
#define CONFIG_SOLARIS_X86_PARTITION 1
#define CONFIG_UNIXWARE_DISKLABEL 1
#undef  CONFIG_LDM_PARTITION
#undef  CONFIG_SGI_PARTITION
#undef  CONFIG_ULTRIX_PARTITION
#undef  CONFIG_SUN_PARTITION
#undef  CONFIG_EFI_PARTITION
#undef  CONFIG_SMB_NLS
#undef  CONFIG_NLS

/*
 * Sound
 */
#undef  CONFIG_SOUND

/*
 * Misc devices
 */

/*
 * USB support
 */
#undef  CONFIG_USB

/*
 * Support for USB gadgets
 */
#undef  CONFIG_USB_GADGET

/*
 * Bluetooth support
 */
#undef  CONFIG_BLUEZ

/*
 * Kernel hacking
 */
#define CONFIG_FRAME_POINTER 1
#undef  CONFIG_DEBUG_USER
#undef  CONFIG_DEBUG_INFO
#undef  CONFIG_NO_PGT_CACHE
#define CONFIG_DEBUG_KERNEL 1
#undef  CONFIG_DEBUG_SLAB
#undef  CONFIG_MAGIC_SYSRQ
#undef  CONFIG_DEBUG_SPINLOCK
#undef  CONFIG_DEBUG_WAITQ
#undef  CONFIG_DEBUG_BUGVERBOSE
#undef  CONFIG_DEBUG_ERRORS
#define CONFIG_DEBUG_LL 1
#undef  CONFIG_DEBUG_DC21285_PORT
#undef  CONFIG_DEBUG_CLPS711X_UART2
#undef  CONFIG_KGDB
#undef  CONFIG_KGDB_SERIAL
#undef  CONFIG_KGDB_CONSOLE
#undef  CONFIG_KGDB_SYSRQ
#undef  CONFIG_KGDB_MORE
#define CONFIG_LOG_BUF_SHIFT (0)

/*
 * Cryptographic options
 */
#undef  CONFIG_CRYPTO

/*
 * Library routines
 */
#undef  CONFIG_CRC32
#define CONFIG_ZLIB_INFLATE 1
#define CONFIG_ZLIB_DEFLATE 1
