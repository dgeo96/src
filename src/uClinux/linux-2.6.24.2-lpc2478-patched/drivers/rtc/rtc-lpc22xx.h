#ifndef __RTC_LPC22XX_H
#define __RTC_LPC22XX_H

#include <asm/arch/lpc22xx.h>

/* APB=9 - Real Time Clock */
#define REG_RTC_ILR             (*((volatile unsigned long *) (APB_RTC_BASE+0x0000)))
#define KREG_RTC_ILR_RTCCIF       (1 << 0) /* Counter Increment Interrupt */
#define KREG_RTC_ILR_RTCALF       (1 << 1) /* Alarm Interrupt */
#define KREG_RTC_ILR_RTSSF        (1 << 2) /* Sub-Seconds Interrupt */
#define REG_RTC_CTC             (*((volatile unsigned long *) (APB_RTC_BASE+0x0004)))
#define REG_RTC_CCR             (*((volatile unsigned long *) (APB_RTC_BASE+0x0008)))
#define REG_RTC_CIIR            (*((volatile unsigned long *) (APB_RTC_BASE+0x000C)))
#define REG_RTC_AMR             (*((volatile unsigned long *) (APB_RTC_BASE+0x0010)))
#define REG_RTC_CTIME0          (*((volatile unsigned long *) (APB_RTC_BASE+0x0014)))
#define REG_RTC_CTIME1          (*((volatile unsigned long *) (APB_RTC_BASE+0x0018)))
#define REG_RTC_CTIME2          (*((volatile unsigned long *) (APB_RTC_BASE+0x001C)))
#define REG_RTC_SEC             (*((volatile unsigned long *) (APB_RTC_BASE+0x0020)))
#define REG_RTC_MIN             (*((volatile unsigned long *) (APB_RTC_BASE+0x0024)))
#define REG_RTC_HOUR            (*((volatile unsigned long *) (APB_RTC_BASE+0x0028)))
#define REG_RTC_DOM             (*((volatile unsigned long *) (APB_RTC_BASE+0x002C)))
#define REG_RTC_DOW             (*((volatile unsigned long *) (APB_RTC_BASE+0x0030)))
#define REG_RTC_DOY             (*((volatile unsigned long *) (APB_RTC_BASE+0x0034)))
#define REG_RTC_MONTH           (*((volatile unsigned long *) (APB_RTC_BASE+0x0038)))
#define REG_RTC_YEAR            (*((volatile unsigned long *) (APB_RTC_BASE+0x003C)))
#define REG_RTC_CISS            (*((volatile unsigned long *) (APB_RTC_BASE+0x0040)))
#define REG_RTC_ALSEC           (*((volatile unsigned long *) (APB_RTC_BASE+0x0060)))
#define REG_RTC_ALMIN           (*((volatile unsigned long *) (APB_RTC_BASE+0x0064)))
#define REG_RTC_ALHOUR          (*((volatile unsigned long *) (APB_RTC_BASE+0x0068)))
#define REG_RTC_ALDOM           (*((volatile unsigned long *) (APB_RTC_BASE+0x006C)))
#define REG_RTC_ALDOW           (*((volatile unsigned long *) (APB_RTC_BASE+0x0070)))
#define REG_RTC_ALDOY           (*((volatile unsigned long *) (APB_RTC_BASE+0x0074)))
#define REG_RTC_ALMON           (*((volatile unsigned long *) (APB_RTC_BASE+0x0078)))
#define REG_RTC_ALYEAR          (*((volatile unsigned long *) (APB_RTC_BASE+0x007C)))
#define REG_RTC_PREINT          (*((volatile unsigned long *) (APB_RTC_BASE+0x0080)))
#define REG_RTC_PREFRAC         (*((volatile unsigned long *) (APB_RTC_BASE+0x0084)))

#endif
