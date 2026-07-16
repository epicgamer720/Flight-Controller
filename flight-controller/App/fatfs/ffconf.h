/*---------------------------------------------------------------------------/
/  FatFs configuration: Flight Controller SD logging (FatFs R0.15)
/  Derived from Middlewares/FatFs/source/ffconf_template.h (FFCONF_DEF 80286).
/  Deltas from template: CP437, LFN off, no-RTC fixed timestamp 2026,
/  single volume, fixed 512 B sectors, R/W, no mkfs/exFAT.
/---------------------------------------------------------------------------*/

#define FFCONF_DEF	80286	/* Revision ID, must match FF_DEFINED in ff.h */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_READONLY	0	/* read/write: f_write/f_sync needed for logging */

#define FF_FS_MINIMIZE	0	/* keep f_stat() for LOGnnn free-slot scan */

#define FF_USE_FIND		0

#define FF_USE_MKFS		0	/* card is pre-formatted; no f_mkfs */

#define FF_USE_FASTSEEK	0

#define FF_USE_EXPAND	0

#define FF_USE_CHMOD	0

#define FF_USE_LABEL	0

#define FF_USE_FORWARD	0

#define FF_USE_STRFUNC	0
#define FF_PRINT_LLI	1
#define FF_PRINT_FLOAT	1
#define FF_STRF_ENCODE	3

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define FF_CODE_PAGE	437		/* U.S. */

#define FF_USE_LFN		0		/* 8.3 names only (LOGnnn.BIN fits) */
#define FF_MAX_LFN		255

#define FF_LFN_UNICODE	0

#define FF_LFN_BUF		255
#define FF_SFN_BUF		12

#define FF_FS_RPATH		0

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define FF_VOLUMES		1		/* single SD volume */

#define FF_STR_VOLUME_ID	0
#define FF_VOLUME_STRS		"RAM","NAND","CF","SD","SD2","USB","USB2","USB3"

#define FF_MULTI_PARTITION	0

#define FF_MIN_SS		512		/* SD card: fixed 512 B sectors */
#define FF_MAX_SS		512

#define FF_LBA64		0

#define FF_MIN_GPT		0x10000000

#define FF_USE_TRIM		0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_TINY		0

#define FF_FS_EXFAT		0		/* would require LFN; keep off */

#define FF_FS_NORTC		1		/* no RTC: fixed timestamp below */
#define FF_NORTC_MON	1
#define FF_NORTC_MDAY	1
#define FF_NORTC_YEAR	2026

#define FF_FS_NOFSINFO	0

#define FF_FS_LOCK		0

#define FF_FS_REENTRANT	0		/* bare-metal superloop, no RTOS */
#define FF_FS_TIMEOUT	1000

/*--- End of configuration options ---*/
