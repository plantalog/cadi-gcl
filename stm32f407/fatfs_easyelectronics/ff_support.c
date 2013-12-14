/********************************************************************************/
/*!
	@file			ff_support.c
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        1.00
    @date           2012.08.27
	@brief          Interface of FatFs API Control.						@n
					Based on Chan's FatFs Test Terminal Thanks!

    @section HISTORY
		2012.08.27	V1.00	ReReStart Here.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "ff_support.h"
/* check header file version for fool proof */
#if __FF_SUPPORT_H!= 0x0100
#error "header file version is not correspond!"
#endif

/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
//FF_RTC ff_rtc;										/* See ff_rtc_if.h */
DWORD AccSize;										/* Work register for fs command */
WORD AccFiles, AccDirs;								/* Work register for fs command */
FILINFO Finfo;										/* Work register for fs command */

#if _USE_LFN
char Lfname[512];
#endif
char Line[256];										/* Console input buffer */

FATFS Fatfs[_VOLUMES];								/* File system object for each logical drive */
FIL File[2];										/* File objects */
DIR Dir;											/* Directory object */

BYTE Buff[BUFSIZE] __attribute__ ((aligned (4)));	/* Working buffer */


volatile UINT Timer;								/* Performance timer (1kHz increment) */

/* Constants -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**************************************************************************/
/*! 
    MUST called by a timer interrupt-handler every 1ms
*/
/**************************************************************************/
inline void ff_support_timerproc(void)
{
	Timer++;
}


/**************************************************************************/
/*! 
	RealTimeClock function
*/
/**************************************************************************/
uint32_t get_fattime (void)
{
	/* Get local time */
	//rtc_gettime(&ff_rtc);

	/* Pack date and time into a DWORD variable */
	return	1;//  ((DWORD)(ff_rtc.year - 1980) << 25)
			//| ((DWORD)ff_rtc.month << 21)
			//| ((DWORD)ff_rtc.mday << 16)
			//| ((DWORD)ff_rtc.hour << 11)
			//| ((DWORD)ff_rtc.min << 5)
			//| ((DWORD)ff_rtc.sec >> 1);
}


/**************************************************************************/
/*! 
	FatFs Upper Layer. Scanning Files.
*/
/**************************************************************************/
static FRESULT scan_files (
	char* path		/* Pointer to the path name working buffer */
)
{
	DIR 	dirs;
	FRESULT res;
	BYTE 	i;
	char*	fn;


	if ((res = f_opendir(&dirs, path)) == FR_OK) {
		i = strlen(path);
		while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
			if (_FS_RPATH && Finfo.fname[0] == '.') continue;
#if _USE_LFN
			fn = *Finfo.lfname ? Finfo.lfname : Finfo.fname;
#else
			fn = Finfo.fname;
#endif
			if (Finfo.fattrib & AM_DIR) {
				AccDirs++;
				*(path+i) = '/'; strcpy(path+i+1, fn);
				res = scan_files(path);
				*(path+i) = '\0';
				if (res != FR_OK) break;
			} else {
				/*xprintf("%s/%s\n", path, fn);*/
				AccFiles++;
				AccSize += Finfo.fsize;
			}
		}
	}

	return res;
}


/**************************************************************************/
/*! 
	Display Status Messages
*/
/**************************************************************************/
static void put_rc(FRESULT rc)
{
	const char *str =
		"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
		"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
		"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
		"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
	FRESULT i;

	for (i = 0; i != rc && *str; i++) {
		while (*str++) ;
	}
	xprintf("rc=%u FR_%s\n", (UINT)rc, str);
}


