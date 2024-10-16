/****************************************************************
 * calc
 *  (C) 1993-2021 R. Grothmann
 *  (C) 2021-     E. Bouchare
 *
 * sysdep.c
 *
 ****************************************************************/
/* Primitive user interface for calc.
	- wait does not work, but waits for a keystroke
*/

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <fcntl.h>

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_power.h"
#include "fsl_usart.h"
#include "fsl_powerquad.h"

#include "fsl_sd.h"
#include "ff.h"
#include "diskio.h"

#include "sysdep.h"
#include "calc.h"
#include "stack.h"
#include "sysdep_pcm.h"
#include "OA_algorithm.h"

#include "lcd.h"

#include "fsl_i2c.h"
#include "mma8652fc.h"


/*******************************************************************************
 * global calc struct
 ******************************************************************************/

Calc _calc;
Calc *calc=&_calc;

static int omode=CC_EDIT;

static volatile bool updateFS =false;
FATFS *fileSystem;

/*******************************************************************************
 * STANDARD UART-CONSOLE INPUT/OUTPUT ROUTINES
 ******************************************************************************/
#define RING_BUF_SIZE	32

typedef volatile struct RingBuffer {
	char data[RING_BUF_SIZE];
	int	i_w;
	int i_r;
} RingBuffer;

RingBuffer rxbuf = {
	.i_w=0,
	.i_r=0
};

RingBuffer txbuf = {
	.i_w=0,
	.i_r=0
};

volatile int user_break=0;

// called from the ISR for each received char
void FLEXCOMM0_IRQHandler()
{
	uint32_t status = USART_GetStatusFlags(USART0);
    if ((USART0->FIFOSTAT & USART_FIFOSTAT_RXERR_MASK) != 0U) {
        /* Clear rx error state. */
        USART0->FIFOSTAT |= USART_FIFOSTAT_RXERR_MASK;
        /* clear rxFIFO */
        USART0->FIFOCFG |= USART_FIFOCFG_EMPTYRX_MASK;
    } else if (((rxbuf.i_w+1) % RING_BUF_SIZE)!=rxbuf.i_r) {
		while ((status & kUSART_RxFifoNotEmptyFlag) && (((rxbuf.i_w+1) % RING_BUF_SIZE)!=rxbuf.i_r)) {
			rxbuf.data[rxbuf.i_w]=USART0->FIFORD;
			rxbuf.i_w = (rxbuf.i_w+1) % RING_BUF_SIZE;
			status=USART_GetStatusFlags(USART0);
			user_break=1;
		}
    } else {
    	USART_ReadByte(USART0);
    }
	while ((status & kUSART_TxFifoNotFullFlag) && (txbuf.i_r!=txbuf.i_w)) {
		USART0->FIFOWR=txbuf.data[txbuf.i_r];
		txbuf.i_r=(txbuf.i_r+1) % RING_BUF_SIZE;
		status=USART_GetStatusFlags(USART0);
		if (txbuf.i_r==txbuf.i_w) {
			USART_DisableInterrupts(USART0, kUSART_TxLevelInterruptEnable);
		}
	}
}

void uart_init(USART_Type *base, uint32_t baudrate)
{
    usart_config_t config;

    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

     /*
       * config.baudRate_Bps = 115200U;
       * config.parityMode = kUSART_ParityDisabled;
       * config.stopBitCount = kUSART_OneStopBit;
       * config.loopback = false;
       * config.enableTxFifo = false;
       * config.enableRxFifo = false;
       */
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = baudrate;
	config.enableTx     = true;
	config.enableRx     = true;

	USART_Init(base, &config, CLOCK_GetFlexCommClkFreq(0U));

	/* Enable RX interrupt. */
	USART_EnableInterrupts(base, kUSART_RxLevelInterruptEnable);
    NVIC_SetPriority(FLEXCOMM0_IRQn,3);
    NVIC_EnableIRQ(FLEXCOMM0_IRQn);
}

static void update_sd_state();

/*
 * uart_putc : send a char over the serial link (polling)
 */
void uart_putc(USART_Type *base, char c) {
	while ((txbuf.i_w+1) % RING_BUF_SIZE == txbuf.i_r) ;	// buffer full, so wait.
	uint32_t interruptMask = USART_GetEnabledInterrupts(base);
	USART_DisableInterrupts(base, interruptMask);
	txbuf.data[txbuf.i_w]=c;
	txbuf.i_w = (txbuf.i_w+1) % RING_BUF_SIZE;
	// enable TxE IRQ
    USART_EnableInterrupts(base, interruptMask|kUSART_TxLevelInterruptEnable);
}

void uart_puts(USART_Type *base, const char *s) {
	for (int i=0; s[i]!=0; i++) {
		uart_putc(base, s[i]);
	}
}

/*
 * uart_getc : get a char from the serial link (blocking)
 */
char uart_getc(USART_Type *base)
{
	char c;
    uint32_t interruptMask = 0U;

restart:
	while (rxbuf.i_w==rxbuf.i_r && !updateFS) ;
	user_break=0;
    if (updateFS) {
    	updateFS=false;
    	update_sd_state();
    	goto restart;
    }
	interruptMask = USART_GetEnabledInterrupts(base);
	USART_DisableInterrupts(base, interruptMask);
	c=rxbuf.data[rxbuf.i_r];
	rxbuf.i_r = (rxbuf.i_r+1) % RING_BUF_SIZE;
	USART_EnableInterrupts(base, interruptMask);
	return c;
}

/* Allow color and prompt customisation
 *   valid 'mode' parameter values include
 *   - CC_OUTPUT: prepare for output display
 *   - CC_WARN: prepare for warning display
 *   - CC_ERROR: prepare for error display
 *   - CC_EDIT: prepare for standard input (prompt "> ")
 *   - CC_FEDIT: prepare for function body input (prompt "$ ")
 */
void sys_out_mode (int mode)
{
	omode=mode;
	switch (mode) {
	case CC_OUTPUT:
		uart_puts(USART0,"\x1b[32m");
		break;
	case CC_WARN:
		uart_puts(USART0,"\x1b[35m");
		break;
	case CC_ERROR:
		uart_puts(USART0,"\x1b[31m");
		break;
	case CC_EDIT:
		uart_puts(USART0,"> ");
		uart_puts(USART0,"\x1b[96m");
		break;
	case CC_FEDIT:
		uart_puts(USART0,"$ ");
		uart_puts(USART0,"\x1b[36m");
		break;
	default:
		break;
	}
}

void sys_print (char *s)
/*****
Print a line onto the text screen, parse tabs and '\n'.
Printing should be done at the cursor position. There is no need
to clear the line at a '\n'.
The cursor should move forward with the print.
Think of the function as a simple emulator.
If you have a line buffered input with echo then do not print,
when the command line is on.
*****/
{
	char crlf[3]="\r\n";
	while (*s) {
		if (*s=='\n') {
			uart_puts(USART0,crlf);
		} else {
			uart_putc(USART0,*s);
		}
		s++;
	}
}

/***** 
	wait for a keystroke. return the scancode and the ascii code.
	scancode should be a code from scan_t. Do at least generate
	'enter'.
*****/
int sys_wait_key (scan_t *scan)
{
	 char c = uart_getc(USART0);
	 if (c == '\x1b') { // Start of an escape sequence

		 c = uart_getc(USART0);
		 if (c == '[') { // Escape sequences start with "\x1b["
			 c = uart_getc(USART0);
			 switch (c) {
				 case 'A':
					 *scan = cursor_up;
					 break;

				 case 'B':
					 *scan = cursor_down;
					 break;

				 case 'C':
					 *scan = cursor_right;
					 break;

				 case 'D':
					 *scan = cursor_left;
					 break;

				 case 'F':
					 *scan = line_end;
					 break;

				 case 'H':
					 *scan = line_start;
					 break;

				 case '3':
					 uart_getc(USART0); //read the '~'
					 *scan = delete;
					 break;

				 case '5':
					 uart_getc(USART0);
					 *scan = page_up;
					 break;

				 case '6':
					 uart_getc(USART0);
					 *scan = page_down;
					 break;

				 default:
					 *scan = key_normal;
					 break;
				 }
			 	 if (c == '1') { // F5 à F9
				 c = uart_getc(USART0);
				 switch (c) {
				 	 case '5':
					 uart_getc(USART0);
					 *scan = fk5;
					 break;

					 case '7':
						 uart_getc(USART0);
						 *scan = fk6;
						 break;

					 case '8':
						 uart_getc(USART0);
						 *scan = fk7;
						 break;

					 case '9':
						 uart_getc(USART0);
						 *scan = fk8;
						 break;

					 default:
						 *scan = key_normal;
						 break;
				 	 }

			 	 }
			 	 else if (c == '2') { // F10 à F12
					 c = uart_getc(USART0);
					 switch (c) {

						 case '0':
							 uart_getc(USART0); // Read the tilde '~'
							 *scan = fk9;
							 break;

						 case '1':
							 uart_getc(USART0); // Read the tilde '~'
							 *scan = fk10;
							 break;

						 case '3':
							 uart_getc(USART0); // Read the tilde '~'
							 *scan = fk11;
							 break;

						 case '4':
							 uart_getc(USART0); // Read the tilde '~'
							 *scan = fk12;
							 break;

						 default:
							 *scan = key_normal;
							 break;
					 	 }

			 	 }



		 } else if (c == 'O') { // Function keys F1 to F4 start with "\x1bO"

			 c = uart_getc(USART0);
			 switch (c) {
				 case 'P':
					 *scan = fk1;
					 break;

				 case 'Q':
					 *scan = fk2;
					 break;
				 case 'R':
					 *scan = fk3;
					 break;

				 case 'S':
					 *scan = fk4;
					 break;

				 default:
					 *scan = key_normal;
					 break;

				 }

		 }

		 else if (c == '\x1b') {
			 *scan = escape;

		 }

	 } else if (c == '\x0') {

		 c = uart_getc(USART0);
		 switch (c) {
		 	 case '4':
		 		 *scan = eot;
		 		 break;
		 }

	 }

	 else if (c=='\x7f') {
		 *scan = backspace;
	 }
	 else if (c=='\x08') {
		 *scan = backspace;
	 }

	 else if (c == '\r' || c == '\n') {
		 *scan = enter;
		 return '\n';

	 } else {
		 *scan = key_normal;
	 }
	 return c;

}

int sys_test_key (void)
/***** test_key
	see, if user pressed the keyboard.
	return the scancode, if he did.
*****/
{
	if (USART0->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK) {
		USART_ReadByte(USART0);
		return escape;
	}
	return 0;
}

int test_code (void)
/***** test_code
	see, if user pressed the keyboard.
	return the scancode, if he did.
*****/
{	return 0;
}

/****
The following text screen commands should be emulated on a graphic
work station. This can be done by a standard emulator (e.g. VT100)
or within a window displaying the text. Additional features may be
added, such as viewing old text. But the input line should be
visible as soon as a key is pressed by the user.
****/

void sys_clear (void)
/***** Clear the text screen
******/
{
	uart_puts(USART0, "\x1b[2J");
}

void text_mode ()
{
}

void move_cl_cb (void)
/* move the text cursor left */
{
	/* A COMPLETER */
	uart_puts(USART0, "\x1b[1D");
}

void move_cr_cb (void)
/* move the text cursor right */
{
	/* A COMPLETER */
	uart_puts(USART0, "\x1b[1C");
}

void cursor_on_cb (void)
/* switch cursor on */
{
	/* A COMPLETER */
	uart_puts(USART0, "\x1b[?25h");
}

void cursor_off_cb (void)
/* switch cursor off */
{
	/* A COMPLETER */
	uart_puts(USART0, "\x1b[?251");
}

void clear_eol (void)
/* clear the text line from cursor position */
{
	/* A COMPLETER */
	uart_puts(USART0, "\x1b[K");
}

void edit_off_cb (void)
/* the command line is no longer in use (graphics or computing) */
{
}

void edit_on_cb (void)
/* the command line is active */
{
}

void page_up_cb(void)
{
}

void page_down_cb()
{
}
/***************** clock and wait ********************/
volatile uint32_t sys_tick_cnt=0;
extern uint32_t SystemCoreClock;

/* sys_tick timer IRQ Handler
 *    limit sys_tick_cnt to 23 bits
 */
void SysTick_Handler(void) {
    if (sys_tick_cnt < (1 << 23) - 1) {
        sys_tick_cnt++;
    }
    else{
    	sys_tick_cnt = 0;
    }
}

real sys_clock(void) {
    return (real)sys_tick_cnt;
}

void sys_wait(real time, scan_t *scan) {
    // Si le délai est négatif, on attend une touche
	//time = -1000;
    if (time < 0) {
        uart_puts(USART0,"Press a key...\n");
        while (1) {
            if (user_break) {
                char c = uart_getc(USART0);  // Lire la touche
                *scan = 0;  // Retourner le code d'échappement
                return;
            }
        }
    } else {
        uint32_t start_time = sys_tick_cnt;  // Sauvegarde du temps de départ
        // Attente jusqu'à ce que le temps soit écoulé ou qu'une touche soit pressée
        while ((sys_tick_cnt - start_time) < (uint32_t)time) {
            if (user_break) {
                char c = uart_getc(USART0);  // Lire la touche
                *scan = escape;  // Retourner le code d'échappement
                return;
            }
        }
        // Temps écoulé sans interruption
        *scan = 0;
    }
}

void init_systick(void) {
    uint32_t ticks = SystemCoreClock / 1000;  // 1 ms
    SysTick_Config(ticks);
    NVIC_EnableIRQ(SysTick_IRQn);  // Activer les interruptions SysTick
}

/*******************************************************************************
 * FILE INPUT/OUTPUT ROUTINES (SDCARD)
 ******************************************************************************/
static bool card_ready = false;
sd_card_t card;
static FATFS fs;

char cur_path[64];

status_t sd_init(sd_card_t *card, sd_cd_t detectCb, void *userData)
{
	SDK_ALIGN(static uint32_t s_sdmmcHostDmaBuffer[64], SDMMCHOST_DMA_DESCRIPTOR_BUFFER_ALIGN_SIZE);
	static sd_detect_card_t s_cd;
	static sdmmchost_t s_host;
	
	/* attach main clock to SDIF */
	CLOCK_AttachClk(kMAIN_CLK_to_SDIO_CLK);
	/* need call this function to clear the halt bit in clock divider register */
	CLOCK_SetClkDiv(kCLOCK_DivSdioClk, (uint32_t)(SystemCoreClock / FSL_FEATURE_SDIF_MAX_SOURCE_CLOCK + 1U), true);
	
	memset(card, 0U, sizeof(sd_card_t));
	card->host = &s_host;
	card->host->dmaDesBuffer					= s_sdmmcHostDmaBuffer;
	card->host->dmaDesBufferWordsNum			= 64;
	card->host->hostController.base				= SDIF;
	card->host->hostController.sourceClock_Hz	= CLOCK_GetSdioClkFreq();
	
	/* install card detect callback */
	s_cd.cdDebounce_ms	= 100u;
	s_cd.type			= kSD_DetectCardByHostCD;
	s_cd.callback		= detectCb;
	s_cd.userData		= userData;
	card->usrParam.cd	= &s_cd;
	
	NVIC_SetPriority(SDIO_IRQn, 5);
	
	return SD_HostInit(card);
}


static void sd_info(sd_card_t *card)
{
    assert(card);

    outputf(calc,"\nCard size %d * %d bytes\n", card->blockCount, card->blockSize);
    outputf(calc,"Working condition:\n");
    if (card->operationVoltage==kSDMMC_OperationVoltage330V) {
        outputf(calc,"  Voltage : 3.3V\n");
    } else if (card->operationVoltage==kSDMMC_OperationVoltage180V) {
        outputf(calc,"  Voltage : 1.8V\n");
    }

    if (card->currentTiming == kSD_TimingSDR12DefaultMode) {
        if (card->operationVoltage==kSDMMC_OperationVoltage330V) {
            outputf(calc,"  Timing mode: Default mode\n");
        } else if (card->operationVoltage==kSDMMC_OperationVoltage180V) {
            outputf(calc,"  Timing mode: SDR12 mode\n");
        }
    } else if (card->currentTiming==kSD_TimingSDR25HighSpeedMode) {
        if (card->operationVoltage==kSDMMC_OperationVoltage180V) {
            outputf(calc,"  Timing mode: SDR25\n");
        } else {
            outputf(calc,"  Timing mode: High Speed\n");
        }
    } else if (card->currentTiming==kSD_TimingSDR50Mode) {
        outputf(calc,"  Timing mode: SDR50\n");
    } else if (card->currentTiming==kSD_TimingSDR104Mode) {
        outputf(calc,"  Timing mode: SDR104\n");
    } else if (card->currentTiming==kSD_TimingDDR50Mode) {
        outputf(calc,"  Timing mode: DDR50\n");
    }

    outputf(calc,"\r\n  Freq : %d HZ\r\n",card->busClock_Hz);
}

/* card_detect_cb
 *   card detection callback, called from the SDIO card detection ISR
 */
static void card_detect_cb(bool isInserted, void *userData)
{
    updateFS = true;
}

/* update_sd_state
 *   initialize SD card, mount/unmount the filesystem
 */
static void update_sd_state()
{
	FRESULT res;
	const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};

	if (!card_ready) {
		/* Init card. */
		SD_CardInit(&card);
		sd_info(&card);
		/* mount file system */
		res=f_mount(&fs,driverNumberBuffer,1);
		if (res!=FR_OK) return;
		/* select the drive */
		res=f_chdrive(driverNumberBuffer);
		if (res!=FR_OK) return;
		/* get the current root path */
		res=f_getcwd(cur_path, sizeof(cur_path));
		if (res!=FR_OK) return;
		card_ready=true;
	} else {
		card_ready=false;
		/* unmount file system */
		f_mount(NULL,driverNumberBuffer,0);
	}
}

/* search path list:
 *   path[0]       --> current directory
 */
char *path[MAX_PATH]={cur_path};
int npath=1;


/*
 *	scan a directory and get :
 *		files : an array of entries matching the pattern
 *		files_count : number of files entries
 *	the function returns the max length of a file entry
 */
static int match (char *pat, char *s)
{	if (*pat==0) return *s==0;
	if (*pat=='*') {
		pat++;
		if (!*pat) return 1;
		while (*s) {
			if (match(pat,s)) return 1;
			s++;
		}
		return 0;
	}
	if (*s==0) return 0;
	if (*pat=='?') return match(pat+1,s+1);
	if (*pat!=*s) return 0;
	return match(pat+1,s+1);
}

static int entry_cmp(const char**e1, char**e2)
{
	return strcmp(*e1,*e2);
}

/* fs_dir --> ls command
 *   list the files from the directory *dir_name* corresponding to pattern *pat*
 *   returns an array of all the *files_count* names in the *files* array
 */
int fs_dir(char *dir_name, char *pat, char ** files[], int *files_count)
{
	FRESULT res;
	DIR dir;
	FILINFO f;
	int entry_count=0, len=0;
	char **buf = NULL, **tmp;

	if (!card_ready) return 0;

	res=f_opendir(&dir, dir_name);
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &f);                   /* Read a directory item */
            if (res != FR_OK || f.fname[0] == 0) break;  /* Break on error or end of dir */
			if (match(pat,f.fname)) {
//				if (strcmp(f.fname,".")==0 || strcmp(f.fname,"..")==0)
//					continue;
				int isdir=f.fattrib & AM_DIR;            /* is it a directory? */
				int l=strlen(f.fname);
				len = len>l ? len : l;
				tmp = (char**)realloc(buf,(entry_count+1)*sizeof(char *));
				if (tmp) {
					buf=tmp;
					buf[entry_count]=(char*)malloc(isdir ? l+2 : l+1);
					if (buf[entry_count]) {
						strcpy(buf[entry_count],f.fname);
						if (isdir) {
							strcat(buf[entry_count],"/");
						}
						entry_count++;
					} else break;
				} else break;
			}
		}

		f_closedir(&dir);

		if (buf)
			qsort(buf,entry_count,sizeof(char*),(int (*)(const void*, const void*))entry_cmp);
	}

	*files = buf;
	*files_count = entry_count;

	return len;
}

/* fs_cd --> cd command
 *   sets the path if dir!=0 and returns the path
 *   else returns the current path
 */
char *fs_cd (char *dir)
{
	/* A COMPLETER */
	if (dir != NULL) {
		f_chdir(dir);
		f_getcwd(path[0],sizeof(path));
		return path[0];

	}
	else {
		return cur_path;
	}
}

/* fs_mkdir --> mkdir command
 *   create a new directory
 */
int fs_mkdir(char* dirname)
{
	FRESULT res;
	if (!card_ready) return -1;

	/* A COMPLETER */
	else {
		res = f_mkdir(dirname);
		if(res==FR_OK) return 1;
		}

	return -1;
}

/* fs_rm --> rm command
 *   delete a file or empty directory
 */
int fs_rm(char* filename)
{
	FRESULT res;
	if (!card_ready) return -1;

	/* A COMPLETER */
	else {
			res = f_unlink(filename);
			if(res==FR_OK) return 1;
			}

	return -1;
}

/************************ libc stubs *******************/
/*   file access:
 *   - to load modules
 *   - to read the help text
 */
#define MAX_FILES			5

FIL files[MAX_FILES];
int cur_file=-1;

typedef struct {
    int file_stack;
    int flags;
} file_ref;

int _open(const char *name, int flags, int mode) {
    FRESULT res;
    BYTE fatfs_mode = 0;
    if (flags & O_WRONLY) {
        fatfs_mode = FA_WRITE;
    }else if (flags & O_RDWR) {
        fatfs_mode = FA_READ | FA_WRITE;
    }else{
        fatfs_mode = FA_READ;
    }
    if (flags & O_CREAT) {
        fatfs_mode |= FA_OPEN_ALWAYS;
    }
    if (flags & O_EXCL) {
        fatfs_mode |= FA_CREATE_NEW;
    }
    if (flags & O_APPEND) {
        fatfs_mode |= FA_OPEN_APPEND;
    }
    // Find an empty file object slot
    for (int i = 0; i < MAX_FILES; i++) {
        if (files[i].obj.fs == NULL) {  // Check if file object is unused
            res = f_open(&files[i], name, fatfs_mode);
            if (res == FR_OK) {
                cur_file = i;
                return i;  
           }
            break;
        }
    }
    return -1;
}

int _close(int fd) {
    if (fd < 0 || fd >= MAX_FILES || files[fd].obj.fs == NULL) {
        return -1;  // Invalid file descriptor
    }
    FRESULT res = f_close(&files[fd]);
    if (res == FR_OK) {
        memset(&files[fd], 0, sizeof(FIL));  // Clear the file object
        return 0;  // Success
    }
    return -1;  // Error closing file
}

// do not work 
int _read(int fd, char *ptr, int len) {
    if (fd < 0 || fd >= MAX_FILES || files[fd].obj.fs == NULL) {
        return -1;  // Invalid file descriptor
    }
    UINT bytes_read;
    FRESULT res = f_read(&files[fd], ptr, len, &bytes_read);
    if (res == FR_OK) {
        return bytes_read;  // Return the number of bytes read
    }
    return -1;  // Error reading file
}

int _write(int fd, char *ptr, int len) {
    if (fd < 0 || fd >= MAX_FILES || files[fd].obj.fs == NULL) {
        return -1;  // Invalid file descriptor
    }
    UINT bytes_written;
    FRESULT res = f_write(&files[fd], ptr, len, &bytes_written);
    if (res == FR_OK) {
        return bytes_written;  // Return the number of bytes written
    }
    return -1;  // Error writing file
}

/*******************************************************************************
 * MULTICORE HANDLING
 ******************************************************************************/
#ifdef MULTICORE_APP
#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void)
{
    uint32_t image_size;
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
#elif defined(__ICCARM__)
#pragma section = "__core1_image"
    image_size = (uint32_t)__section_end("__core1_image") - (uint32_t)&core1_image_start;
#elif defined(__GNUC__)
    image_size = (uint32_t)core1_image_size;
#endif
    return image_size;
}
#endif

/* mailbox communication between cores */
#define EVT_MASK						(0xFF<<24)

#define EVT_NONE						0
#define EVT_CORE_UP						(1U<<24)
#define EVT_RETVAL						(2U<<24)
static volatile bool core1_up = false;
static volatile bool core1_handshake = false;

void mb_init(void)
{
    CLOCK_EnableClock(kCLOCK_Mailbox);
    RESET_PeripheralReset(kMAILBOX_RST_SHIFT_RSTn);
    NVIC_SetPriority(MAILBOX_IRQn, 5);
    NVIC_EnableIRQ(MAILBOX_IRQn);
}

/* pop events from CPU1 */
uint32_t mb_pop_evt(void)
{
	uint32_t evt=MAILBOX->MBOXIRQ[1].IRQ & EVT_MASK;
	MAILBOX->MBOXIRQ[1].IRQCLR=evt;
	return evt;
}

/* send event to CPU1, wait if there is already a pending event, unless force is set */
bool mb_push_evt(uint32_t evt, bool force)
{
	if (MAILBOX->MBOXIRQ[0].IRQ & EVT_MASK && !force) {
		return false;
	}
	MAILBOX->MBOXIRQ[0].IRQ=evt;
	return true;
}

/* Mailbox IRQ handler */
void MAILBOX_IRQHandler(void)
{
	uint32_t data = mb_pop_evt();

	if ((data & EVT_MASK)==EVT_CORE_UP) {
		core1_up = true;
	} else if ((data & EVT_MASK)==EVT_RETVAL) {
		core1_handshake=true;
	}
}

/* start core1 from bootaddr address */
void core1_startup(void *bootaddr)
{
	mb_init();

    // Start the CORE_1 CPU
    SYSCON->CPUCFG |= SYSCON_CPUCFG_CPU1ENABLE_MASK;

	/* Boot source for Core 1 from RAM */
	SYSCON->CPBOOT = (uint32_t)(bootaddr);

	uint32_t temp = SYSCON->CPUCTRL | 0xc0c48000U;
	SYSCON->CPUCTRL = temp | SYSCON_CPUCTRL_CPU1RSTEN_MASK | SYSCON_CPUCTRL_CPU1CLKEN_MASK;
	SYSCON->CPUCTRL = (temp | SYSCON_CPUCTRL_CPU1CLKEN_MASK) & (~SYSCON_CPUCTRL_CPU1RSTEN_MASK);

	while(!core1_up);	/* wait for core 1 to start */
}

bool handshake(void)
{
	while (!core1_handshake) {}
	core1_handshake=false;
	return true;
}

/* LCD remote interface */
#define EVT_DRAWPOINT					(3U<<24)
#define EVT_DRAWLINE					(4U<<24)
#define EVT_DRAWRECT					(5U<<24)
#define EVT_DRAWRNDRECT					(6U<<24)
#define EVT_DRAWCIRCLE					(7U<<24)
#define EVT_DRAWELLISPSE				(8U<<24)
#define EVT_DRAWLINES					(9U<<24)
#define EVT_DRAWSEGMENTS				(10U<<24)
#define EVT_FILLRECT					(11U<<24)
#define EVT_CLIP						(12U<<24)
#define EVT_UNCLIP						(13U<<24)
#define EVT_DRAWPATH					(14U<<24)

#define EVT_FORECOLOR					(15U<<24)
#define EVT_BACKCOLOR					(16U<<24)
#define EVT_SETFONT						(17U<<24)
#define EVT_SETALIGNEMENT				(18U<<24)
#define EVT_DRAWSTRING					(19U<<24)
#define EVT_GETDEFAULTDC                (20U<<24)
#define EVT_CLEARSCREEN					(21U<<24)

#define EVT_GETBUFFER					(30U<<24)
#endif

/*******************************************************************************
 * SHARED BUFFER
 ******************************************************************************/
#define MAXPOINTS	2048

__attribute__ ((section(".shmem")))
SPoint shdata[2*MAXPOINTS];				/* shared data buffer */

/*******************************************************************************
 * GRAPHICS HANDLING ROUTINES
 ******************************************************************************/
typedef struct _Graph {
	int 			pxmin, pymin, pxmax, pymax, pxorg, pyorg;	// graph window in pixel coordinates
	real 			xorg, xmin, xmax, xfactor, xtick;	// x axis origin, limits and scaling
	real			yorg, ymin, ymax, yfactor, ytick;	// y axis origin, limits and scaling
	int				xscaleexp,yscaleexp;				// x and y axis factor exponent
	unsigned int	flags;								// graph style
	int				color;								// default color for next plot
	int				ltype;								// default line type for next plot
	int				lwidth;								// default line width
	int				mtype;								// default marker type for next plot
	int				msize;								// marker size
} Graph;

typedef struct GraphWindow {
	Graph			graph[9];
	int				pch, pcw;		// char pixel size
	int				n;				// number of graph used
	int				rows,cols;		// subplot layout: nb of rows, nb of columns
	int				cur;			// current graph used
} GraphWindow;

GraphWindow gw;

static Color gcolors[MAX_COLORS] = {
	BLACK,
	BLUE,
	BRED,
	GRED,
	GBLUE,
	RED,
	MAGENTA,
	GREEN,
	CYAN,
	YELLOW,
	BROWN,
	RGB(0x00,0xAE,0x00),
	RGB(0xFF,0x99,0x66),
	RGB(0x00,0x99,0xFF),
	RGB(0x99,0x66,0xCC)
};
static Color gridcolor = RGB(0xB3,0xB3,0xB3);

const Font *gsfont=&fixed8;

#ifdef MULTICORE_APP

/* Display management functions */
void lcd_set_display_orientation(Orientation orientation)
{

}

uint16_t lcd_get_display_width(void)
{
	return 0;
}

uint16_t lcd_get_display_height(void)
{
	return 0;
}

/* touchscreen API functions */
void    lcd_ts_id(uint8_t *ver, uint16_t *id)
{

}

uint8_t lcd_ts_get_data(uint16_t *x, uint16_t *y, uint8_t *z)
{
	return 0;
}

int     lcd_ts_touched(void)
{
	return 0;
}

/* memory emplacement for data types */
/* |			SPOINT			| DC | COLOR | FONT | GRAPH | ALIG | DIR | X1 | X2 | Y1 | Y2 | N | STR ....|*/

#define SHDATA_BASE		0x2003c000
#define SPOINT_ADR		SHDATA_BASE + 0
#define DC_ADR          SPOINT_ADR + 2048 * 4
#define COLOR_ADR       DC_ADR + 32
#define FONT_ADR        COLOR_ADR + 2
#define GRAPH_ADR       FONT_ADR + 8
#define ALING_ADR       GRAPH_ADR + 96
#define DIRECTION_ADR   ALING_ADR + 4
#define X1_ADR          DIRECTION_ADR + 4
#define X2_ADR          X1_ADR + 4
#define Y1_ADR          X2_ADR + 4
#define Y2_ADR          Y1_ADR + 4
#define N_ADR           Y2_ADR + 4
#define ISAVAILABLE_ADR (N_ADR + 4)
#define STR_ADR         ISAVAILABLE_ADR + 4

#define MASK_SPOINT		1<<0
#define MASK_DC			1<<1
#define MASK_COLOR		1<<2
#define MASK_FONT		1<<3
#define MASK_GRAPH		1<<4
#define MASK_ALIGN		1<<5
#define MASK_DIRECTION	1<<6
#define MASK_X1			1<<7
#define MASK_X2			1<<8
#define MASK_Y1			1<<9
#define MASK_Y2			1<<10
#define MASK_N			1<<11
#define MASK_STR		1<<12

static bool memory_available(uint32_t mask){
	if (mask & *(uint32_t*)ISAVAILABLE_ADR){
		return false;
	}
	return true;
}

/* Drawing context management functions */
void lcd_get_default_DC(DC *dc)
{
	while (!memory_available(MASK_DC)){}
	while (!mb_push_evt( EVT_GETDEFAULTDC|MASK_DC,false));
	handshake();
	memcpy(dc, (void *)DC_ADR, sizeof(DC));
}

const Font* lcd_set_font(DC *dc, const Font *font)
{
	while (!memory_available(MASK_DC|MASK_FONT)){}
	memcpy((void*)DC_ADR, dc, sizeof(DC));
	memcpy((void*)FONT_ADR, font, sizeof(Font));
	while (!mb_push_evt(EVT_SETFONT|MASK_DC|MASK_FONT,false));
	return NULL;
}

Color lcd_set_forecolor(Color color)
{
	while(!memory_available(MASK_COLOR)){}
	memcpy((void *)COLOR_ADR, &color,sizeof(Color));
	while (!mb_push_evt(EVT_FORECOLOR|MASK_COLOR,false)) ;
	return 0;
}

Color lcd_set_background(DC *dc, Color color)
{
	return color;
}

uint32_t lcd_set_alignment(DC *dc, uint32_t alignment)
{
	while(!memory_available(MASK_DC|MASK_ALIGN)){}
	memcpy((void *) DC_ADR, dc, sizeof(DC));
	memcpy((void *) ALING_ADR, &alignment, 4);
	while (!mb_push_evt(EVT_SETALIGNEMENT|MASK_DC|MASK_ALIGN,false));
	return 0;
}

uint32_t lcd_set_direction(DC *dc, uint32_t direction)
{
	return direction;
}

/* graphic drawing functions */
void lcd_clear_screen(uint16_t color)
{
	// while (!memory_available(MASK_COLOR)){}
	// memcpy((void*)COLOR_ADR, &color, sizeof(Color));
	// while (!mb_push_evt(EVT_CLEARSCREEN|MASK_COLOR,false)) ;
}

void lcd_draw_point(uint16_t x, uint16_t y, Color c)
{

}

void lcd_draw_line(int16_t x1, int16_t y1, int16_t x2, int16_t y2, Color c)
{
	while (!memory_available(MASK_X1|MASK_X2|MASK_Y1|MASK_Y2|MASK_COLOR)){}
	memcpy((int16_t*)X1_ADR, &x1, 2);
	memcpy((int16_t*)Y1_ADR, &y1, 2);
	memcpy((int16_t*)X2_ADR, &x2, 2);
	memcpy((int16_t*)Y2_ADR, &y2, 2);
	memcpy((Color*)COLOR_ADR, &c, sizeof(Color));
	while (!mb_push_evt(EVT_DRAWRECT|MASK_X1|MASK_X2|MASK_Y1|MASK_Y2|MASK_COLOR,false)) ;
}

void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, Color c)
{
	while (!memory_available(MASK_X1|MASK_X2|MASK_Y1|MASK_Y2|MASK_COLOR)){}
	memcpy((void*)X1_ADR, &x, 2);
	memcpy((void*)Y1_ADR, &y, 2);
	memcpy((void*)X2_ADR, &w, 2);
	memcpy((void*)Y2_ADR, &h, 2);
	memcpy((void*)COLOR_ADR, &c, sizeof(Color));
	while (!mb_push_evt(EVT_DRAWRECT|MASK_X1|MASK_X2|MASK_Y1|MASK_Y2|MASK_COLOR,false)) ;
}

void lcd_draw_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, Color c)
{

}

void lcd_draw_circle(int16_t x0, int16_t y0, int16_t r, Color c)
{

}

void lcd_draw_ellipse(Rect r, Color c, int lwidth)
{

}

void lcd_draw_segments(SPoint *p, int n, Color c)
{
	if (n<MAXPOINTS) {
		while(!memory_available(MASK_SPOINT|MASK_N|MASK_COLOR)){}
		memcpy((void*)SPOINT_ADR, p, sizeof(SPoint));
		memcpy((void*)N_ADR, &n, sizeof(int));
		memcpy((void*)COLOR_ADR, &c, sizeof(Color));
		while (!mb_push_evt(EVT_DRAWSEGMENTS|MASK_SPOINT|MASK_N|MASK_COLOR,false));
	}
}

void lcd_draw_lines(SPoint *p, int n)
{
	if (n<MAXPOINTS) {
		while(!memory_available(MASK_SPOINT|MASK_N)){}
		memcpy((void *)SPOINT_ADR, p, n*sizeof(SPoint));
		memcpy((void *)N_ADR, &n, sizeof(int));
		while (!mb_push_evt(EVT_DRAWLINES|MASK_SPOINT|MASK_N,false)) ;
	}
}

void lcd_draw_path2d(Graph *g, SPoint *p, int n)
{
	if (n<MAXPOINTS) {
		while(!memory_available(MASK_GRAPH|MASK_SPOINT|MASK_N)){}
		memcpy((void*)GRAPH_ADR, g, sizeof(Graph));
		memcpy((void*)SPOINT_ADR, p, sizeof(SPoint));
		memcpy((void*)N_ADR, &n, sizeof(int));
		while (!mb_push_evt(EVT_DRAWPATH|MASK_GRAPH|MASK_SPOINT|MASK_N,false));
	}
}

void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	while(!memory_available(MASK_X1|MASK_Y1|MASK_X2|MASK_Y2)){}
	memcpy((void*)X1_ADR, &x, 2);
	memcpy((void*)Y1_ADR, &y, 2);
	memcpy((void*)X2_ADR, &w, 2);
	memcpy((void*)Y2_ADR, &h, 2);
	memcpy((void*)COLOR_ADR, &color, 2);
	while (!mb_push_evt(EVT_FILLRECT|MASK_X1|MASK_Y1|MASK_X2|MASK_Y2,false)) ;
	while (!handshake()) {}
}

/* draw lines with clipping */
void lcd_clip(int x1, int y1, int x2, int y2)
{
	while(!memory_available(MASK_X1|MASK_Y1|MASK_X2|MASK_Y2)){}
	memcpy((void*)X1_ADR, &x1, sizeof(int));
	memcpy((void*)Y1_ADR, &y1, sizeof(int));
	memcpy((void*)X2_ADR, &x2, sizeof(int));
	memcpy((void*)Y2_ADR, &y2, sizeof(int));
	while (!mb_push_evt(EVT_CLIP|MASK_X1|MASK_Y1|MASK_X2|MASK_Y2,false)) ;
}

void lcd_unclip(void)
{
	while (!mb_push_evt(EVT_UNCLIP,false));
}

void lcd_line(int x1, int y1, int x2, int y2, Color c)
{

}

/* String drawing functions */
uint16_t lcd_draw_char(DC *dc, int16_t x, int16_t y, char c)
{
	return 0;
}

void lcd_draw_string(DC *dc, int16_t x, int16_t y, const char *s)
{
	while(!memory_available(MASK_DC| MASK_X1| MASK_Y1| MASK_STR)){}
	memcpy((void*)DC_ADR, dc, sizeof(DC));
	memcpy((void*)X1_ADR, &x, 2);
	memcpy((void*)Y1_ADR, &y, 2);
	memcpy((void*)STR_ADR, s, strlen(s)+1);
	while (!mb_push_evt(EVT_DRAWSTRING|MASK_DC|MASK_X1|MASK_Y1|MASK_STR,false)) ;
}

void lcd_get_string_size(DC *dc, const char *s, uint16_t *width, uint16_t *height)
{
}

void lcd_flush(void)
{

}

#endif

#define TICKSIZE		5
#define SUBTICKSIZE		3

void g_xgrid(Graph *g, header *xticks, real factor, unsigned int color)
{
	real *ticks=matrixof(xticks);
	int n=dimsof(xticks)->c;
	DC dc;
	
	lcd_get_default_DC(&dc);
	lcd_set_alignment(&dc,ALIGN_S);
	
	if (!(g->flags & G_XLOG)) {
		g->xfactor = (real)(g->pxmax - g->pxmin) / (g->xmax - g->xmin);
		for (int i=0 ; i<n ; i++) {
			shdata[2*i].x = (short)(g->pxmin + (ticks[i] - g->xmin) * g->xfactor);
			shdata[2*i].y = g->pymin;
			shdata[2*i+1].x = shdata[2*i].x;
			shdata[2*i+1].y = g->pymax;
		}
		
		if (g->flags & G_XGRID) {	/* draw the grid */
			lcd_draw_segments(shdata,n,gridcolor);
		}
				
		for (int i=0 ; i<n ; i++) {
			shdata[2*i+1].y = g->pymin+TICKSIZE;
		}
		lcd_draw_segments(shdata,n,BLACK);
		
		for (int i=0 ; i<n ; i++) {
			char s[32];
			snprintf(s,32,"%g",fabs(ticks[i]/factor) < 1e-6 ? 0.0 : ticks[i]/factor);
			shdata[2*i+1].y = g->pymax;
			shdata[2*i].y = shdata[2*i+1].y-TICKSIZE;
			lcd_draw_string(&dc,shdata[2*i+1].x,shdata[2*i+1].y+fixed12.height/2,s);
		}
		lcd_draw_segments(shdata,n,BLACK);
		
		if (factor!=1.0) {
			char s[32];
			snprintf(s,32,"x%g", factor);
			lcd_set_alignment(&dc,ALIGN_SW);
			lcd_draw_string(&dc,g->pxmax+fixed12.width,g->pymax+2*fixed12.height,s);
		}
	} else {
		g->xfactor = (real)(g->pxmax - g->pxmin) / log10(g->xmax / g->xmin);
		for (int i=0 ; i<n ; i++) {
			shdata[2*i].x = (short)(g->pxmin + g->xfactor*log10(ticks[i]/g->xmin));
			shdata[2*i].y = g->pymin;
			shdata[2*i+1].x = shdata[2*i].x;
			shdata[2*i+1].y = g->pymax;
		}
		
		if (g->flags & G_XGRID) {
			lcd_draw_segments(shdata,n,gridcolor);
		}
				
		for (int i=0 ; i<n ; i++) {
			real d = ticks[i]/pow(10.0,floor(log10(ticks[i])));
			if (d!=1.0) {
				shdata[2*i+1].y = g->pymin+SUBTICKSIZE;
			} else {
				shdata[2*i+1].y = g->pymin+TICKSIZE;
			}
		}
		lcd_draw_segments(shdata,n,BLACK);
		
		for (int i=0 ; i<n ; i++) {
			char s[32];
			shdata[2*i+1].y = g->pymax;
			
			real e = floor(log10(ticks[i]));
			real d = ticks[i]/pow(10.0,e);
			if (d!=1.0) {
				shdata[2*i].y = shdata[2*i+1].y-SUBTICKSIZE;
			} else {
				shdata[2*i].y = shdata[2*i+1].y-TICKSIZE;
				snprintf(s,32,"%g", e);
				lcd_set_font(&dc,&fixed12);
				lcd_set_alignment(&dc,ALIGN_S);
				lcd_draw_string(&dc,shdata[2*i+1].x,shdata[2*i+1].y+3*fixed12.height/4,"10");
				lcd_set_font(&dc,&fixed8);
				lcd_set_alignment(&dc,ALIGN_E);
				lcd_draw_string(&dc,shdata[2*i+1].x+fixed12.width,shdata[2*i+1].y+3*fixed12.height/4,s);
			}
		}
		lcd_draw_segments(shdata,n,BLACK);
	}
}

void g_ygrid(Graph *g, header *yticks, real factor, unsigned int color)
{
	real *ticks=matrixof(yticks);
	int n=dimsof(yticks)->c;
	DC dc;
	
	lcd_get_default_DC(&dc);
	lcd_set_alignment(&dc,ALIGN_W);
	
	if (!(g->flags & G_YLOG)) {
		g->yfactor = (real)(g->pymax - g->pymin) / (g->ymax - g->ymin);
		for (int i=0 ; i<n ; i++) {
			shdata[2*i].x = g->pxmin;
			shdata[2*i].y = g->pymax - g->yfactor*(ticks[i]-g->ymin);
			shdata[2*i+1].x = g->pxmax;
			shdata[2*i+1].y = shdata[2*i].y;
		}
		
		if (g->flags & G_YGRID) {	/* draw the grid */
			lcd_draw_segments(shdata,n,gridcolor);
		}
		
		for (int i=0 ; i<n ; i++) {
			char s[32];
			snprintf(s,32,"%g",fabs(ticks[i]/factor) < 1e-6 ? 0.0 : ticks[i]/factor);
			shdata[2*i+1].x = g->pxmin+TICKSIZE;
			lcd_draw_string(&dc,shdata[2*i].x-fixed12.width/2,shdata[2*i].y,s);
		}
		lcd_draw_segments(shdata,n,BLACK);
		
		for (int i=0 ; i<n ; i++) {
			shdata[2*i+1].x = g->pxmax;
			shdata[2*i].x = shdata[2*i+1].x-TICKSIZE;
		}
		
		lcd_draw_segments(shdata,n,BLACK);
		
		if (factor!=1.0) {
			char s[32];
			snprintf(s,32,"x%g", factor);
			lcd_set_alignment(&dc,ALIGN_SE);
			lcd_draw_string(&dc,g->pxmin-5*fixed12.width,g->pymin-fixed12.height,s);
		}
		
	} else {
		g->yfactor = (real)(g->pymax - g->pymin) / log10(g->ymax / g->ymin);
		for (int i=0 ; i<n ; i++) {
			shdata[2*i].x = g->pxmin;
			shdata[2*i].y = g->pymax - g->yfactor*log10(ticks[i]/g->ymin);
			shdata[2*i+1].x = g->pxmax;
			shdata[2*i+1].y = shdata[2*i].y;
		}
		
		if (g->flags & G_YGRID) {
			lcd_draw_segments(shdata,n,gridcolor);
		}
		
		for (int i=0 ; i<n ; i++) {
			char s[32];
			real e = floor(log10(ticks[i]));
			real d = ticks[i]/pow(10.0,e);
			if (d!=1.0) {
				shdata[2*i+1].x = g->pxmin+SUBTICKSIZE;
			} else {
				shdata[2*i+1].x = g->pxmin+TICKSIZE;
				int len = snprintf(s,32,"%g", e);
				lcd_set_font(&dc,&fixed12);
				lcd_set_alignment(&dc,ALIGN_W);
				lcd_draw_string(&dc,shdata[2*i].x-fixed12.width/2-len*fixed8.width,shdata[2*i].y,"10");
				lcd_set_font(&dc,&fixed8);
				lcd_set_alignment(&dc,ALIGN_E);
				lcd_draw_string(&dc,shdata[2*i].x-fixed12.width/2-len*fixed8.width,shdata[2*i].y-fixed12.height/2,s);
			}
		}
		lcd_draw_segments(shdata,n,BLACK);
		
		for (int i=0 ; i<n ; i++) {
			shdata[2*i+1].x = g->pxmax;
			real d = ticks[i]/pow(10.0,floor(log10(ticks[i])));
			if (d!=1.0) {
				shdata[2*i].x = shdata[2*i+1].x-SUBTICKSIZE;
			} else {
				shdata[2*i].x = shdata[2*i+1].x-TICKSIZE;
			}
		}
		lcd_draw_segments(shdata,n,BLACK);
	}
}


static int g_draw_plot(Graph *g, header *hdx, header *hdy)
{
	real *x, *y;
	int rx, ry, cx, cy;
	
	getmatrix(hdx,&rx,&cx,&x); getmatrix(hdy,&ry,&cy,&y);
	
	// no more than MAXPOINTS allowed
	if (cx>MAXPOINTS) cc_error(calc,"Too many points to draw");

	// clip frame
	lcd_clip(g->pxmin,g->pymin,g->pxmax,g->pymax);
	
	// precalculate the x value for the first line
	if (!(g->flags & G_XLOG)) {
		for (int i=0 ; i<cx ; i++) {
			shdata[i].x = (short)(g->pxmin + (x[i] - g->xmin) * g->xfactor);
		}
	} else {
		for (int i=0 ; i<cx ; i++) {
			shdata[i].x = (short)(g->pxmin + log10(x[i] / g->xmin) * g->xfactor);
		}
	}
	
	if (!(g->flags & G_YLOG)) {
		for (int k=0; k<ry; k++) {
			// if the x matrix has several lines, process x matrix 1 line at a time
			if (k && rx>1) {
				x+=cx;
				if (!(g->flags & G_XLOG)) {
					for (int i=0 ; i<cx ; i++) {
						shdata[i].x = (short)(g->pxmin + (x[i] - g->xmin) * g->xfactor);
					}
				} else {
					for (int i=0 ; i<cx ; i++) {
						shdata[i].x = (short)(g->pxmin + log10(x[i] / g->xmin) * g->xfactor);
					}
				}
			}
			// process y matrix 1 line at a time
			for (int i=0 ; i<cx ; i++) {
				if (y[i]>g->ymax)			/* avoid int16 overflow */
					shdata[i].y = g->pymin - 5;
				else if (y[i]<g->ymin)		/* avoid int16 overflow */
					shdata[i].y = g->pymax + 5;
				else						/* calculate standard case */
					shdata[i].y = (short)(g->pymin + (g->ymax - y[i]) * g->yfactor);
			}
			lcd_draw_path2d(g,shdata,cx);
			y+=cy;
			if (g->flags & G_AUTOCOLOR) {
				g->color=(g->color+1) % MAX_COLORS;
			}
			if (g->mtype!=M_NONE)
				g->mtype=(g->mtype+1) % (M_NONE-1);
			
			if (sys_test_key()==escape) break;
		}
	} else {
		// G_YLOG
		for (int k=0; k<ry; k++) {
			// if the x matrix has several lines, process x matrix 1 line at a time
			if (k && rx>1) {
				x+=cx;
				if (!(g->flags & G_XLOG)) {
					for (int i=0 ; i<cx ; i++) {
						shdata[i].x = (short)(g->pxmin + (x[i] - g->xmin) * g->xfactor);
					}
				} else {
					for (int i=0 ; i<cx ; i++) {
						shdata[i].x = (short)(g->pxmin + log10(x[i] / g->xmin) * g->xfactor);
					}
				}
			}
			for (int i=0 ; i<cx ; i++) {
				shdata[i].y = (short)(g->pymax - log10(y[i] /g->ymin) * g->yfactor);
			}
			lcd_draw_path2d(g,shdata,cx);
			y+=cy;
			if (g->flags & G_AUTOCOLOR) {
				g->color=(g->color+1) % MAX_COLORS;
			}
			if (g->mtype!=M_NONE) {
				g->mtype=(g->mtype+1) % (M_NONE-1);
			}
			
			if (sys_test_key()==escape) break;
		}
	}
	lcd_unclip();
	
	return ry;
}

void graphic_mode ()
/***** graphic_mode
 * switch to graphic mode
 *****/
{
}

void gsubplot(int r, int c, int i);

static void gupdate(Graph *g)
{
	// update g->pxorg and g->pyorg
	if (!(g->flags & G_XLOG)) {
		g->xfactor = (real)(g->pxmax - g->pxmin) / (g->xmax - g->xmin);
		if (g->xorg>g->xmin && g->xorg<g->xmax) {
			g->pxorg = g->pxmin + (g->xorg - g->xmin)*g->xfactor;
		} else if (g->xorg<=g->xmin) {
			g->pxorg = g->pxmin;
		} else {
			g->pxorg = g->pxmax;
		}
	} else {
		g->xfactor = (real)(g->pxmax - g->pxmin) / log10(g->xmax / g->xmin);
		if (g->xorg>g->xmin && g->xorg<g->xmax) {
			g->pxorg = g->pxmin + log10(g->xorg / g->xmin)*g->xfactor;
		} else if (g->xorg<=g->xmin) {
			g->pxorg = g->pxmin;
		} else {
			g->pxorg = g->pxmax;
		}
	}
	if (!(g->flags & G_YLOG)) {
		g->yfactor = (real)(g->pymax - g->pymin) / (g->ymax - g->ymin);
		if (g->yorg>g->ymin && g->yorg<g->ymax) {
			g->pyorg = g->pymin + (g->ymax - g->yorg) * g->yfactor;
		} else if (g->yorg<=g->ymin) {
			g->pyorg = g->pymax;
		} else {
			g->pyorg = g->pymin;
		}
	} else {
		g->yfactor = (real)(g->pymax - g->pymin) / log10(g->ymax / g->ymin);
		if (g->yorg>g->ymin && g->yorg<g->ymax) {
			g->pyorg = g->pymax - log10(g->yorg /g->ymin) * g->yfactor;
		} else if (g->yorg<=g->ymin) {
			g->pyorg = g->pymax;
		} else {
			g->pyorg = g->pymin;
		}
	}
}

static int ginit(GraphWindow *gw)
{
	gw->pch=fixed12.height;
	gw->pcw=fixed12.width;

	gw->cur=0;
	gw->n=1;
	gw->rows=1;
	gw->cols=1;
	
	gsubplot(1,1,1);
	return true;
}

/* subplot callback
 * r:nb of rows, c:nb of columns, i: index of the current graph
 */
void gsubplot(int r, int c, int i)
{
	if (i==1) {
		lcd_clear_screen(WHITE);
		// setup new layout
		gw.rows=r; gw.cols=c; gw.n=r*c;
		int w=LCD_WIDTH,h=LCD_HEIGHT-3*gw.pch;
		for (int k=0; k<gw.n; k++) {
			Graph *g=gw.graph+k;
			g->pxmin=((k)%c)*w/c+6*gw.pcw;
			g->pxmax=(((k)%c)+1)*w/c-2*gw.pcw;
			g->pymin=((k)/c)*h/r+3*gw.pch/2;
			g->pymax=(((k)/c)+1)*h/r-3*gw.pch/2;
			g->xorg=0.0; g->yorg=0.0;
			g->xmin=-1.0; g->xmax=1.0; g->ymin=-1.0; g->ymax=1.0;
			g->flags=G_WORLDUNSET|G_AXISUNSET|G_AUTOSCALE;
			g->flags|=(M_NONE<<20)|(1<<28);
			g->color=0;
			g->ltype=L_SOLID;
			g->lwidth=1;
			g->mtype=M_NONE;
		}
	} else if (gw.rows!=r || gw.cols!=c) cc_error(calc,"Plot layout changed, but index not 1");
	gw.cur=i-1;
	Graph *g=gw.graph+gw.cur;
	int w=LCD_WIDTH,h=LCD_HEIGHT-3*gw.pch;
	// subplot may have been called before, so cleanup if it has.
	g->pxmin=((i-1)%c)*w/c+6*gw.pcw;
	g->pxmax=(((i-1)%c)+1)*w/c-2*gw.pcw;
	g->pymin=((i-1)/c)*h/r+3*gw.pch/2;
	g->pymax=(((i-1)/c)+1)*h/r-3*gw.pch/2;
	g->xorg=0.0; g->yorg=0.0;
	g->xmin=-1.0; g->xmax=1.0; g->ymin=-1.0; g->ymax=1.0;
	g->flags=G_WORLDUNSET|G_AXISUNSET|G_AUTOSCALE;
	g->flags|=(M_NONE<<20)|(1<<28);
	g->color=0;
	g->ltype=L_SOLID;
	g->lwidth=1;
	g->mtype=M_NONE;
}

/* setplot callback:
 * setup the limits of the plot (xmin,xmax,ymin,ymax), and flags.
 */
void gsetplot(real xmin, real xmax, real ymin, real ymax, unsigned long flags, unsigned long mask)
{
	if (gw.n) {
		Graph *g=gw.graph+gw.cur;
		// update flags
		g->flags=(g->flags & ~mask) | flags;
		if (mask & G_LTYPE_MSK) g->ltype=(flags & G_LTYPE_MSK)>>16;
		if (mask & G_MTYPE_MSK) g->mtype=(flags & G_MTYPE_MSK)>>20;
		if (mask & G_COLOR_MSK) g->color=(flags & G_COLOR_MSK)>>24;
		if (mask & G_LWIDTH_MSK) g->lwidth=(flags & G_LWIDTH_MSK)>>28;
		
		if (g->xmin!=xmin || g->xmax!=xmax || g->ymin!=ymin || g->ymax!=ymax) {
			if (g->flags & G_WORLDUNSET) {
				g->xmin=xmin; g->xmax=xmax; g->ymin=ymin; g->ymax=ymax;
				g->flags &= ~G_WORLDUNSET;
			} else if (g->flags & G_AUTOSCALE) {
				g->xmin=(xmin<g->xmin) ? xmin : g->xmin;
				g->xmax=(xmax>g->xmax) ? xmax : g->xmax;
				g->ymin=(ymin<g->ymin) ? ymin : g->ymin;
				g->ymax=(ymax>g->ymax) ? ymax : g->ymax;
			} else {
				g->xmin=xmin; g->xmax=xmax; g->ymin=ymin; g->ymax=ymax;
			}
		}
		
		if ((g->flags & G_XLOG) && g->xorg<=0.0) g->xorg=1.0;
		if ((g->flags & G_YLOG) && g->yorg<=0.0) g->yorg=1.0;
		gupdate(g);
		if (g->flags & G_AXIS) {
			lcd_draw_line(g->pxmin,g->pyorg,g->pxmax,g->pyorg,gridcolor);
			lcd_draw_line(g->pxorg,g->pymin,g->pxorg,g->pymax,gridcolor);
		}
		if (g->flags & G_FRAME) {
			lcd_draw_rect(g->pxmin,g->pymin,g->pxmax-g->pxmin+1,g->pymax-g->pymin+1,BLACK);
		}
	}
}

/* getplotlimits callback:
 * get the limits of the plot (xmin,xmax,ymin,ymax), and flags.
 */
void ggetplot(real *xmin, real *xmax, real *ymin, real *ymax, unsigned long *flags)
{
	if (gw.n) {
		Graph *g=gw.graph+gw.cur;
		*xmin=g->xmin; *xmax=g->xmax; *ymin=g->ymin; *ymax=g->ymax;
		*flags=g->flags;
	}
}

/* plot callback
 * plot the y=f(x) function
 */
void gplot(Calc *cc, header *hdx, header *hdy)
{
	if (gw.n) {
		Graph *g=gw.graph+gw.cur;
		g->ltype  = (g->flags & G_LTYPE_MSK)>>16;
		g->lwidth = (g->flags & G_LWIDTH_MSK)>>28;
		g->msize     = 6;
		int r=g_draw_plot(g,hdx,hdy);
		// to allow color rotation for each plot
		if (g->flags & G_AUTOCOLOR) {
			g->color=(g->color+r) % MAX_COLORS;
		}
		if (((g->flags & G_MTYPE_MSK)>>20)!=M_NONE) {
			g->mtype=(g->mtype+r) % (M_NONE-1);
		}
	}
}

/* gxgrid calback
 *   setup a manual grid for X axis
 */
void gsetxgrid(header *ticks, real factor, unsigned int color)
{
	if (gw.n) {
		Graph *g=gw.graph+gw.cur;
		g_xgrid(g,ticks,factor,color);
	}
}

/* gygrid calback
 *   setup a manual grid for Y axis
 */
void gsetygrid(header *ticks, real factor, unsigned int color)
{
	if (gw.n) {
		Graph *g=gw.graph+gw.cur;
		g_ygrid(g,ticks,factor,color);
	}
}

/* gtext callback
 *   draw the "text" string at position [x,y] with the defined attributes
 */
void gtext (real x, real y, char *text, unsigned int align, int angle, unsigned int color)
{
	DC dc;
	short px, py;
	
	lcd_get_default_DC(&dc);
	lcd_set_alignment(&dc,align<<4);
	
	if (gw.n) {
		Graph *g=gw.graph+gw.cur;
//		if (!(g->flags & G_AXISUNSET)) {
			if (!(g->flags & G_XLOG)) {
				g->xfactor = (real)(g->pxmax - g->pxmin) / (g->xmax - g->xmin);
				px = (short)(g->pxmin + (x - g->xmin) * g->xfactor);
			} else {
				g->xfactor = (real)(g->pxmax - g->pxmin) / log10(g->xmax / g->xmin);
				px = (short)(g->pxmin + log10(x / g->xmin) * g->xfactor);
			}
			if (!(g->flags & G_YLOG)) {
				g->yfactor = (real)(g->pymax - g->pymin) / (g->ymax - g->ymin);
				py = (short)(g->pymin + (g->ymax - y) * g->yfactor);
			} else {
				g->yfactor = (real)(g->pymax - g->pymin) / log10(g->ymax / g->ymin);
				py = (short)(g->pymax - log10(y /g->ymin) * g->yfactor);
			}
			
			lcd_draw_string(&dc,px,py,text);
//		}
	}
}

/* glabel callback
 *   draw the "text" string as a standard label according to second parameter
 */
void glabel(char *text, unsigned int type)
{
	DC dc;
	
	lcd_get_default_DC(&dc);
	lcd_set_alignment(&dc,ALIGN_S);
	
	if (type & G_TITLE) {
		lcd_draw_string(&dc,LCD_WIDTH/2,2,text);
	}
	if (gw.n) {
		Graph *g=gw.graph+gw.cur;
		if (type & G_XLABEL) {
			lcd_draw_string(&dc,(g->pxmin+g->pxmax)/2,g->pymax+2*gw.pch,text);
		}
		if (type & G_YLABEL) {
			lcd_set_direction(&dc,DIR_VERTICAL_INV);
			lcd_draw_string(&dc,g->pxmin-6*gw.pcw,(g->pymin+g->pymax)/2,text);
			lcd_set_direction(&dc,DIR_HORIZONTAL);
		}
	}

}

void gclear (void)
/***** clear the graphics screen
*****/
{
	// Color color = 0xFFFF;
	// while (!memory_available(MASK_COLOR)){}
	// memcpy((void*)COLOR_ADR, &color, sizeof(Color));
	// while (!mb_push_evt(EVT_CLEARSCREEN|MASK_COLOR,false)) ;
}

void mouse (int* x, int* y)
/****** mouse
	wait, until the user marked a screen point with the mouse.
	Return screen coordinates.
******/
{	*x=0; *y=0;
}

void getpixel (real *x, real *y)
/***** Compute the size of pixel in screen coordinates.
******/
{	*x=1;
	*y=1;
}

void gflush (void)
/***** Flush out remaining graphic commands (for multitasking).
This serves to synchronize the graphics on multitasking systems.
******/
{
}



/*******************************************************************************
 * MAIN
 ******************************************************************************/
int main ()
/******
Initialize memory and call main_loop
******/
{
	/* set BOD VBAT level to 1.65V */
	POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    BOARD_InitPins();
    BOARD_InitBootClocks();

 	/* Allocate the stack: initialize stack limit pointers */
	calc->ramstart=(char*)0x20018000;
	calc->ramend=(char*)0x20030000;
	void * core1_start_adr = (void *)0x4000000;

	/* i2c config */
	i2c_master_config_t fc4_config = {
        .enableMaster = true,
        .baudRate_Bps = 400000,  // mode Fast
        .enableTimeout = false,
		.timeout_Ms = 35
    };
	#define I2C4_MASTER_CLK 12000000
    I2C_MasterInit(I2C4, &fc4_config, I2C4_MASTER_CLK);
    mma8652_init(I2C4, MMA8652_RATE_1_56 | MMA8652_SCALE_8G | MMA8652_RES_8);

    /* UART initialization */
    uart_init(USART0,115200U);

	/* get width of the terminal */
	calc->termwidth = TERMWIDTH;

	/* clear terminal screen */
	uart_puts(USART0,"\x1b[2J\x1b[H");

	/* Enable PowerQuad Capabilities */
	PQ_Init(POWERQUAD);
	pq_config_t pq_cfg;
	PQ_GetDefaultConfig(&pq_cfg);
	PQ_SetConfig(POWERQUAD, &pq_cfg);

	/* setup sd card */
	sd_init(& card,card_detect_cb,NULL);
	
	/* set up default pathes and directory */
    OSA_TimeDelay(500);
    if (updateFS) {
    	update_sd_state();
    	updateFS=false;
    }




    /* LCD initialization */
	lcd_init();
	lcd_switch_to(LCD_DPY);
	ginit(&gw);

	// Codec init
	pcm_init();
	
	// Set systick reload value to generate 1ms interrupt
	SysTick_Config(150000);
	#ifdef MULTICORE_APP
    core1_startup(core1_start_adr);
	#endif

    // debug 
    // OLS_start();
    // UPOLS_start();

	main_loop(calc,0,NULL);

	return EXIT_SUCCESS;
}

