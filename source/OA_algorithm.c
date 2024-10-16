/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_dma.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_i2s_dma.h"
#include "fsl_codec_common.h"


#include <stdbool.h>
#include "fsl_sysctl.h"
#include "fsl_wm8904.h"
#include "fsl_codec_adapter.h"
// #include "fsl_power.h"
#include "arm_const_structs.h"
#include "OA_algorithm.h"
#include "fsl_powerquad.h"
#include <math.h>

#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN __attribute__((aligned(4)))
#endif
#ifndef __ALIGN_END
#define __ALIGN_END __attribute__((aligned(4)))
#endif
#include "test_data_array.h"
#define DEBUG_PCM_LOOP 		1
// #define DEBUG_IMPULSE_RESPONSE
// #define TIM_DEBUG           1

#ifdef TIM_DEBUG
#include "fsl_ctimer.h"
#define CTIMER_CLK_FREQ CLOCK_GetCTimerClkFreq(2U)
#define CTIMER_MAT_OUT  kCTIMER_Match_1 /* Match output 1 */
#define CTIMER          CTIMER2
#endif

extern volatile int user_break;
char uart_getc(USART_Type *base);
char uart_puts(USART_Type *base, const char *s);

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C                        (I2C4)
#define I2S_MASTER_CLOCK_FREQUENCY (24576000)
#define I2S_TX                     (I2S7)
#define I2S_RX                     (I2S6)
#define DMA                        (DMA0)
#define I2S_TX_CHANNEL             (19)
#define I2S_RX_CHANNEL             (16)
#define I2S_CLOCK_DIVIDER          (CLOCK_GetPll0OutFreq() / 48000U / 16U / 2U)
#define I2S_RX_MODE                (kI2S_MasterSlaveNormalSlave)
#define I2S_TX_MODE                (kI2S_MasterSlaveNormalMaster)
#define AUDIO_BIT_WIDTH            (16)
#define AUDIO_SAMPLE_RATE          (48000)
#define AUDIO_PROTOCOL             kCODEC_BusI2S
#ifndef CODEC_VOLUME
#define CODEC_VOLUME               70U
#endif

/*************************************************************************/
#define SAMPLE_NB						256		/* nb samples per channel (must be a power of two : max 256)*/
#define BUFFER_LENGTH					(SAMPLE_NB)
#define N_CHANNEL						(2)
#define BYTE_PER_SAMPLE					(AUDIO_BIT_WIDTH / 8)
#define AUDIO_BIT_WIDTH					(16)
#define BUF_NB	                        (3)
#define SCALE_FACTOR                    (0X7F00) // almost max value : 0X7FFF
#define NEXT_ID(id) (((id)+1)<BUF_NB ? (id)+1 : 0)

// /* Algorithm variables */
#define B_ALGO		        BUFFER_LENGTH		// taille des blocs 
#define P_ALGO_MAX          4                   // max filter size B*4
#define P_ALGO_UPOLS		(int)(FILTER_LENGTH/B_ALGO)	// nombre de partitions du filtre
#define P_ALGO_OLS          (1)
#define K_ALGO		        (2*B_ALGO) 			// taille de la transformée de fourier
// #define N_ALGO		        (K_ALGO*P_ALGO) 	//longueur du filtre

/*******************************************************************************
 * Variables
 ******************************************************************************/
wm8904_config_t wm8904Config_OA = {
    .i2cConfig    = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
    .recordSource = kWM8904_RecordSourceLineInput,
    .recordChannelLeft  = kWM8904_RecordChannelLeft2,
    .recordChannelRight = kWM8904_RecordChannelRight2,
    .playSource         = kWM8904_PlaySourceDAC,
    .slaveAddress       = WM8904_I2C_ADDRESS,
    .protocol           = kWM8904_ProtocolI2S,
    .format             = {.sampleRate = kWM8904_SampleRate48kHz, .bitWidth = kWM8904_BitWidth16},
    .mclk_HZ            = I2S_MASTER_CLOCK_FREQUENCY,
    .master             = false,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8904, .codecDevConfig = &wm8904Config_OA};

static dma_handle_t s_DmaTxHandle;
static dma_handle_t s_DmaRxHandle;
static i2s_config_t s_TxConfig;
static i2s_config_t s_RxConfig;
static i2s_dma_handle_t s_TxHandle;
static i2s_dma_handle_t s_RxHandle;
static i2s_transfer_t s_TxTransfer;
static i2s_transfer_t s_RxTransfer;
extern codec_config_t boardCodecConfig;
codec_handle_t codecHandle;


/* Algorithm memory buffers */    //////////§§§§§§§§§§§§!!!!!!!!!!!!!! delete volatile if that works !!!!!!!!!!!!!!!!!!!!!
/* sliding window */
__ALIGN_BEGIN static volatile uint16_t s_SlidingWindow[N_CHANNEL*K_ALGO] __ALIGN_END;
// /* frequency domain output buffer / time domain output buffer*/
__ALIGN_BEGIN static volatile uint16_t s_Acumulator[N_CHANNEL*K_ALGO] __ALIGN_END ;
// /* Frequency delay Line */
__ALIGN_BEGIN static volatile uint16_t s_FDL[K_ALGO*N_CHANNEL*P_ALGO_MAX] __ALIGN_END;
// /* Filter */
__ALIGN_BEGIN static volatile uint16_t s_Filter[B_ALGO*P_ALGO_MAX*4] __ALIGN_END;

/* ping-pong rx buffers : (2 channels x SAMPLE_NB samples x sizeof(int16_t))
 * data: buffer address
 * dataSize: buffer size in bytes
 */
static volatile int pcm_rx_buf_done=0;
__ALIGN_BEGIN static int16_t pcm_rx_buf[SAMPLE_NB*6]__ALIGN_END;		/* 3 buffers with 2 channels with SAMPLE_NB samples */
static i2s_transfer_t pcm_rx_xfer[3] = {
	[0] = {
		.data     = (uint8_t*)pcm_rx_buf,
		.dataSize = 2*SAMPLE_NB*sizeof(int16_t)
	},
	[1] = {
		.data     = (uint8_t*)(pcm_rx_buf+2*SAMPLE_NB),
		.dataSize = 2*SAMPLE_NB*sizeof(int16_t)
	},
	[2] = {
		.data     = (uint8_t*)(pcm_rx_buf+4*SAMPLE_NB),
		.dataSize = 2*SAMPLE_NB*sizeof(int16_t)
	}
};

/* ping-pong tx buffers : (2 channels x SAMPLE_NB samples x sizeof(int16_t))
 * data: buffer address
 * dataSize: buffer size in bytes
 */
static volatile int pcm_tx_buf_done=0;
__ALIGN_BEGIN static int16_t pcm_tx_buf[SAMPLE_NB*6]__ALIGN_END;		/* 3 buffers with 2 channels with SAMPLE_NB samples */
static i2s_transfer_t pcm_tx_xfer[3] = {
	[0] = {
		.data     = (uint8_t*)pcm_tx_buf,
		.dataSize = 2*SAMPLE_NB*sizeof(int16_t)
	},
	[1] = {
		.data     = (uint8_t*)(pcm_tx_buf+2*SAMPLE_NB),
		.dataSize = 2*SAMPLE_NB*sizeof(int16_t)
	},
	[2] = {
		.data     = (uint8_t*)(pcm_tx_buf+4*SAMPLE_NB),
		.dataSize = 2*SAMPLE_NB*sizeof(int16_t)
	}
};

DMA_ALLOCATE_HEAD_DESCRIPTORS(dmaLinkDescriptorTableRx, BUF_NB);
DMA_ALLOCATE_HEAD_DESCRIPTORS(dmaLinkDescriptorTableTx, BUF_NB);

uint16_t *s_SlidingWindow1= (uint16_t*) &s_SlidingWindow[0]; //[BUFFER_LENGTH/2]
uint16_t *s_SlidingWindow2 = (uint16_t*) &s_SlidingWindow[BUFFER_LENGTH*2]; // lenght: BUFFER_LENGHT / 2

static pq_config_t PQ_config_fft;
static pq_prescale_t PQ_CFFT_prescale;
static int P_algo=0;

static volatile uint8_t FDLIndex = 0;
static volatile int pcm_buf_for_rxqueue=NEXT_ID(1);

static void audio_overlapp_add_convolution_stereo(int16_t* data_in, int16_t* data_out);
static void cmplx_mult_cmplx_int16(const int16_t * pSrcA,const int16_t * pSrcB,int16_t * pDst,uint32_t numSamples);

#ifdef TIM_DEBUG
void TIMER0_Init(void){
    /**************** TIMER INIT DEBUG ****************/
	ctimer_config_t config;
	ctimer_match_config_t matchConfig;
	/* Use FRO_HF clock as input clock source. */
	CLOCK_AttachClk(kFRO_HF_to_CTIMER2);
	CTIMER_GetDefaultConfig(&config);
	config.prescale = CTIMER_CLK_FREQ/1000000;

	CTIMER_Init(CTIMER, &config);

	matchConfig.enableCounterReset = true;
	matchConfig.enableCounterStop  = false;
	matchConfig.matchValue         = CTIMER_CLK_FREQ / 2;
	matchConfig.outControl         = kCTIMER_IncreaseOnRiseEdge;
	matchConfig.outPinInitState    = false;
	matchConfig.enableInterrupt    = false;
	CTIMER_SetupMatch(CTIMER, CTIMER_MAT_OUT, &matchConfig);
	CTIMER->TCR|= 0b11;
}
void TIMER_Start(void){
	CTIMER->TCR&= ~0b10; // start counter
}
// return the time in us after TIMER_Start
uint32_t TIMER_GetValue(void){
	uint32_t val = CTIMER2->TC;
	CTIMER->TCR|= 0b10; // set the counter in reset state
	return val;
}
#endif // TIM_DEBUG

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_InitSysctrl(void)
{
    SYSCTL_Init(SYSCTL);
    /* select signal source for share set */
    SYSCTL_SetShareSignalSrc(SYSCTL, kSYSCTL_ShareSet0, kSYSCTL_SharedCtrlSignalSCK, kSYSCTL_Flexcomm7);
    SYSCTL_SetShareSignalSrc(SYSCTL, kSYSCTL_ShareSet0, kSYSCTL_SharedCtrlSignalWS, kSYSCTL_Flexcomm7);
    /* select share set for special flexcomm signal */
    SYSCTL_SetShareSet(SYSCTL, kSYSCTL_Flexcomm7, kSYSCTL_FlexcommSignalSCK, kSYSCTL_ShareSet0);
    SYSCTL_SetShareSet(SYSCTL, kSYSCTL_Flexcomm7, kSYSCTL_FlexcommSignalWS, kSYSCTL_ShareSet0);
    SYSCTL_SetShareSet(SYSCTL, kSYSCTL_Flexcomm6, kSYSCTL_FlexcommSignalSCK, kSYSCTL_ShareSet0);
    SYSCTL_SetShareSet(SYSCTL, kSYSCTL_Flexcomm6, kSYSCTL_FlexcommSignalWS, kSYSCTL_ShareSet0);
}

static void configure_codec(void)
{
    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* USART0 clock */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* I2C clock */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);

    PMC->PDRUNCFGCLR0 |= PMC_PDRUNCFG0_PDEN_XTAL32M_MASK;   /*!< Ensure XTAL16M is on  */
    PMC->PDRUNCFGCLR0 |= PMC_PDRUNCFG0_PDEN_LDOXO32M_MASK;  /*!< Ensure XTAL16M is on  */
    SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK; /*!< Ensure CLK_IN is on  */
    ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_ENABLE_SYSTEM_CLK_OUT_MASK;

    /*!< Switch PLL0 clock source selector to XTAL16M */
    CLOCK_AttachClk(kEXT_CLK_to_PLL0);

    const pll_setup_t pll0Setup = {
        .pllctrl = SYSCON_PLL0CTRL_CLKEN_MASK | SYSCON_PLL0CTRL_SELI(8U) | SYSCON_PLL0CTRL_SELP(31U),
        .pllndec = SYSCON_PLL0NDEC_NDIV(125U), // to do  : change with the frequency
        .pllpdec = SYSCON_PLL0PDEC_PDIV(8U),
        .pllsscg = {0x0U, (SYSCON_PLL0SSCG1_MDIV_EXT(3072U) | SYSCON_PLL0SSCG1_SEL_EXT_MASK)},
        .pllRate = 24576000U, // to do : change with the frequency
        .flags   = PLL_SETUPFLAG_WAITLOCK,
    };
    /*!< Configure PLL to the desired values */
    CLOCK_SetPLL0Freq(&pll0Setup);

    CLOCK_SetClkDiv(kCLOCK_DivPll0Clk, 0U, true);
    CLOCK_SetClkDiv(kCLOCK_DivPll0Clk, 1U, false);

    /* I2S clocks */
    CLOCK_AttachClk(kPLL0_DIV_to_FLEXCOMM6);
    CLOCK_AttachClk(kPLL0_DIV_to_FLEXCOMM7);

    /* Attach PLL clock to MCLK for I2S, no divider */
    CLOCK_AttachClk(kPLL0_to_MCLK);
    SYSCON->MCLKDIV = SYSCON_MCLKDIV_DIV(0U);
    SYSCON->MCLKIO  = 1U;

    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);

    /* reset FLEXCOMM for DMA0 */
    RESET_PeripheralReset(kDMA0_RST_SHIFT_RSTn);

    /* reset FLEXCOMM for I2S */
    RESET_PeripheralReset(kFC6_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);

    /* reset NVIC for FLEXCOMM6 and FLEXCOMM7 */
    NVIC_ClearPendingIRQ(FLEXCOMM6_IRQn);
    NVIC_ClearPendingIRQ(FLEXCOMM7_IRQn);

    /* Enable interrupts for I2S */
    EnableIRQ(FLEXCOMM6_IRQn);
    EnableIRQ(FLEXCOMM7_IRQn);

    /* Initialize the rest */
    BOARD_InitBootPins();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();
    BOARD_InitSysctrl();

     PRINTF("Configure codec\r\n");

    /* protocol: i2s
     * sampleRate: 48K
     * bitwidth:16
     */
    if (CODEC_Init(&codecHandle, &boardCodecConfig) != kStatus_Success)
    {
        PRINTF("codec_Init failed!\r\n");
        assert(false);
    }
    /* Initial volume kept low for hearing safety.
     * Adjust it to your needs, 0-100, 0 for mute, 100 for maximum volume.
     */
    if (CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight,
                        CODEC_VOLUME) != kStatus_Success)
    {
        assert(false);
    }

    PRINTF("Configure I2S\r\n");

    /*
     * masterSlave = kI2S_MasterSlaveNormalMaster;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = true;
     * pack48 = false;
     */
    I2S_TxGetDefaultConfig(&s_TxConfig);
    s_TxConfig.divider     = I2S_CLOCK_DIVIDER;
    s_TxConfig.masterSlave = I2S_TX_MODE;

    /*
     * masterSlave = kI2S_MasterSlaveNormalSlave;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = false;
     * pack48 = true;
     */
    I2S_RxGetDefaultConfig(&s_RxConfig);
    s_RxConfig.divider     = I2S_CLOCK_DIVIDER;
    s_RxConfig.masterSlave = I2S_RX_MODE;

    I2S_TxInit(I2S_TX, &s_TxConfig);
    I2S_RxInit(I2S_RX, &s_RxConfig);

    DMA_Init(DMA);

    DMA_EnableChannel(DMA, I2S_TX_CHANNEL);
    DMA_EnableChannel(DMA, I2S_RX_CHANNEL);
    DMA_SetChannelPriority(DMA, I2S_TX_CHANNEL, kDMA_ChannelPriority3);
    DMA_SetChannelPriority(DMA, I2S_RX_CHANNEL, kDMA_ChannelPriority2);
    DMA_CreateHandle(&s_DmaTxHandle, DMA, I2S_TX_CHANNEL);
    DMA_CreateHandle(&s_DmaRxHandle, DMA, I2S_RX_CHANNEL);

    // StartDigitalLoopback();
}

static void POWERQUAD_init(void){
     PQ_GetDefaultConfig(&PQ_config_fft);
    PQ_config_fft.inputAFormat = kPQ_16Bit;
    PQ_config_fft.inputAPrescale = 9; // 1<<9 = 512 = fftsize
    PQ_config_fft.inputBFormat = kPQ_16Bit;
    PQ_config_fft.inputBPrescale = 0;
    PQ_config_fft.tmpFormat = kPQ_16Bit;
    PQ_config_fft.tmpPrescale = 0;
    PQ_config_fft.outputFormat = kPQ_16Bit;
    PQ_config_fft.outputPrescale = 0;
    PQ_config_fft.machineFormat= kPQ_32Bit;

    PQ_CFFT_prescale.inputPrescale = 0;
    PQ_CFFT_prescale.outputPrescale= 0;

    PQ_SetCoprocessorScaler(POWERQUAD, &PQ_CFFT_prescale);
}

#define io_set(on) (GPIO_PinWrite(GPIO, 1, 9, (on) ? 1 : 0))


/* pcm_loop
 *   use DMA0 to get a buffer of n samples (2 ways) from I2S6_Rx connected to
 *   the audio device input line
 *****/

static void loop_rx_cb(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	pcm_rx_buf_done=1;
	/* setup next buffer for reception */
    I2S_RxTransferReceiveDMA(I2S6, &s_RxHandle, pcm_rx_xfer[pcm_buf_for_rxqueue]);
    pcm_buf_for_rxqueue= NEXT_ID(pcm_buf_for_rxqueue);
}

// static void loop_tx_cb(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
// {
// 	pcm_tx_buf_done=1;
// }


static void configure_loop_DMA_I2S(void){

	DMA_Init(DMA0);
	DMA_EnableChannel(DMA0, 16);
	DMA_SetChannelPriority(DMA0, 16, kDMA_ChannelPriority2);
	DMA_CreateHandle(&s_DmaRxHandle, DMA0, 16);
	DMA_EnableChannel(DMA0, 19);
	DMA_SetChannelPriority(DMA0, 19, kDMA_ChannelPriority3);
	DMA_CreateHandle(&s_DmaTxHandle, DMA0, 19);

    I2S_RxTransferCreateHandleDMA(I2S6,
                                   &s_RxHandle,
                                   &s_DmaRxHandle,
                                   loop_rx_cb,
                                   NULL);
    I2S_TxTransferCreateHandleDMA(I2S7,
                                   &s_TxHandle,
                                   &s_DmaTxHandle,
                                   NULL,
                                   NULL);
    I2S_TransferInstallLoopDMADescriptorMemory(&s_RxHandle,
                                                (void*)dmaLinkDescriptorTableRx,
                                                BUF_NB);
    I2S_TransferInstallLoopDMADescriptorMemory(&s_TxHandle,
                                                (void*)dmaLinkDescriptorTableTx,
                                                BUF_NB);
    /* we need to queue two receive buffers so when the first one
     * is full, the other immediately is starting to be filled */
    I2S_RxTransferReceiveDMA(I2S6, &s_RxHandle, pcm_rx_xfer[0]);
    I2S_RxTransferReceiveDMA(I2S6, &s_RxHandle, pcm_rx_xfer[1]);
    I2S_TxTransferSendDMA(I2S7, &s_TxHandle, pcm_tx_xfer[0]);
    I2S_TxTransferSendDMA(I2S7, &s_TxHandle, pcm_tx_xfer[1]);
    // I2S_TransferReceiveLoopDMA(I2S6,
    //                                  &s_RxHandle,
    //                                 pcm_rx_xfer,
    //                                 3);
    // I2S_TransferReceiveLoopDMA(I2S7,
    //                                  &s_TxHandle,
    //                                 pcm_tx_xfer,
    //                                 3);
}

/* Algorithme to filter 512 coeficients max efficiently using POWERQUAD FFT */ 
// problem :: déphasage entre chaque blocs ?????
int OLS_start(void)
{
    // int pcm_buf_id=0;

    // pcm_rx_buf_done=0;

    // configure_codec();

    // // set tx buffer empty
    // for (int i=0; i<4*SAMPLE_NB; ++i) {
    //     pcm_tx_buf[i]=0;
    // }

    // configure_loop_DMA_I2S();

    // // configure POWERQUAD FFT
    // // fft config
    // POWERQUAD_init();

    // #ifdef TIM_DEBUG
    // TIMER0_Init();
    // #endif
    // P_algo = P_ALGO_OLS;

    // // copy the filter into complex vector : s_Filter
    // const int16_t *ptrSrc = Impulse_Response;
    // int16_t *ptrDst = (int16_t*)s_Filter;
    // while (ptrSrc < &Impulse_Response[(int)FILTER_LENGTH]){
    //     *ptrDst++ = *ptrSrc++;  // real part
    //     *ptrDst++ = 0;          // imaginary part = 0 if it was not the case
    // }   
    // memset(ptrDst, 0,(K_ALGO-FILTER_LENGTH)*2); // zero padding of the filter blocs

    // // compute the fft of the impulse responce blocs
    // PQ_SetConfig(POWERQUAD, &PQ_config_fft);    
    // PQ_TransformCFFT(POWERQUAD, K_ALGO, (void*)s_Filter , (void*)s_Filter); // perform fft of Filter blocs K length
    // PQ_WaitDone(POWERQUAD);

    // // invert the imaginary part
    // int16_t *ptr = s_Filter;
    // while (ptr != s_Filter){
    //     ptr++;
    //     *ptr++=-*ptr;
    // }

    // #ifdef DEBUG_PCM_LOOP
    // int test_data_index = 0;
    // #endif
    // while (1) {
    //     // #define P_ALGO      1
        
    //     while (!pcm_rx_buf_done && !user_break) {}
    //     pcm_rx_buf_done=0;

    //     #ifndef DEBUG_PCM_LOOP
    //     // if (user_break) {        // a key was pressed, so stop and quit
    //     //  char c=uart_getc(USART0);
    //     //  if (c=='+') {
    //     //      pcmvol[0]=(pcmvol[0]<57) ? pcmvol[0]+1 : 57;
    //     //      pcmvol[1]=(pcmvol[1]<57) ? pcmvol[1]+1 : 57;
    //     //  } else if (c=='-') {
    //     //      pcmvol[0]=(pcmvol[0]>0) ? pcmvol[0]-1 : 0;
    //     //      pcmvol[1]=(pcmvol[1]>0) ? pcmvol[1]-1 : 0;
    //     //  } else break;
    //     //  WM8904_SetMute(&wm8904Handle, !pcmvol[0], !pcmvol[1]);
    //     //  WM8904_SetVolume(&wm8904Handle, pcmvol[0], pcmvol[1]);
    //     //  continue;
    //     // }
    //     io_set(1);
    //     /* process data here */
    //     int16_t *din=(int16_t*)pcm_rx_xfer[pcm_buf_id].data;
    //     #endif
    //     #ifdef DEBUG_PCM_LOOP
    //     int16_t *din = &test_array_data[test_data_index*K_ALGO];
    //     test_data_index=(test_data_index +1)%2;
    //     #endif
    //     int16_t *dout=(int16_t*)pcm_tx_xfer[pcm_buf_id].data;

    //     #ifdef TIM_DEBUG
    //     TIMER_Start();
    //     #endif
    //     audio_overlapp_add_convolution_stereo(din, dout);
    //     #ifdef TIM_DEBUG
    //     uint32_t var = TIMER_GetValue();
    //     if (var>5000){
    //         assert(false); // computing time too long !!!
    //     }
    //     #endif
        
    //     /* output processed data */
    //     I2S_TxTransferSendDMA(I2S7, &s_TxHandle, pcm_tx_xfer[pcm_buf_id]);
    //     pcm_buf_id=NEXT_ID(pcm_buf_id);

    //     io_set(0);
    // }

    // I2S_TransferAbortDMA(I2S6, &s_RxHandle);
    // I2S_TransferAbortDMA(I2S7, &s_TxHandle);
    return 1;
} 

int UPOLS_start(real *m, int r, int c)
{
	int pcm_buf_id=2;

    pcm_rx_buf_done=0;

    configure_codec();

    // configure POWERQUAD FFT    
    POWERQUAD_init();

    // setup the DMA for the sliding window transfert 
    // Sliding window dma transfer 
    // dma_channel_config_t SWtransferConfig;

    // DMA_CreateHandle(&s_DmaSlidingWindowHandle, DMA1, DMA_SW_CHANNEL);
    // DMA_EnableChannel(DMA1, DMA_SW_CHANNEL);
    // DMA_SetChannelPriority(DMA1, DMA_SW_CHANNEL, kDMA_ChannelPriority4);
    // DMA_SetCallback(&s_DmaSlidingWindowHandle, NULL, NULL);
    // DMA_PrepareChannelTransfer(&SWtransferConfig, s_SlidingWindow2, s_SlidingWindow1,
    //                        		DMA_CHANNEL_XFER(false, false, true, false,
    //                           					kDMA_Transfer32BitWidth, kDMA_AddressInterleave1xWidth,
    //                                             kDMA_AddressInterleave1xWidth, BUFFER_LENGTH),
    //                         					kDMA_MemoryToMemory, NULL, NULL);
    // DMA_SubmitChannelTransfer(&s_DmaSlidingWindowHandle, &SWtransferConfig);

    #ifdef TIM_DEBUG
    TIMER0_Init();
    #endif
    #ifdef DEBUG_IMPULSE_RESPONSE
    P_algo = P_ALGO_UPOLS;
    #else
    // store matrix len
    int len = c;
    P_algo = (int)(len/B_ALGO);
    #endif

    // copy the filter into complex vector : s_Filter
    #ifdef DEBUG_IMPULSE_RESPONSE
    const int16_t *ptrSrc = Impulse_Response;
    int16_t *ptrDst = (int16_t*)s_Filter;
    for (int i=1; i<=P_algo;i++){
        while (ptrSrc < &Impulse_Response[B_ALGO*i]){
            *ptrDst++ = *ptrSrc++;  // real part
            *ptrDst++ = 0;          // imaginary part = 0 if it was not the case
        }   
        memset(ptrDst, 0,B_ALGO*4); // zero padding of the filter blocs
        ptrDst += B_ALGO*2;
    }
    // compute the fft of the impulse responce blocs
    PQ_SetConfig(POWERQUAD, &PQ_config_fft);
    for (int i=0; i<P_algo;i++){
        PQ_TransformCFFT(POWERQUAD, K_ALGO, (void*)&s_Filter[i*B_ALGO*2*2] , (void*)&s_Filter[i*B_ALGO*2*2]); // perform fft of Filter blocs K length
        PQ_WaitDone(POWERQUAD);
    }
    #else // normalize and preprocess the input filter
    float upscale = 1000;
    for(int a=0; a<3; a++){ // go for three passes to get the optimal normalisation
        const float *ptrSrc = m;
        int16_t *ptrDst = (int16_t*)s_Filter;
        for (int i=1; i<=P_algo; i++){
            int limit = len > B_ALGO*i ? B_ALGO*i : len;
            while (ptrSrc < &m[limit]){
                *ptrDst++ = (int16_t)(
                    (*ptrSrc++)*upscale);
                *ptrDst++ = 0;
            }
            if (len > (B_ALGO*i)){
                memset(ptrDst, 0, 4*B_ALGO); // zero padding of the filter blocs

            }else{
                memset(ptrDst, 0,4*B_ALGO + 4*(len-B_ALGO*i)); // zero padding of the filter blocs
            }
            ptrDst = &s_Filter[i*2*K_ALGO];
        }
        // compute the fft of the impulse responce blocs
        PQ_SetConfig(POWERQUAD, &PQ_config_fft);
        for (int i=0; i<P_algo;i++){
            PQ_TransformCFFT(POWERQUAD, K_ALGO, (void*)&s_Filter[i*B_ALGO*2*2] , (void*)&s_Filter[i*B_ALGO*2*2]); // perform fft of Filter blocs K length
            PQ_WaitDone(POWERQUAD);
        }
        // get the maximum module of one block
        int max =0;
        for (int i=0; i<P_algo; i++){
            int16_t *ptrSrc = &s_Filter[i*B_ALGO*2*2];
            int16_t a = *ptrSrc++;
            int16_t b = *ptrSrc++;
            max = pow(a, 2)+pow(b,2)>max ? pow(a, 2)+pow(b,2) : max;
        }
        max = sqrt(max);
        upscale = upscale/max *(float)(SCALE_FACTOR);
    }
    #endif

    // set tx buffer empty
    for (int i=0; i<4*SAMPLE_NB; ++i) {
        pcm_tx_buf[i]=0;
    }
    FDLIndex = 0;
    configure_loop_DMA_I2S();

    #ifdef DEBUG_PCM_LOOP
    int test_data_index = 0;
    #endif
    while (1) {
    	while (!pcm_rx_buf_done && !user_break) {}
    	pcm_rx_buf_done=0;
        #ifndef DEBUG_PCM_LOOP
    	if (user_break) {		// a key was pressed, so stop and quit
    		char c=uart_getc(USART0);
            break;
    		// if (c=='+') {
    		// 	pcmvol[0]=(pcmvol[0]<57) ? pcmvol[0]+1 : 57;
    		// 	pcmvol[1]=(pcmvol[1]<57) ? pcmvol[1]+1 : 57;
    		// } else if (c=='-') {
    		// 	pcmvol[0]=(pcmvol[0]>0) ? pcmvol[0]-1 : 0;
    		// 	pcmvol[1]=(pcmvol[1]>0) ? pcmvol[1]-1 : 0;
    		// } else break;
    		// WM8904_SetMute(&wm8904Handle, !pcmvol[0], !pcmvol[1]);
			// WM8904_SetVolume(&wm8904Handle, pcmvol[0], pcmvol[1]);
			continue;
    	}
    	io_set(1);
       	/* process data here */
    	int16_t *din=(int16_t*)pcm_rx_xfer[pcm_buf_id].data;
    	#endif
    	#ifdef DEBUG_PCM_LOOP
    	int16_t *din = &test_array_data[test_data_index*K_ALGO];
        test_data_index=(test_data_index +1)%3;
       	#endif
    	int16_t *dout=(int16_t*)pcm_tx_xfer[pcm_buf_id].data;

    	// for (int k=0;k<2*SAMPLE_NB;++k) {
    	// 	*dout++=*din++;
    	// }

    	#ifdef TIM_DEBUG
    	TIMER_Start();
    	#endif
    	audio_overlapp_add_convolution_stereo(din, dout);
    	#ifdef TIM_DEBUG
    	uint32_t var = TIMER_GetValue();
        if (var>5000){
            assert(false); // computing time too long !!!
        }
    	#endif
    	
       	/* output processed data */
        I2S_TxTransferSendDMA(I2S7, &s_TxHandle, pcm_tx_xfer[pcm_buf_id]);
    	pcm_buf_id=NEXT_ID(pcm_buf_id);

       	io_set(0);
    }

    I2S_TransferAbortDMA(I2S6, &s_RxHandle);
    I2S_TransferAbortDMA(I2S7, &s_TxHandle);
	return 1;
}

#if 0
void audio_overlapp_add_convolution_stereo(int16_t* data_in, int16_t* data_out){
	memcpy(s_SlidingWindow2, data_in, BUFFER_LENGTH*N_CHANNEL*BYTE_PER_SAMPLE); // copy buffer into sliding window
	memcpy(data_out,s_SlidingWindow2,
	    BUFFER_LENGTH*N_CHANNEL*BYTE_PER_SAMPLE); // copy Accumulator into Buffer -> TODO : change to dma transfer

}
#endif
#if 1
void audio_overlapp_add_convolution_stereo(int16_t* data_in, int16_t* data_out){
	memset((void*)s_Acumulator, 0, K_ALGO*2*2); // clear accumulator

    memcpy(s_SlidingWindow1, s_SlidingWindow2, BUFFER_LENGTH*N_CHANNEL*BYTE_PER_SAMPLE); // slide window
	memcpy(s_SlidingWindow2, data_in, BUFFER_LENGTH*N_CHANNEL*BYTE_PER_SAMPLE); // copy buffer into sliding window

    uint16_t * FDL = &s_FDL[K_ALGO*FDLIndex*N_CHANNEL];
    
    PQ_config_fft.inputAPrescale = 0;
    PQ_SetConfig(POWERQUAD, &PQ_config_fft);
    PQ_TransformCFFT(POWERQUAD, K_ALGO, (void*)s_SlidingWindow, (void*)FDL); // perform fft of sliding window K length
    PQ_WaitDone(POWERQUAD);

    //DMA_StartTransfer(&s_DmaSlidingWindowHandle); // slide window
    // memcpy(s_SlidingWindow1, s_SlidingWindow2, BUFFER_LENGTH*N_CHANNEL*BYTE_PER_SAMPLE);

    // complex multiplications :
    int loopcnt = 0;
    while (loopcnt < P_algo){
        int idx = (FDLIndex - loopcnt) % P_algo;  // Adjust index calculation
        if (idx < 0) {
                idx += P_algo;  // Handle negative modulus to keep it within bounds
            }
    	cmplx_mult_cmplx_int16((const int16_t*)&s_FDL[K_ALGO*N_CHANNEL*idx],
	    	(const int16_t*)&s_Filter[loopcnt*K_ALGO*2],
	    	(int16_t*)s_Acumulator, K_ALGO); // 110 us
    	loopcnt++;
    }

    FDLIndex = (FDLIndex + 1)%P_algo; // increment FDLIndex
    PQ_config_fft.inputAPrescale = 9;
    PQ_SetConfig(POWERQUAD, &PQ_config_fft);
    PQ_TransformIFFT(POWERQUAD, K_ALGO, (void*)s_Acumulator, (void*)s_Acumulator); // perform K-LENGHT complex IFFT
    PQ_WaitDone(POWERQUAD); // 8us
    memcpy((void*)data_out,
	    (void*)&s_Acumulator[BUFFER_LENGTH*N_CHANNEL],
	    BUFFER_LENGTH*N_CHANNEL*BYTE_PER_SAMPLE); // copy Accumulator into Buffer -> TODO : change to dma transfer
    // memcpy(data_out, data_in, BUFFER_LENGTH*N_CHANNEL*BYTE_PER_SAMPLE);
}
#endif
#define CMPLX_MULT_SHIFT 16
#pragma GCC optimize ("O3") // use maximum optimisation for this function
static void cmplx_mult_cmplx_int16(
  const int16_t * pSrcA,
  const int16_t * pSrcB,
        int16_t * pDst,
        uint32_t numSamples)
{
    uint32_t blkCnt;                               /* Loop counter */
    int16_t a, b, c, d;                              /* Temporary variables */

	/* Initialize blkCnt with number of samples */
	blkCnt = numSamples>>2U;

	while (blkCnt > 0U)
	{
	  /* C[2 * i    ] = A[2 * i] * B[2 * i    ] - A[2 * i + 1] * B[2 * i + 1]. */
	  /* C[2 * i + 1] = A[2 * i] * B[2 * i + 1] + A[2 * i + 1] * B[2 * i    ]. */

	  a = *pSrcA++;
	  b = *pSrcA++;
	  c = *pSrcB++;
	  d = *pSrcB++;

	  /* store result in destination buffer. */
	  *pDst++ += (int16_t) ((((int32_t) a * c) >> CMPLX_MULT_SHIFT) - (((int32_t) b * d) >> CMPLX_MULT_SHIFT) );
	  *pDst++ += (int16_t) ((((int32_t) a * d) >> CMPLX_MULT_SHIFT) + (((int32_t) b * c) >> CMPLX_MULT_SHIFT) );

	  a = *pSrcA++;
	  b = *pSrcA++;
	  c = *pSrcB++;
	  d = *pSrcB++;

	  /* store result in destination buffer. */
	  *pDst++ += (int16_t) ((((int32_t) a * c) >> CMPLX_MULT_SHIFT) - (((int32_t) b * d) >> CMPLX_MULT_SHIFT) );
	  *pDst++ += (int16_t) ((((int32_t) a * d) >> CMPLX_MULT_SHIFT) + (((int32_t) b * c) >> CMPLX_MULT_SHIFT) );

	  a = *pSrcA++;
	  b = *pSrcA++;
	  c = *pSrcB++;
	  d = *pSrcB++;

	  /* store result in destination buffer. */
	  *pDst++ += (int16_t) ((((int32_t) a * c) >> CMPLX_MULT_SHIFT) - (((int32_t) b * d) >> CMPLX_MULT_SHIFT) );
	  *pDst++ += (int16_t) ((((int32_t) a * d) >> CMPLX_MULT_SHIFT) + (((int32_t) b * c) >> CMPLX_MULT_SHIFT) );

	  a = *pSrcA++;
	  b = *pSrcA++;
	  c = *pSrcB++;
	  d = *pSrcB++;

	  /* store result in destination buffer. */
	  *pDst++ += (int16_t) ((((int32_t) a * c) >> CMPLX_MULT_SHIFT) - (((int32_t) b * d) >> CMPLX_MULT_SHIFT) );
	  *pDst++ += (int16_t) ((((int32_t) a * d) >> CMPLX_MULT_SHIFT) + (((int32_t) b * c) >> CMPLX_MULT_SHIFT) );

	  /* Decrement loop counter */
	  blkCnt--;
	}
}
