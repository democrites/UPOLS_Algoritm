
DDEFS +=

DRIVERS = lib/drivers

DINCDIR += $(DRIVERS)

SRC += $(DRIVERS)/fsl_clock.c \
       $(DRIVERS)/fsl_common.c \
       $(DRIVERS)/fsl_common_arm.c \
       $(DRIVERS)/fsl_ctimer.c \
       $(DRIVERS)/fsl_dma.c \
       $(DRIVERS)/fsl_flexcomm.c \
       $(DRIVERS)/fsl_gpio.c \
       $(DRIVERS)/fsl_i2c.c \
       $(DRIVERS)/fsl_i2s.c  \
       $(DRIVERS)/fsl_i2s_dma.c \
       $(DRIVERS)/fsl_power.c \
       $(DRIVERS)/fsl_powerquad_basic.c \
       $(DRIVERS)/fsl_powerquad_data.c \
       $(DRIVERS)/fsl_powerquad_filter.c \
       $(DRIVERS)/fsl_powerquad_matrix.c \
       $(DRIVERS)/fsl_powerquad_math.c \
       $(DRIVERS)/fsl_powerquad_transform.c \
       $(DRIVERS)/fsl_reset.c \
       $(DRIVERS)/fsl_sdif.c \
       $(DRIVERS)/fsl_spi.c \
       $(DRIVERS)/fsl_sysctl.c \
       $(DRIVERS)/fsl_usart.c
