
#DDEFS += -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DSERIAL_PORT_TYPE_UART=1
#DDEFS += -DSDK_DEBUGCONSOLE=0 -DSERIAL_PORT_TYPE_UART=1

UTILS = lib/utilities

DINCDIR += $(UTILS)

SRC += $(UTILS)/fsl_assert.c \
       $(UTILS)/fsl_debug_console.c \
       $(UTILS)/fsl_str.c
