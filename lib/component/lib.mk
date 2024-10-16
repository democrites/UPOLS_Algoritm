
DDEFS += -DSDK_OS_BAREMETAL -DFSL_RTOS_BM

COMPONENT = lib/component

DINCDIR += $(COMPONENT)/i2c $(COMPONENT)/lists $(COMPONENT)/osa \
           $(COMPONENT)/serial_manager $(COMPONENT)/uart

SRC += $(COMPONENT)/i2c/fsl_adapter_flexcomm_i2c.c \
       $(COMPONENT)/lists/fsl_component_generic_list.c \
       $(COMPONENT)/osa/fsl_os_abstraction_bm.c \
       $(COMPONENT)/serial_manager/fsl_component_serial_manager.c \
       $(COMPONENT)/serial_manager/fsl_component_serial_port_uart.c \
       $(COMPONENT)/uart/fsl_adapter_usart.c

