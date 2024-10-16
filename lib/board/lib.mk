
DDEFS += 

BOARD = lib/board

DINCDIR += $(BOARD)

SRC += $(BOARD)/board.c $(BOARD)/clock_config.c \
       $(BOARD)/peripherals.c $(BOARD)/pin_mux.c
