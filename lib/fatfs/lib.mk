
DDEFS += -DSD_ENABLED

FATFS = lib/fatfs

DINCDIR += $(FATFS)

SRC += $(FATFS)/diskio.c \
       $(FATFS)/ff.c $(FATFS)/ffsystem.c $(FATFS)/ffunicode.c
