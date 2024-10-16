
DDEFS += -DUSE_HS_SPI=1 -DBOARD_USE_CODEC=1 -DSDK_I2C_BASED_COMPONENT_USED=1 -DCODEC_WM8904_ENABLE

CODEC = lib/codec

DINCDIR += $(CODEC) $(CODEC)/port $(CODEC)/port/wm8904

SRC += $(CODEC)/fsl_codec_common.c $(CODEC)/fsl_codec_i2c.c \
       $(CODEC)/fsl_wm8904.c \
       $(CODEC)/port/wm8904/fsl_codec_wm8904_adapter.c
