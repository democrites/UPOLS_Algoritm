
DDEFS +=

SDMMC = lib/sdmmc

DINCDIR += $(SDMMC)

SRC+= $(SDMMC)/fsl_sd.c $(SDMMC)/fsl_sdmmc_common.c \
      $(SDMMC)/fsl_sdmmc_host.c $(SDMMC)/fsl_sdmmc_osa.c
