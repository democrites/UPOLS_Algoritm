#ifndef _OA_ALGORITHM_H_
#define _OA_ALGORITHM_H_

#include "sysdep.h"
#include "calc.h"
#include "stack.h"

#include "fsl_wm8904.h"
#include "fsl_codec_common.h"
#include "fsl_codec_adapter.h"
#include "fsl_sysctl.h"
#include "fsl_i2s.h"
#include "fsl_i2s_dma.h"

int UPOLS_start(real *m, int r, int c);
int OLS_start(void);

#endif
