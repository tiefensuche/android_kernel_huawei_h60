/*
********************************************************************************
*
*      GSM AMR-NB speech codec   R98   Version 7.6.0   December 12, 2001
*                                R99   Version 3.3.0
*                                REL-4 Version 4.1.0
*
********************************************************************************
*
*      File             : c2_9pf.tab
*      Purpose          : track start positions for fixed codebook routines
*                         in c2_9pf.c/d2_9pf.c
*      $Id $
*
********************************************************************************
*/
#include "codec_op_etsi.h"

static const Word16 startPos[2*4*2] = {0, 2, 0, 3,
                                       0, 2, 0, 3,
                                       1, 3, 2, 4,
                                       1, 4, 1, 4};
