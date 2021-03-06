/*
********************************************************************************
*
*      GSM AMR-NB speech codec   R98   Version 7.6.0   December 12, 2001
*                                R99   Version 3.3.0
*                                REL-4 Version 4.1.0
*
********************************************************************************
*
*      File             : gray.tab
*      Purpose          : gray encoding and decoding tables
*      $Id $
*
*
********************************************************************************
*/
#include "codec_op_etsi.h"
#include "cnst.h"

static const Word16 gray[8]  = {0, 1, 3, 2, 6, 4, 5, 7};
static const Word16 dgray[8] = {0, 1, 3, 2, 5, 6, 4, 7};
