
#ifndef _TAF_MMA_SND_IMSA_H_
#define _TAF_MMA_SND_IMSA_H_


/*****************************************************************************
  1 ����ͷ�ļ�����
*****************************************************************************/
#if (FEATURE_IMS == FEATURE_ON)
#include "ImsaMmaInterface.h"
#endif
#include "MmaMmcInterface.h"
#include "TafSdcCtx.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/*****************************************************************************
  2 �궨��
*****************************************************************************/


/*****************************************************************************
  3 ö�ٶ���
*****************************************************************************/


/*****************************************************************************
  4 ȫ�ֱ�������
*****************************************************************************/


/*****************************************************************************
  5 ��Ϣͷ����
*****************************************************************************/


/*****************************************************************************
  6 ��Ϣ����
*****************************************************************************/


/*****************************************************************************
  7 STRUCT����
*****************************************************************************/


/*****************************************************************************
  8 UNION����
*****************************************************************************/


/*****************************************************************************
  9 OTHERS����
*****************************************************************************/


/*****************************************************************************
  10 ��������
*****************************************************************************/


#if (FEATURE_IMS == FEATURE_ON)
MMA_IMSA_SERVICE_STATUS_ENUM_UINT8 TAF_MMA_ConvertMmaPsServiceStatusToImsaFormat(
    MMA_MMC_SERVICE_STATUS_ENUM_UINT32 enMmcPsServiceStatus
);

VOS_VOID TAF_MMA_SndImsaSrvInfoNotify(
    MMA_MMC_SERVICE_STATUS_ENUM_UINT32  enPsServiceStatus
);

VOS_VOID TAF_MMA_SndImsaCampInfoChangeInd(VOS_VOID);
VOS_VOID TAF_MMA_SndImsaStopReq(
    MMA_IMSA_STOP_TYPE_ENUM_UINT32      enStopType
);

VOS_UINT32 TAF_MMA_SndImsaStartReq(
    MMA_IMSA_START_TYPE_ENUM_UINT32     enStartType
);
VOS_VOID TAF_MMA_SndImsaVoiceDomainChangeInd(
    MMA_IMSA_VOICE_DOMAIN_ENUM_UINT32   enVoiceDomain
);
#if (FEATURE_MULTI_MODEM == FEATURE_ON)
VOS_VOID TAF_MMA_SndImsaModem1InfoInd(
    MMA_IMSA_MODEM_POWER_STATE_ENUM_UINT8   enModem1PowerState);
#endif

#endif

VOS_UINT32 TAF_MMA_IsCGIInfoChanged(
    TAF_SDC_CAMP_PLMN_INFO_STRU        *pstOldCampInfo,
    TAF_SDC_CAMP_PLMN_INFO_STRU        *pstNewCampInfo
);

VOS_UINT32 TAF_MMA_IsNetworkCapInfoChanged(
    TAF_SDC_NETWORK_CAP_INFO_STRU       *pstNewNwCapInfo
);


#if ((VOS_OS_VER == VOS_WIN32) || (VOS_OS_VER == VOS_NUCLEUS))
#pragma pack()
#else
#pragma pack(0)
#endif
#ifdef __cplusplus
}
#endif
/*****************************************************************************/

/*===========================================================================*/
#endif      /* __STATUS_H__*/

/***** End of the file *****/