
#ifndef __MLIB_REGISTER_PROFILE4_INC__
#define __MLIB_REGISTER_PROFILE4_INC__

#include "mlib_macro.inc"

/* �ⲿģ��ͷ�ļ����� */
#include "module_default.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/*****************************************************************************
  ����: MLIB_PATH_CS_VOICE_CALL
*****************************************************************************/
MLIB_REGISTER_PATH_MODULE_BEGIN(MLIB_PROFILE4,MLIB_PATH_CS_VOICE_CALL_MICIN)
    /* ģ��ע��˵��: ģ��ID|ģ���ʼ��|ģ���������|ģ�������ȡ|ģ�����ݴ���|ģ����� */
    MLIB_REGISTER_MODULE(
                MLIB_MODULE_DEFAULT,
                NULL,
                NULL,
                NULL,
                MLIB_DEFAULT_Proc,
                NULL)
MLIB_REGISTER_PATH_MODULE_END()

MLIB_REGISTER_PATH_MODULE_BEGIN(MLIB_PROFILE4,MLIB_PATH_CS_VOICE_CALL_SPKOUT)
    /* ģ��ע��˵��: ģ��ID|ģ���ʼ��|ģ���������|ģ�������ȡ|ģ�����ݴ���|ģ����� */
    MLIB_REGISTER_MODULE(
                MLIB_MODULE_DEFAULT,
                NULL,
                NULL,
                NULL,
                MLIB_DEFAULT_Proc,
                NULL)
MLIB_REGISTER_PATH_MODULE_END()

/*****************************************************************************
  ����: MLIB_PATH_VOIP_CALL
*****************************************************************************/
MLIB_REGISTER_PATH_MODULE_BEGIN(MLIB_PROFILE4,MLIB_PATH_VOIP_CALL_MICIN)
    /* ģ��ע��˵��: ģ��ID|ģ���ʼ��|ģ���������|ģ�������ȡ|ģ�����ݴ���|ģ����� */
    MLIB_REGISTER_MODULE(
                MLIB_MODULE_DEFAULT,
                NULL,
                NULL,
                NULL,
                MLIB_DEFAULT_Proc,
                NULL)
MLIB_REGISTER_PATH_MODULE_END()

MLIB_REGISTER_PATH_MODULE_BEGIN(MLIB_PROFILE4,MLIB_PATH_VOIP_CALL_SPKOUT)
    /* ģ��ע��˵��: ģ��ID|ģ���ʼ��|ģ���������|ģ�������ȡ|ģ�����ݴ���|ģ����� */
    MLIB_REGISTER_MODULE(
                MLIB_MODULE_DEFAULT,
                NULL,
                NULL,
                NULL,
                MLIB_DEFAULT_Proc,
                NULL)
MLIB_REGISTER_PATH_MODULE_END()

/*****************************************************************************
  ����: MLIB_PATH_AUDIO_PLAY
        MLIB_PATH_AUDIO_RECORD
*****************************************************************************/
MLIB_REGISTER_PATH_MODULE_BEGIN(MLIB_PROFILE4,MLIB_PATH_AUDIO_PLAY)
    /* ģ��ע��˵��: ģ��ID|ģ���ʼ��|ģ���������|ģ�������ȡ|ģ�����ݴ���|ģ����� */
    MLIB_REGISTER_MODULE(
                MLIB_MODULE_DEFAULT,
                NULL,
                NULL,
                NULL,
                MLIB_DEFAULT_Proc,
                NULL)
MLIB_REGISTER_PATH_MODULE_END()

MLIB_REGISTER_PATH_MODULE_BEGIN(MLIB_PROFILE4,MLIB_PATH_AUDIO_RECORD)
    /* ģ��ע��˵��: ģ��ID|ģ���ʼ��|ģ���������|ģ�������ȡ|ģ�����ݴ���|ģ����� */
    MLIB_REGISTER_MODULE(
                MLIB_MODULE_DEFAULT,
                NULL,
                NULL,
                NULL,
                MLIB_DEFAULT_Proc,
                NULL)
MLIB_REGISTER_PATH_MODULE_END()

/*****************************************************************************
  ����: MLIB_PATH_SIRI
*****************************************************************************/
MLIB_REGISTER_PATH_MODULE_BEGIN(MLIB_PROFILE4,MLIB_PATH_SIRI_SPKOUT)
    /* ģ��ע��˵��: ģ��ID|ģ���ʼ��|ģ���������|ģ�������ȡ|ģ�����ݴ���|ģ����� */
    MLIB_REGISTER_MODULE(
                MLIB_MODULE_DEFAULT,
                NULL,
                NULL,
                NULL,
                MLIB_DEFAULT_Proc,
                NULL)
MLIB_REGISTER_PATH_MODULE_END()

MLIB_REGISTER_PATH_MODULE_BEGIN(MLIB_PROFILE4,MLIB_PATH_SIRI_MICIN)
    /* ģ��ע��˵��: ģ��ID|ģ���ʼ��|ģ���������|ģ�������ȡ|ģ�����ݴ���|ģ����� */
    MLIB_REGISTER_MODULE(
                MLIB_MODULE_DEFAULT,
                NULL,
                NULL,
                NULL,
                MLIB_DEFAULT_Proc,
                NULL)
MLIB_REGISTER_PATH_MODULE_END()


/*****************************************************************************
  ����ע��
*****************************************************************************/
MLIB_REGISTER_PATH_BEGIN(MLIB_PROFILE4)
    MLIB_REGISTER_PATH(
                MLIB_PATH_CS_VOICE_CALL_MICIN,
                MLIB_GetPathModules(MLIB_PROFILE4,MLIB_PATH_CS_VOICE_CALL_MICIN),
                MLIB_GetPathModuleNum(MLIB_PROFILE4,MLIB_PATH_CS_VOICE_CALL_MICIN))
    MLIB_REGISTER_PATH(
                MLIB_PATH_CS_VOICE_CALL_SPKOUT,
                MLIB_GetPathModules(MLIB_PROFILE4,MLIB_PATH_CS_VOICE_CALL_SPKOUT),
                MLIB_GetPathModuleNum(MLIB_PROFILE4,MLIB_PATH_CS_VOICE_CALL_SPKOUT))
    MLIB_REGISTER_PATH(
                MLIB_PATH_VOIP_CALL_MICIN,
                MLIB_GetPathModules(MLIB_PROFILE4,MLIB_PATH_VOIP_CALL_MICIN),
                MLIB_GetPathModuleNum(MLIB_PROFILE4,MLIB_PATH_VOIP_CALL_MICIN))
    MLIB_REGISTER_PATH(
                MLIB_PATH_VOIP_CALL_SPKOUT,
                MLIB_GetPathModules(MLIB_PROFILE4,MLIB_PATH_VOIP_CALL_SPKOUT),
                MLIB_GetPathModuleNum(MLIB_PROFILE4,MLIB_PATH_VOIP_CALL_SPKOUT))
    MLIB_REGISTER_PATH(
                MLIB_PATH_AUDIO_PLAY,
                MLIB_GetPathModules(MLIB_PROFILE4,MLIB_PATH_AUDIO_PLAY),
                MLIB_GetPathModuleNum(MLIB_PROFILE4,MLIB_PATH_AUDIO_PLAY))
    MLIB_REGISTER_PATH(
                MLIB_PATH_AUDIO_RECORD,
                MLIB_GetPathModules(MLIB_PROFILE4,MLIB_PATH_AUDIO_RECORD),
                MLIB_GetPathModuleNum(MLIB_PROFILE4,MLIB_PATH_AUDIO_RECORD))
    MLIB_REGISTER_PATH(
                MLIB_PATH_SIRI_SPKOUT,
                MLIB_GetPathModules(MLIB_PROFILE4,MLIB_PATH_SIRI_SPKOUT),
                MLIB_GetPathModuleNum(MLIB_PROFILE4,MLIB_PATH_SIRI_SPKOUT))
    MLIB_REGISTER_PATH(
                MLIB_PATH_SIRI_MICIN,
                MLIB_GetPathModules(MLIB_PROFILE4,MLIB_PATH_SIRI_MICIN),
                MLIB_GetPathModuleNum(MLIB_PROFILE4,MLIB_PATH_SIRI_MICIN))
MLIB_REGISTER_PATH_END()


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* end of mlib_module.h */