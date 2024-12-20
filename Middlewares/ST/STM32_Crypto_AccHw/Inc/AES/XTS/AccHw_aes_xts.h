/**
  ******************************************************************************
  * @file    AccHw_aes_xts.h
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    30-October-2015
  * @brief   AES in XTS Mode
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#ifndef __CRL_AccHw_AES_XTS_H__
#define __CRL_AccHw_AES_XTS_H__

#ifdef __cplusplus
 extern "C" {
#endif

/** @ingroup AESXTS
  * @{
  */

typedef struct
{  
  uint32_t   mContextId;   /*!< Unique ID of this context. \b Not \b used in current implementation. */  
  AccHw_SKflags_et mFlags;       /*!< 32 bit mFlags, used to perform keyschedule, choose betwen hw/sw and future use */  
  const uint8_t *pmKey;    /*!< Pointer to original XTS Key buffer */  
  const uint8_t *pmTweak;  /*!< Pointer to original Tweak buffer */  
  int32_t   mTweakSize;    /*!< Size of the Tweak in bytes */  
  uint32_t  amTweak[4]; /*!< Temporary result/Tweak */  
  int32_t   mKeySize; /*!< Size of half of the XTS Key in bytes */
  uint32_t  amExpKey[AccHw_CRL_AES_MAX_EXPKEY_SIZE]; /*!< Expanded AES Key 1 */
  uint32_t  amExpKey2[AccHw_CRL_AES_MAX_EXPKEY_SIZE]; /*!< Expanded AES Key 2 */
} AccHw_AESXTSctx_stt; /*!< AES context structure for CBC mode */


/* load the key and ivec, eventually performs key schedule, init hw, etc. */
int32_t AccHw_AES_XTS_Encrypt_Init(AccHw_AESXTSctx_stt *P_pAESXTSctx, const uint8_t *P_pKey, const uint8_t *P_pTweak);

/* launch crypto operation , can be called several times */
int32_t AccHw_AES_XTS_Encrypt_Append (AccHw_AESXTSctx_stt *P_pAESXTSctx,
                                const uint8_t *P_pInputBuffer,
                                int32_t        P_inputSize,
                                uint8_t       *P_pOutputBuffer,
                                int32_t       *P_pOutputSize);

/* Possible final output */
int32_t AccHw_AES_XTS_Encrypt_Finish (AccHw_AESXTSctx_stt *P_pAESXTSctx,
                                uint8_t       *P_pOutputBuffer,
                                int32_t       *P_pOutputSize);

#ifdef INCLUDE_AccHw_DECRYPTION
/* load the key and ivec, eventually performs key schedule, init hw, etc. */
int32_t AccHw_AES_XTS_Decrypt_Init (AccHw_AESXTSctx_stt *P_pAESXTSctx, const uint8_t *P_pKey, const uint8_t *P_pIv);

/* launch crypto operation , can be called several times */
int32_t AccHw_AES_XTS_Decrypt_Append (AccHw_AESXTSctx_stt *P_pAESXTSctx,
                                const uint8_t *P_pInputBuffer,
                                int32_t        P_inputSize,
                                uint8_t       *P_pOutputBuffer,
                                int32_t       *P_pOutputSize);

/* Possible final output */
int32_t AccHw_AES_XTS_Decrypt_Finish (AccHw_AESXTSctx_stt *P_pAESXTSctx,
                                uint8_t       *P_pOutputBuffer,
                                int32_t       *P_pOutputSize);
#endif
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __CRL_AccHw_AES_XTS_H__*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
