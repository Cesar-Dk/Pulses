/**
  ******************************************************************************
  * @file    AccHw_elliptic.h
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    30-October-2015
  * @brief   Provides Elliptic Curve Cryptography (ECC) primitives.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Image SW License Agreement V2, (the "License");
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRL_AccHw_ELLIPTIC_H__
#define __CRL_AccHw_ELLIPTIC_H__

#ifdef __cplusplus
extern "C"
{
#endif


  /** @addtogroup ECClowlevel
    * @{
    */


  /* Exported types ------------------------------------------------------------*/
  /* Note:
  *  Elliptic curve equation over GF(p): y^2 = x^3+ax+b.
  *  All the NIST elliptic curves have a=-3 for performance reasons.
  *  The point number for an elliptic curve is given by n*h where
  *  h is the co-factor; it's usual for standard curves h=1.
  *  n is always a prime number and represents the curve order.
  */


  /* Exported functions ------------------------------------------------------- */
  /* Computes the symmetric of the point P_Point ********************************/
  int32_t AccHw_ECCsymmetricPoint(const AccHw_ECpoint_stt *P_pECpnt,
                            AccHw_ECpoint_stt *P_pECsymmetricPnt,
                            const AccHw_EC_stt *P_pECctx,
                            membuf_stt *P_pMemBuf);

  /* PA=normalized PP. **********************************************************/
  int32_t AccHw_Normalize(const AccHw_ECpoint_stt *P_pECinputPnt,
                    AccHw_ECpoint_stt *P_pECnormPnt,
                    const montyParams_stt* P_pMontyParams);

  /* Converts the x coordinates from integers modulo p to the Montgomery domain */
  int32_t AccHw_ECCpoint2Monty(AccHw_ECpoint_stt *P_pECpnt, const montyParams_stt *P_pMontyParams);

  /* Converts the x coordinates from the Montgomery domain to integers modulo p */
  int32_t AccHw_ECCpointFromMonty(AccHw_ECpoint_stt *P_pECpnt, const montyParams_stt *P_pMontyParams);

  /* Point addition in Jacobian Projective coordinates and in the Montgomery domain. */
  int32_t AccHw_MontyJacProjAddPoints(const AccHw_ECpoint_stt *P_pECpnt1,
                                const AccHw_ECpoint_stt *P_pECpnt2,
                                AccHw_ECpoint_stt *P_pECpntSum,
                                const AccHw_EC_stt *P_pECctx,
                                const montyParams_stt *P_pMontyParams);

  /* Point doubling in Jacobian Projective coordinates and in the Montgomery domain. P2=2*P1*/
  int32_t  AccHw_MontyJacProjDoublePoint(const AccHw_ECpoint_stt *P_pECpnt,
                                   AccHw_ECpoint_stt *P_pECpntDoubled,
                                   const AccHw_EC_stt *P_pECctx,
                                   const montyParams_stt *P_pMontyParams);


  int32_t AccHw_NAF_Binary_PointMul(const BigNum_stt *P_pBNk,
                              const AccHw_ECpoint_stt * P_pECbasePnt,
                              AccHw_ECpoint_stt * P_pECresultPnt,
                              const AccHw_EC_stt *P_pECctx,
                              const montyParams_stt *P_pMontyParams);
  int32_t AccHw_Binary_DoublePointMul(const BigNum_stt *P_pBNk1,
                                     AccHw_ECpoint_stt * P_pECbase1Pnt,
                                     const BigNum_stt *P_pBNk2,
                                     const AccHw_ECpoint_stt * P_pECbase2Pnt,
                                     AccHw_ECpoint_stt * P_pECresultPnt,
                                     const AccHw_EC_stt *P_pECctx,
                                     const montyParams_stt *P_pMontyParams);

#ifdef __cplusplus
}
#endif


#endif /*__CRL_AccHw_ELLIPTIC_H__ */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
