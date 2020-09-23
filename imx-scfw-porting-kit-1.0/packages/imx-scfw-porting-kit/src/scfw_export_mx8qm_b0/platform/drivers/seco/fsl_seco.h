/*
 * Copyright 2017-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DRV_SECO_H
#define DRV_SECO_H

/*!
 * @addtogroup seco_driver
 * @{
 */

/*! @file */

/* Includes */

#include "main/types.h"
#include "svc/misc/api.h"

/* Defines */

/*!
 * @name Defines for seco_lifecycle_t
 */
/*@{*/
#define SECO_LIFECYCLE_DEFAULT      BIT(0)    /* Default fab mode (early_fuses_pgrm not blown) */
#define SECO_LIFECYCLE_FAB          BIT(1)    /* Fab mode */
#define SECO_LIFECYCLE_NO_SECRETS   BIT(2)    /* No secrets */
#define SECO_LIFECYCLE_SECRETS      BIT(3)    /* With Secrets */
#define SECO_LIFECYCLE_SC_FW_CLSD   BIT(4)    /* SCU FW Closed */
#define SECO_LIFECYCLE_SECO_FW_CLSD BIT(5)    /* SECO FW Closed */
#define SECO_LIFECYCLE_CLOSED       BIT(6)    /* Closed */
#define SECO_LIFECYCLE_CLOSED_FW    BIT(7)    /* Closed with FW */
#define SECO_LIFECYCLE_PART_RTN     BIT(8,    /* Partial field return */
#define SECO_LIFECYCLE_RTN          BIT(9)    /* Field return */
#define SECO_LIFECYCLE_NO_RTN       BIT(10)   /* No Return */
/*@}*/

/*!
 * @name Defines for seco_snvs_id_t
 */
/*@{*/
#define AHAB_SNVS_ID_INIT           0x00U
#define AHAB_SNVS_ID_POWER_OFF      0x01U
#define AHAB_SNVS_ID_SRTC           0x02U
#define AHAB_SNVS_ID_SRTC_CALB      0x03U
#define AHAB_SNVS_ID_SRTC_ALRM      0x04U
#define AHAB_SNVS_ID_RTC            0x05U
#define AHAB_SNVS_ID_RTC_CALB       0x06U
#define AHAB_SNVS_ID_RTC_ALRM       0x07U
#define AHAB_SNVS_ID_BTN_CONFIG     0x08U
#define AHAB_SNVS_ID_BTN_MASK       0x09U
#define AHAB_SNVS_ID_BTN            0x0AU
#define AHAB_SNVS_ID_BTN_CLEAR      0x0BU
#define AHAB_SNVS_ID_SSM_STATE      0x0CU
#define AHAB_SNVS_ID_BTN_TIME       0x0DU
/*@}*/

/*!
 * @name Defines for seco_rng_stat_t
 */
/*@{*/
#define SECO_RNG_STAT_UNAVAILABLE   0U
#define SECO_RNG_STAT_INPROGRESS    1U
#define SECO_RNG_STAT_READY         2U
/*@}*/

/* Types */

typedef uint16_t seco_lifecycle_t;
typedef uint8_t seco_snvs_id_t;
typedef uint32_t seco_rng_stat_t;

/* Functions */

void SECO_Init(uint8_t phase);
seco_lifecycle_t SECO_Get_Lifecycle(void);
sc_err_t SECO_Ping(void);
sc_err_t SECO_Power(sc_sub_t ss, uint32_t inst, sc_bool_t up);
sc_err_t SECO_Image_Load(sc_faddr_t addr_src, sc_faddr_t addr_dst, uint32_t len,
    sc_bool_t fw);
sc_err_t SECO_Authenticate(sc_misc_seco_auth_cmd_t cmd, sc_faddr_t addr);
void SECO_ClearCache(void);
sc_err_t SECO_CAAM_Config(uint16_t master, sc_bool_t lock, sc_rm_spa_t sa,
    sc_rm_did_t did, sc_rm_sid_t sid);
sc_err_t SECO_CAAM_PowerDown(void);
sc_err_t SECO_EnterLPM(void);
sc_err_t SECO_ExitLPM(void);
sc_err_t SECO_WriteSNVS(seco_snvs_id_t id, uint32_t val);
sc_err_t SECO_ReadSNVS(seco_snvs_id_t id, uint32_t *val);
sc_err_t SECO_WriteFuse(uint32_t word, uint32_t val);
sc_err_t SECO_SecureWriteFuse(sc_faddr_t addr);
sc_err_t SECO_EnableDebug(sc_faddr_t addr);
sc_err_t SECO_ForwardLifecycle(uint32_t change);
sc_err_t SECO_ReturnLifecycle(sc_faddr_t addr);
sc_err_t SECO_DumpDebug(void);
sc_err_t SECO_KickWdog(void);
sc_err_t SECO_Version(uint32_t *version, uint32_t *commit, sc_bool_t *dirty);
sc_err_t SECO_AttachDebug(void);
sc_err_t SECO_ChipInfo(uint16_t *lc, uint16_t *monotonic, 
    uint32_t *uid_l, uint32_t *uid_h);
sc_err_t SECO_DumpEvents(void);
uint32_t SECO_ErrNumber(void);
sc_err_t SECO_StartRNG(seco_rng_stat_t *status);
sc_err_t SECO_AttestMode(uint32_t mode);
sc_err_t SECO_Attest(uint64_t nonce);
sc_err_t SECO_GetAttestPublicKey(sc_faddr_t addr);
sc_err_t SECO_GetAttestSign(sc_faddr_t addr);
sc_err_t SECO_AttestVerify(sc_faddr_t addr);
sc_err_t SECO_Commit(uint32_t *info);

#endif /* DRV_SECO_H */

/**@}*/

