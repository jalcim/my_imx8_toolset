/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#ifndef DRV_SNVS_H
#define DRV_SNVS_H

/*!
 * @addtogroup snvs_driver
 * @{
 */

/*! @file */

/* Includes */

#include "main/types.h"

/* Defines */

/*!
 * @name Defines for snvs_btn_config_t
 */
/*@{*/
#define SNVS_DRV_BTN_CONFIG_ACTIVELOW       0U
#define SNVS_DRV_BTN_CONFIG_ACTIVEHIGH      1U
#define SNVS_DRV_BTN_CONFIG_RISINGEDGE      2U
#define SNVS_DRV_BTN_CONFIG_FALLINGEDGE     3U
#define SNVS_DRV_BTN_CONFIG_ANYEDGE         4U
/*@}*/

/*!
 * @name Defines for snvs_btn_on_time_t
 */
/*@{*/
#define SNVS_DRV_BTN_ON_50MS                0U
#define SNVS_DRV_BTN_ON_100MS               1U
#define SNVS_DRV_BTN_ON_500MS               2U
#define SNVS_DRV_BTN_ON_0MS                 3U
/*@}*/

/*!
 * @name Defines for snvs_btn_debounce_t
 */
/*@{*/
#define SNVS_DRV_BTN_DEBOUNCE_50MS          0U
#define SNVS_DRV_BTN_DEBOUNCE_100MS         1U
#define SNVS_DRV_BTN_DEBOUNCE_500MS         2U
#define SNVS_DRV_BTN_DEBOUNCE_0MS           3U
/*@}*/

/*!
 * @name Defines for snvs_btn_press_time_t
 */
/*@{*/
#define SNVS_DRV_BTN_PRESS_5S               0U
#define SNVS_DRV_BTN_PRESS_10S              1U
#define SNVS_DRV_BTN_PRESS_15S              2U
#define SNVS_DRV_BTN_PRESS_OFF              3U
/*@}*/

/* Types */

/*!
 * This type is used configure the button active state.
 */
typedef uint8_t snvs_btn_config_t;

/*!
 * This type is used configure the button on time.
 */
typedef uint8_t snvs_btn_on_time_t;

/*!
 * This type is used configure the button debounce time.
 */
typedef uint8_t snvs_btn_debounce_t;

/*!
 * This type is used configure the button press time.
 */
typedef uint8_t snvs_btn_press_time_t;

/* SNVS Functions */

/*!
 * @name Initialization Functions
 * @{
 */

void SNVS_Init(uint8_t phase);

/* @} */

/*!
 * @name SRTC Functions
 * @{
 */

sc_err_t SNVS_PowerOff(void);
sc_err_t SNVS_SetSecureRtc(uint32_t seconds);
sc_err_t SNVS_GetSecureRtc(uint32_t *seconds);
sc_err_t SNVS_SetSecureRtcCalb(int8_t count);
sc_err_t SNVS_GetSecureRtcCalb(int8_t *count);
sc_err_t SNVS_SetSecureRtcAlarm(uint32_t seconds);
sc_err_t SNVS_GetSecureRtcAlarm(uint32_t *seconds);
sc_err_t SNVS_SetRtc(uint32_t seconds);
sc_err_t SNVS_GetRtc(uint32_t *seconds);
sc_err_t SNVS_SetRtcCalb(int8_t count);
sc_err_t SNVS_SetRtcAlarm(uint32_t seconds);
sc_err_t SNVS_GetRtcAlarm(uint32_t *seconds);
sc_err_t SNVS_ConfigButton(snvs_btn_config_t config, sc_bool_t enable);
sc_err_t SNVS_ButtonTime(snvs_btn_on_time_t on, snvs_btn_debounce_t debounce,
    snvs_btn_press_time_t press);
sc_bool_t SNVS_GetButtonStatus(void);
void SNVS_ClearButtonIRQ(void);
void SNVS_EnterLPM(void);
void SNVS_ExitLPM(void);
uint32_t SNVS_GetState(void);
void SNVS_SecurityViolation_IRQHandler(void);

/* @} */

#endif /* DRV_SNVS_H */

/**@}*/

