/*
** ###################################################################
**
**     Copyright 2018 NXP
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/*!
 * @file
 *
 * Header file used to configure board specific features of the SCFW.
 *
 */
/*==========================================================================*/

#ifndef SC_BOARD_H
#define SC_BOARD_H

/* Includes */
#include "drivers/pmic/fsl_pmic.h"
/* Defines */

//#define SKIP_DDR

/*! Configure PMIC I2C */
#define LPI2C_PMIC              LPI2C_SC

/*! Configure PMIC I2C instance */
#define LPI2C_PMIC_INST         0U

#define PMIC_0_ADDR             0x8U
#define PMIC_1_ADDR             0x9U
#define PMIC_2_ADDR             0xAU

#define PMIC_TEMP_MAX           135U

#define SW1ABC_STEP             25U
#define SW2_STEP                25U
#define SW3_STEP                25U
#define PMIC0_SW4_STEP          25U
#define PMIC1_SW4_STEP          50U
#define VGEN12_STEP             50U
#define VGEN3456_STEP           100U

#define PF100_TRIPLE            0U
#define PF8100_DUAL             1U
#define PF8100_SINGLE           2U

/* 
 * Resume from KS1 ramps VDD_MAIN 200 mV (800 mV to 1000 mV)
 * PF8100 reg freq = 2.5 MHz, SWxDVS_RAMP = 0 => 7.813 mV/us
 * 200 mV / 7.813 mV/us = 25.6 us ==> 26 us
 * 
 */
#define BOARD_KS1_RESUME_USEC   26U
#define BOARD_KS1_RETENTION     BOARD_PARM_KS1_RETENTION_ENABLE
#define BOARD_KS1_ONOFF_WAKE    BOARD_PARM_KS1_ONOFF_WAKE_ENABLE

#endif /* SC_BOARD_H */

