/*
* $QNXLicenseC:
* Copyright 2017-2018 NXP
*
* Licensed under the Apache License, Version 2.0 (the "License"). You
* may not reproduce, modify or distribute this software except in
* compliance with the License. You may obtain a copy of the License
* at: http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" basis,
* WITHOUT WARRANTIES OF ANY KIND, either express or implied.
*
* This file may contain contributions from others, either as
* contributors under the License or as licensors under other terms.
* Please review this entire file for other proprietary rights or license
* notices, as well as the QNX Development Suite License Guide at
* http://licensing.qnx.com/license-guide/ for other information.
* $
*/

/*==========================================================================*/
/*!
 * @file svc/irq/api.h
 *
 * Header file containing the public API for the System Controller (SC)
 * Interrupt (IRQ) function.
 *
 * @addtogroup IRQ_SVC (SVC) Interrupt Service
 *
 * Module for the Interrupt (IRQ) service.
 *
 * @{
 */
/*==========================================================================*/

#ifndef SC_IRQ_API_H
#define SC_IRQ_API_H

/* Includes */

#include <sys/sci_types.h>

/* Defines */

#define SC_IRQ_NUM_GROUP    5U          /*!< Number of groups */

/*!
 * @name Defines for sc_irq_group_t
 */
/*@{*/
#define SC_IRQ_GROUP_TEMP   0U   /*!< Temp interrupts */
#define SC_IRQ_GROUP_WDOG   1U   /*!< Watchdog interrupts */
#define SC_IRQ_GROUP_RTC    2U   /*!< RTC interrupts */
#define SC_IRQ_GROUP_WAKE   3U   /*!< Wakeup interrupts */
#define SC_IRQ_GROUP_SYSCTR 4U   /*!< System counter interrupts */
/*@}*/

/*!
 * @name Defines for sc_irq_temp_t
 */
/*@{*/
#define SC_IRQ_TEMP_HIGH         (1UL << 0U)    /*!< Temp alarm interrupt */
#define SC_IRQ_TEMP_CPU0_HIGH    (1UL << 1U)    /*!< CPU0 temp alarm interrupt */
#define SC_IRQ_TEMP_CPU1_HIGH    (1UL << 2U)    /*!< CPU1 temp alarm interrupt */
#define SC_IRQ_TEMP_GPU0_HIGH    (1UL << 3U)    /*!< GPU0 temp alarm interrupt */
#define SC_IRQ_TEMP_GPU1_HIGH    (1UL << 4U)    /*!< GPU1 temp alarm interrupt */
#define SC_IRQ_TEMP_DRC0_HIGH    (1UL << 5U)    /*!< DRC0 temp alarm interrupt */
#define SC_IRQ_TEMP_DRC1_HIGH    (1UL << 6U)    /*!< DRC1 temp alarm interrupt */
#define SC_IRQ_TEMP_VPU_HIGH     (1UL << 7U)    /*!< DRC1 temp alarm interrupt */
#define SC_IRQ_TEMP_PMIC0_HIGH   (1UL << 8U)    /*!< PMIC0 temp alarm interrupt */
#define SC_IRQ_TEMP_PMIC1_HIGH   (1UL << 9U)    /*!< PMIC1 temp alarm interrupt */
#define SC_IRQ_TEMP_LOW          (1UL << 10U)   /*!< Temp alarm interrupt */
#define SC_IRQ_TEMP_CPU0_LOW     (1UL << 11U)   /*!< CPU0 temp alarm interrupt */
#define SC_IRQ_TEMP_CPU1_LOW     (1UL << 12U)   /*!< CPU1 temp alarm interrupt */
#define SC_IRQ_TEMP_GPU0_LOW     (1UL << 13U)   /*!< GPU0 temp alarm interrupt */
#define SC_IRQ_TEMP_GPU1_LOW     (1UL << 14U)   /*!< GPU1 temp alarm interrupt */
#define SC_IRQ_TEMP_DRC0_LOW     (1UL << 15U)   /*!< DRC0 temp alarm interrupt */
#define SC_IRQ_TEMP_DRC1_LOW     (1UL << 16U)   /*!< DRC1 temp alarm interrupt */
#define SC_IRQ_TEMP_VPU_LOW      (1UL << 17U)   /*!< DRC1 temp alarm interrupt */
#define SC_IRQ_TEMP_PMIC0_LOW    (1UL << 18U)   /*!< PMIC0 temp alarm interrupt */
#define SC_IRQ_TEMP_PMIC1_LOW    (1UL << 19U)   /*!< PMIC1 temp alarm interrupt */
#define SC_IRQ_TEMP_PMIC2_HIGH   (1UL << 20U)   /*!< PMIC2 temp alarm interrupt */
#define SC_IRQ_TEMP_PMIC2_LOW    (1UL << 21U)   /*!< PMIC2 temp alarm interrupt */
/*@}*/

/*!
 * @name Defines for sc_irq_wdog_t
 */
/*@{*/
#define SC_IRQ_WDOG              (1U << 0U)    /*!< Watchdog interrupt */
/*@}*/

/*!
 * @name Defines for sc_irq_rtc_t
 */
/*@{*/
#define SC_IRQ_RTC               (1U << 0U)    /*!< RTC interrupt */
/*@}*/

/*!
 * @name Defines for sc_irq_wake_t
 */
/*@{*/
#define SC_IRQ_BUTTON            (1U << 0U)    /*!< Button interrupt */
#define SC_IRQ_PAD               (1U << 1U)    /*!< Pad wakeup */
/*@}*/

/*!
 * @name Defines for sc_irq_sysctr_t
 */
/*@{*/
#define SC_IRQ_SYSCTR            (1U << 0U)    /*!< SYSCTR interrupt */
/*@}*/

/* Types */

/*!
 * This type is used to declare an interrupt group.
 */
typedef uint8_t sc_irq_group_t;

/*!
 * This type is used to declare a bit mask of temp interrupts.
 */
typedef uint8_t sc_irq_temp_t;

/*!
 * This type is used to declare a bit mask of watchdog interrupts.
 */
typedef uint8_t sc_irq_wdog_t;

/*!
 * This type is used to declare a bit mask of RTC interrupts.
 */
typedef uint8_t sc_irq_rtc_t;

/*!
 * This type is used to declare a bit mask of wakeup interrupts.
 */
typedef uint8_t sc_irq_wake_t;

/* Functions */

/*!
 * This function enables/disables interrupts. If pending interrupts
 * are unmasked, an interrupt will be triggered.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    MU channel
 * @param[in]     group       group the interrupts are in
 * @param[in]     mask        mask of interrupts to affect
 * @param[in]     enable      state to change interrupts to
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_PARM if group invalid
 */
sc_err_t sc_irq_enable(sc_ipc_t ipc, sc_rsrc_t resource,
    sc_irq_group_t group, uint32_t mask, sc_bool_t enable);

/*!
 * This function returns the current interrupt status (regardless if
 * masked). Automatically clears pending interrupts.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    MU channel
 * @param[in]     group       groups the interrupts are in
 * @param[in]     status      status of interrupts
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_PARM if group invalid
 *
 * The returned \a status may show interrupts pending that are
 * currently masked.
 */
sc_err_t sc_irq_status(sc_ipc_t ipc, sc_rsrc_t resource,
    sc_irq_group_t group, uint32_t *status);

#endif /* SC_IRQ_API_H */

/**@}*/

