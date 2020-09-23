/*
** ###################################################################
**     Processors:          MX8
**
**     Compilers:           GNU C Compiler
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MX8
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2017 NXP
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

/*
 * WARNING! DO NOT EDIT THIS FILE DIRECTLY!
 *
 * This file was generated automatically and any changes may be lost.
 */
#ifndef HW_DMA_REGISTERS_H
#define HW_DMA_REGISTERS_H

/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

/** DMA - Register Layout Typedef */
typedef struct {
  __IO  uint32_t MP_CSR;             /**< offset: 0x0 */
  __I   uint32_t MP_ES;              /**< offset: 0x4 */
  uint8_t RESERVED_0[4];
  __I   uint32_t MP_HRS;             /**< offset: 0xC */
  uint8_t RESERVED_1[240];
  __IO  uint32_t CH_GRPRI[32];       /**< offset: 0x100 */
  uint8_t RESERVED_2[65152];
  struct {                           /**< offset: 0x10000 */
  __IO    uint32_t CH_CSR;             /**< offset: 0x0 */
  __IO    uint32_t CH_ES;              /**< offset: 0x4 */
  __IO    uint32_t CH_INT;             /**< offset: 0x8 */
  __IO    uint32_t CH_SBR;             /**< offset: 0xC */
  __IO    uint32_t CH_PRI;             /**< offset: 0x10 */
  uint8_t RESERVED_3[12];
  __IO    uint32_t TCD_SADDR;          /**< offset: 0x20 */
  __IO    uint16_t TCD_SOFF;           /**< offset: 0x24 */
  __IO    uint16_t TCD_ATTR;           /**< offset: 0x26 */
  __IO    uint32_t TCD_NBYTES_MLOFFNO; /**< offset: 0x28 */
  __IO    uint32_t TCD_SLAST_SDA;      /**< offset: 0x2C */
  __IO    uint32_t TCD_DADDR;          /**< offset: 0x30 */
  __IO    uint16_t TCD_DOFF;           /**< offset: 0x34 */
  __IO    uint16_t TCD_CITER_ELINKNO;  /**< offset: 0x36 */
  __IO    uint32_t TCD_DLAST_SGA;      /**< offset: 0x38 */
  __IO    uint16_t TCD_CSR;            /**< offset: 0x3C */
  __IO    uint16_t TCD_BITER_ELINKNO;  /**< offset: 0x3E */
  uint8_t RESERVED_4[65472];
  } CH[32];
} DMA_Type;

/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/*! @name MP_CSR - Management Page Control Register */
#define DMA_MP_CSR_EBW_MASK                    (0x00000001U)
#define DMA_MP_CSR_EBW_SHIFT                   (0U)
#define DMA_MP_CSR_EBW(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_EBW_SHIFT)) & DMA_MP_CSR_EBW_MASK)
#define DMA_MP_CSR_EDBG_MASK                   (0x00000002U)
#define DMA_MP_CSR_EDBG_SHIFT                  (1U)
#define DMA_MP_CSR_EDBG(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_EDBG_SHIFT)) & DMA_MP_CSR_EDBG_MASK)
#define DMA_MP_CSR_ERCA_MASK                   (0x00000004U)
#define DMA_MP_CSR_ERCA_SHIFT                  (2U)
#define DMA_MP_CSR_ERCA(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_ERCA_SHIFT)) & DMA_MP_CSR_ERCA_MASK)
#define DMA_MP_CSR_HAE_MASK                    (0x00000010U)
#define DMA_MP_CSR_HAE_SHIFT                   (4U)
#define DMA_MP_CSR_HAE(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_HAE_SHIFT)) & DMA_MP_CSR_HAE_MASK)
#define DMA_MP_CSR_HALT_MASK                   (0x00000020U)
#define DMA_MP_CSR_HALT_SHIFT                  (5U)
#define DMA_MP_CSR_HALT(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_HALT_SHIFT)) & DMA_MP_CSR_HALT_MASK)
#define DMA_MP_CSR_ECL_MASK                    (0x00000040U)
#define DMA_MP_CSR_ECL_SHIFT                   (6U)
#define DMA_MP_CSR_ECL(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_ECL_SHIFT)) & DMA_MP_CSR_ECL_MASK)
#define DMA_MP_CSR_EMI_MASK                    (0x00000080U)
#define DMA_MP_CSR_EMI_SHIFT                   (7U)
#define DMA_MP_CSR_EMI(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_EMI_SHIFT)) & DMA_MP_CSR_EMI_MASK)
#define DMA_MP_CSR_ECX_MASK                    (0x00000100U)
#define DMA_MP_CSR_ECX_SHIFT                   (8U)
#define DMA_MP_CSR_ECX(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_ECX_SHIFT)) & DMA_MP_CSR_ECX_MASK)
#define DMA_MP_CSR_CX_MASK                     (0x00000200U)
#define DMA_MP_CSR_CX_SHIFT                    (9U)
#define DMA_MP_CSR_CX(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_CX_SHIFT)) & DMA_MP_CSR_CX_MASK)
#define DMA_MP_CSR_ACTIVE_ID_MASK              (0x1F000000U)
#define DMA_MP_CSR_ACTIVE_ID_SHIFT             (24U)
#define DMA_MP_CSR_ACTIVE_ID(x)                (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_ACTIVE_ID_SHIFT)) & DMA_MP_CSR_ACTIVE_ID_MASK)
#define DMA_MP_CSR_ACTIVE_MASK                 (0x80000000U)
#define DMA_MP_CSR_ACTIVE_SHIFT                (31U)
#define DMA_MP_CSR_ACTIVE(x)                   (((uint32_t)(((uint32_t)(x)) << DMA_MP_CSR_ACTIVE_SHIFT)) & DMA_MP_CSR_ACTIVE_MASK)

/*! @name GRPRI - Channel Arbitration Group */
#define DMA_CH_GRPRI_GRPRI_MASK                (0x0000001FU)
#define DMA_CH_GRPRI_GRPRI_SHIFT               (0U)
#define DMA_CH_GRPRI_GRPRI(x)                  (((uint32_t)(((uint32_t)(x)) << DMA_GRPRI_GRPRI_SHIFT)) & DMA_GRPRI_GRPRI_MASK)

/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA0 base pointer */
#define DMA0                                   ((DMA_Type *)DMA0_BASE)
/** Peripheral DMA1 base pointer */
#define DMA1                                   ((DMA_Type *)DMA1_BASE)
/** Peripheral DMA2 base pointer */
#define DMA2                                   ((DMA_Type *)DMA2_BASE)
/** Peripheral DMA3 base pointer */
#define DMA3                                   ((DMA_Type *)DMA3_BASE)
/** Peripheral DMA4 base pointer */
#define DMA4                                   ((DMA_Type *)DMA4_BASE)
/** Peripheral DMA5 base pointer */
#define DMA5                                   ((DMA_Type *)DMA5_BASE)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                         { DMA0_BASE, \
                                                 DMA1_BASE, \
                                                 DMA2_BASE, \
                                                 DMA3_BASE, \
                                                 DMA4_BASE, \
                                                 DMA5_BASE}
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                          { DMA0, \
                                                 DMA1, \
                                                 DMA2, \
                                                 DMA3, \
                                                 DMA4, \
                                                 DMA5}

/*!
 * @}
 */ /* end of group DMA_Peripheral_Access_Layer */

#endif /* HW_DMA_REGISTERS_H */
