/*
** ###################################################################
**     Processors:          MX8
**
**     Compilers:           GNU C Compiler
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MX8
**
**     Copyright (c) 1997 - 2015 Freescale Semiconductor, Inc.
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
#ifndef HW_OTP_REGISTERS_H
#define HW_OTP_REGISTERS_H

/* ----------------------------------------------------------------------------
   -- OTP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OTP_Peripheral_Access_Layer OTP Peripheral Access Layer
 * @{
 */

/** OTP - Register Layout Typedef */
typedef struct {
    __IO uint32_t RW;
    __IO uint32_t SET;
    __IO uint32_t CLR;
    __IO uint32_t TOG;
} OTP_Reg;

typedef struct {
    OTP_Reg CTRL;
    OTP_Reg PDN;
    OTP_Reg DATA;
    OTP_Reg READ_CTRL;
    OTP_Reg READ_FUSE_DATA;
    OTP_Reg SW_STICKY;
    OTP_Reg SCS;
    OTP_Reg CRC_ADDR;
    OTP_Reg CRC_VALUE;
    OTP_Reg STATUS;
    __I  uint32_t STARTWORD;
    uint8_t RESERVED_0[12];
    __I  uint32_t VERSION;
    uint8_t RESERVED_1[1356];
	struct {
        __I uint32_t RW;
        __I uint32_t SET;
        __I uint32_t CLR;
        __I uint32_t TOG;
    } LOCKED[17];
    uint8_t RESERVED_3[240];
    OTP_Reg FUSE[528];
} OTP_Type;

/* ----------------------------------------------------------------------------
   -- OTP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OTP_Register_Masks OTP Register Masks
 * @{
 */

/*! @name OTP Register */
#define OTP_BIT0_MASK                      (0x00000001U)
#define OTP_BIT0_SHIFT                     (0U)
#define OTP_BIT0(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT1_MASK                      (0x00000002U)
#define OTP_BIT1_SHIFT                     (1U)
#define OTP_BIT1(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT2_MASK                      (0x00000004U)
#define OTP_BIT2_SHIFT                     (2U)
#define OTP_BIT2(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT3_MASK                      (0x00000008U)
#define OTP_BIT3_SHIFT                     (3U)
#define OTP_BIT3(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT4_MASK                      (0x00000010U)
#define OTP_BIT4_SHIFT                     (4U)
#define OTP_BIT4(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT5_MASK                      (0x00000020U)
#define OTP_BIT5_SHIFT                     (5U)
#define OTP_BIT5(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT6_MASK                      (0x00000040U)
#define OTP_BIT6_SHIFT                     (6U)
#define OTP_BIT6(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT7_MASK                      (0x00000080U)
#define OTP_BIT7_SHIFT                     (7U)
#define OTP_BIT7(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT8_MASK                      (0x00000100U)
#define OTP_BIT8_SHIFT                     (8U)
#define OTP_BIT8(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT9_MASK                      (0x00000200U)
#define OTP_BIT9_SHIFT                     (9U)
#define OTP_BIT9(x)                        (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT10_MASK                     (0x00000400U)
#define OTP_BIT10_SHIFT                    (10U)
#define OTP_BIT10(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT11_MASK                     (0x00000800U)
#define OTP_BIT11_SHIFT                    (11U)
#define OTP_BIT11(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT12_MASK                     (0x00001000U)
#define OTP_BIT12_SHIFT                    (12U)
#define OTP_BIT12(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT13_MASK                     (0x00002000U)
#define OTP_BIT13_SHIFT                    (13U)
#define OTP_BIT13(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT14_MASK                     (0x00004000U)
#define OTP_BIT14_SHIFT                    (14U)
#define OTP_BIT14(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT15_MASK                     (0x00008000U)
#define OTP_BIT15_SHIFT                    (15U)
#define OTP_BIT15(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT16_MASK                     (0x00010000U)
#define OTP_BIT16_SHIFT                    (16U)
#define OTP_BIT16(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT17_MASK                     (0x00020000U)
#define OTP_BIT17_SHIFT                    (17U)
#define OTP_BIT17(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT18_MASK                     (0x00040000U)
#define OTP_BIT18_SHIFT                    (18U)
#define OTP_BIT18(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT19_MASK                     (0x00080000U)
#define OTP_BIT19_SHIFT                    (19U)
#define OTP_BIT19(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT20_MASK                     (0x00100000U)
#define OTP_BIT20_SHIFT                    (20U)
#define OTP_BIT20(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT21_ASK                      (0x00200000U)
#define OTP_BIT21_SHIFT                    (21U)
#define OTP_BIT21(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT22_MASK                     (0x00400000U)
#define OTP_BIT22_SHIFT                    (22U)
#define OTP_BIT22(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT23_MASK                     (0x00800000U)
#define OTP_BIT23_SHIFT                    (23U)
#define OTP_BIT23(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT24_MASK                     (0x01000000U)
#define OTP_BIT24_SHIFT                    (24U)
#define OTP_BIT24(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT25_MASK                     (0x02000000U)
#define OTP_BIT25_SHIFT                    (25U)
#define OTP_BIT25(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT26_MASK                     (0x04000000U)
#define OTP_BIT26_SHIFT                    (26U)
#define OTP_BIT26(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT27_MASK                     (0x08000000U)
#define OTP_BIT27_SHIFT                    (27U)
#define OTP_BIT27(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT28_MASK                     (0x10000000U)
#define OTP_BIT28_SHIFT                    (28U)
#define OTP_BIT28(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT29_MASK                     (0x20000000U)
#define OTP_BIT29_SHIFT                    (29U)
#define OTP_BIT29(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT30_MASK                     (0x40000000U)
#define OTP_BIT30_SHIFT                    (30U)
#define OTP_BIT30(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_BIT31_MASK                     (0x80000000U)
#define OTP_BIT31_SHIFT                    (31U)
#define OTP_BIT31(x)                       (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_NIBBLE0_MASK                   (0x0000000FU)
#define OTP_NIBBLE0_SHIFT                  (0U)
#define OTP_NIBBLE0(x)                     (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_NIBBLE1_MASK                   (0x000000F0U)
#define OTP_NIBBLE1_SHIFT                  (4U)
#define OTP_NIBBLE1(x)                     (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_NIBBLE2_MASK                   (0x00000F00U)
#define OTP_NIBBLE2_SHIFT                  (8U)
#define OTP_NIBBLE2(x)                     (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_NIBBLE3_MASK                   (0x0000F000U)
#define OTP_NIBBLE3_SHIFT                  (12U)
#define OTP_NIBBLE3(x)                     (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_NIBBLE4_MASK                   (0x000F0000U)
#define OTP_NIBBLE4_SHIFT                  (16U)
#define OTP_NIBBLE4(x)                     (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_NIBBLE5_MASK                   (0x00F00000U)
#define OTP_NIBBLE5_SHIFT                  (20U)
#define OTP_NIBBLE5(x)                     (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_NIBBLE6_MASK                   (0x0F000000U)
#define OTP_NIBBLE6_SHIFT                  (24U)
#define OTP_NIBBLE6(x)                     (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*! @name OTP Register */
#define OTP_NIBBLE7_MASK                   (0xF0000000U)
#define OTP_NIBBLE7_SHIFT                  (28U)
#define OTP_NIBBLE7(x)                     (((uint32_t)(((uint32_t)(x)) << OTP_PDOR_PDO_SHIFT)) & OTP_PDOR_PDO_MASK)

/*!
 * @}
 */ /* end of group OTP_Register_Masks */

/* OTP - Peripheral instance base addresses */
/** Peripheral OTPA base pointer */
#define OTP                                     ((OTP_Type *)OTP_BASE)
/** Array initializer of OTP peripheral base addresses */
#define OTP_BASE_ADDRS                          { OTP_BASE }
/** Array initializer of OTP peripheral base pointers */
#define OTP_BASE_PTRS                           { OTP }

/*!
 * @}
 */ /* end of group OTP_Peripheral_Access_Layer */

#endif /* HW_OTP_REGISTERS_H */
