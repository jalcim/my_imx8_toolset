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
#ifndef HW_SMMU_REGISTERS_H
#define HW_SMMU_REGISTERS_H

#include "stdint.h"

#define SMMU_NUM_SMRG       32U
#define SMMU_NUM_CB         32U
#define SMMU_PAGESHIFT      12U

#define SMMU_PAGESIZE       (1U << SMMU_PAGESHIFT)
#define SMMU_PAGEMASK       (SMMU_PAGESIZE - 1U)

#define SMMU_GLOBAL_SIZE    (SMMU_NUM_CB * PAGESIZE)
#define SMMU_CB_SIZE        SMMU_GLOBAL_SIZE

#define SMMU_CB_BASE(x)     ((uintptr_t)(x) + SMMU_GLOBAL_SIZE) 
#define SMMU_CBn_BASE(x, n) (SMMU_CB_BASE(x) + ((n) * PAGESIZE))

/*******************************************************************************
 * HW_SMMU_SCR0 - Configuration Register 0
 ******************************************************************************/

/*!
 * @brief HW_SMMU_SCR0 - Configuration Register 0 (RW)
 *
 */
typedef union _hw_smmu_scr0
{
    uint32_t U;
    struct _hw_smmu_scr0_bitfields
    {
        uint32_t CLIENTPD : 1;
        uint32_t GFRE : 1;
        uint32_t GFIE : 1;
        uint32_t EXIDENABLE : 1;
        uint32_t GCFGFRE : 1;
        uint32_t GCFGFIE : 1;
        uint32_t TRANSIENTCFG : 2;
        uint32_t STALLD : 1;
        uint32_t GSE : 1;
        uint32_t USFCFG : 1;
        uint32_t VMIDPNE : 1;
        uint32_t PTM : 1;
        uint32_t FB : 1;
        uint32_t BSU : 2;
        uint32_t MEMATTR : 4;
        uint32_t MTCFG : 1;
        uint32_t SMCFCFG : 1;
        uint32_t SHCFG : 2;
        uint32_t RACFG : 2;
        uint32_t WACFG : 2;
        uint32_t NSCFG : 2;
        uint32_t RESERVED0 : 2;
    } B;
} hw_smmu_scr0_t;

/*!
 * @name Constants and macros for entire SMMU_SCR0 register
 */
/*@{*/
#define HW_SMMU_SCR0_ADDR(x)        ((uintptr_t)(x) + 0x0U)
#define HW_SMMU_SCR0(x)             (*(__IO hw_smmu_scr0_t *) HW_SMMU_SCR0_ADDR(x))

#define HW_SMMU_NSCR0_ADDR(x)       ((uintptr_t)(x) + 0x400U)
#define HW_SMMU_NSCR0(x)            (*(__IO hw_smmu_scr0_t *) HW_SMMU_NSCR0_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_SCR1 - Configuration Register 1
 ******************************************************************************/

/*!
 * @brief HW_SMMU_SCR1 - Configuration Register 1 (RW)
 *
 */
typedef union _hw_smmu_scr1
{
    uint32_t U;
    struct _hw_smmu_scr1_bitfields
    {
        uint32_t NSNUMCBO : 8;
        uint32_t NSNUMSMRGO : 8;
        uint32_t NSNUMIRPTO : 8;
        uint32_t GASRAE : 1;
        uint32_t GEFRO : 1;
        uint32_t SIF : 1;
        uint32_t SPMEN : 1;
        uint32_t NSCAFRO : 1;
        uint32_t RESERVED0 : 3;
    } B;
} hw_smmu_scr1_t;

/*!
 * @name Constants and macros for entire SMMU_SCR1 register
 */
/*@{*/
#define HW_SMMU_SCR1_ADDR(x)        ((uintptr_t)(x) + 0x4U)
#define HW_SMMU_SCR1(x)             (*(__IO hw_smmu_scr1_t *) HW_SMMU_SCR1_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_SCR2 - Configuration Register 2
 ******************************************************************************/

/*!
 * @brief HW_SMMU_SCR2 - Configuration Register 2 (RW)
 *
 */
typedef union _hw_smmu_scr2
{
    uint32_t U;
    struct _hw_smmu_scr2_bitfields
    {
        uint32_t BPVMID : 8;
        uint32_t RESERVED0 : 24;
    } B;
} hw_smmu_scr2_t;

/*!
 * @name Constants and macros for entire SMMU_SCR2 register
 */
/*@{*/
#define HW_SMMU_SCR2_ADDR(x)        ((uintptr_t)(x) + 0x8U)
#define HW_SMMU_SCR2(x)             (*(__IO hw_smmu_scr2_t *) HW_SMMU_SCR2_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_SACR - Auxiliary Configuration Register
 ******************************************************************************/

/*!
 * @brief HW_SMMU_SACR - Auxiliary Configuration Register (RW)
 *
 */
typedef union _hw_smmu_sacr
{
    uint32_t U;
    struct _hw_smmu_sacr_bitfields
    {
        uint32_t RESERVED0 : 2;
        uint32_t S1WC2EN : 1;
        uint32_t S2WC2EN : 1;
        uint32_t IPA2PA_CEN : 1;
        uint32_t RESERVED1 : 3;
        uint32_t SMTNMB_TLBEN : 1;
        uint32_t MMUDISB_TLBEN : 1;
        uint32_t S2CRB_TLBEN : 1;
        uint32_t RESERVED2 : 5;
        uint32_t PAGESIZE : 1;
        uint32_t RESERVED3 : 7;
        uint32_t DP4K_TCUDISB : 1;
        uint32_t DP4K_TBUDISB : 1;
        uint32_t CACHE_LOCK : 1;
        uint32_t RESERVED4 : 5;
    } B;
} hw_smmu_sacr_t;

/*!
 * @name Constants and macros for entire SMMU_ACR register
 */
/*@{*/
#define HW_SMMU_SACR_ADDR(x)        ((uintptr_t)(x) + 0x10U)
#define HW_SMMU_SACR(x)             (*(__IO hw_smmu_sacr_t *) HW_SMMU_SACR_ADDR(x))

#define HW_SMMU_NSACR_ADDR(x)       ((uintptr_t)(x) + 0x410U)
#define HW_SMMU_NSACR(x)            (*(__IO hw_smmu_sacr_t *) HW_SMMU_NSACR_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_IDR0 - Identification Register 0
 ******************************************************************************/

/*!
 * @brief HW_SMMU_IDR0 - Identification Register 0 (RO)
 *
 */
typedef union _hw_smmu_idr0
{
    uint32_t U;
    struct _hw_smmu_idr0_bitfields
    {
        uint32_t NUMSMRG : 8;
        uint32_t EXIDS : 1;
        uint32_t NUMSIDB : 4;
        uint32_t BTM : 1;
        uint32_t CTTW : 1;
        uint32_t RESERVED0 : 1;
        uint32_t NUMIRPT : 8;
        uint32_t PTFS : 2;
        uint32_t ATOSNS : 1;
        uint32_t SMS : 1;
        uint32_t NTS : 1;
        uint32_t S2TS : 1;
        uint32_t S1TS : 1;
        uint32_t SES : 1;
    } B;
} hw_smmu_idr0_t;

/*!
 * @name Constants and macros for entire SMMU_IDR0 register
 */
/*@{*/
#define HW_SMMU_IDR0_ADDR(x)        ((uintptr_t)(x) + 0x20U)
#define HW_SMMU_IDR0(x)             (*(__I hw_smmu_idr0_t *) HW_SMMU_IDR0_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_IDR1 - Identification Register 1
 ******************************************************************************/

/*!
 * @brief HW_SMMU_IDR1 - Identification Register 1 (RO)
 *
 */
typedef union _hw_smmu_idr1
{
    uint32_t U;
    struct _hw_smmu_idr1_bitfields
    {
        uint32_t NUMCB : 8;
        uint32_t NUMSSDNDXB : 4;
        uint32_t SSDTP : 2;
        uint32_t RESERVED0 : 1;
        uint32_t SMCD : 1;
        uint32_t NUMS2CB : 8;
        uint32_t RESERVED1 : 4;
        uint32_t NUMPAGENDXB : 3;
        uint32_t PAGESIZE : 1;
    } B;
} hw_smmu_idr1_t;

/*!
 * @name Constants and macros for entire SMMU_IDR1 register
 */
/*@{*/
#define HW_SMMU_IDR1_ADDR(x)        ((uintptr_t)(x) + 0x24U)
#define HW_SMMU_IDR1(x)             (*(__I hw_smmu_idr1_t *) HW_SMMU_IDR1_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_IDR2 - Identification Register 2
 ******************************************************************************/

/*!
 * @brief HW_SMMU_IDR2 - Identification Register 2 (RO)
 *
 */
typedef union _hw_smmu_idr2
{
    uint32_t U;
    struct _hw_smmu_idr2_bitfields
    {
        uint32_t IAS : 4;
        uint32_t OAS : 4;
        uint32_t UBS : 4;
        uint32_t PTFSV8 : 3;
        uint32_t RESERVED0 : 17;
    } B;
} hw_smmu_idr2_t;

/*!
 * @name Constants and macros for entire SMMU_IDR2 register
 */
/*@{*/
#define HW_SMMU_IDR2_ADDR(x)        ((uintptr_t)(x) + 0x28U)
#define HW_SMMU_IDR2(x)             (*(__I hw_smmu_idr2_t *) HW_SMMU_IDR2_ADDR(x))
/*@}*/


/*******************************************************************************
 * HW_SMMU_IDR3-6 - Identification Register 3-6
 ******************************************************************************/

/*!
 * @name Constants and macros for SMMU_IDR3-6 registers
 */
/*@{*/
#define HW_SMMU_IDR3_ADDR(x)        ((uintptr_t)(x) + 0x2CU)
#define HW_SMMU_IDR4_ADDR(x)        ((uintptr_t)(x) + 0x30U)
#define HW_SMMU_IDR5_ADDR(x)        ((uintptr_t)(x) + 0x34U)
#define HW_SMMU_IDR6_ADDR(x)        ((uintptr_t)(x) + 0x38U)
/*@}*/

/*******************************************************************************
 * HW_SMMU_IDR7 - Identification Register 7
 ******************************************************************************/

/*!
 * @brief HW_SMMU_IDR7 - Identification Register 7 (RO)
 *
 */
typedef union _hw_smmu_idr7
{
    uint32_t U;
    struct _hw_smmu_idr7_bitfields
    {
        uint32_t MINOR : 4;
        uint32_t MAJOR : 4;
        uint32_t RESERVED0 : 24;
    } B;
} hw_smmu_idr7_t;

/*!
 * @name Constants and macros for entire SMMU_IDR7 register
 */
/*@{*/
#define HW_SMMU_IDR7_ADDR(x)        ((uintptr_t)(x) + 0x3CU)
#define HW_SMMU_IDR7(x)             (*(__I hw_smmu_idr7_t *) HW_SMMU_IDR7_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_SGFSR - Global Fault Status Register
 ******************************************************************************/

/*!
 * @brief SGFSR - Global Fault Status Register (RW)
 *
 */
typedef union _hw_smmu_sgfsr
{
    uint32_t U;
    struct _hw_smmu_sgfsr_bitfields
    {
        uint32_t ICF : 1;
        uint32_t USF : 1;
        uint32_t SMCF : 1;
        uint32_t UCBF : 1;
        uint32_t UCIF : 1;
        uint32_t CAF : 1;
        uint32_t EF : 1;
        uint32_t PF : 1;
        uint32_t UUT : 1;
        uint32_t RESERVED0 : 22;
        uint32_t MULTI : 1;
    } B;
} hw_smmu_sgfsr_t;

/*!
 * @name Constants and macros for entire SMMU_SGFSR register
 */
/*@{*/
#define HW_SMMU_SGFSR_ADDR(x)        ((uintptr_t)(x) + 0x48U)
#define HW_SMMU_SGFSR(x)             (*(__IO hw_smmu_sgfsr_t *) HW_SMMU_SGFSR_ADDR(x))

#define HW_SMMU_NSGFSR_ADDR(x)       ((uintptr_t)(x) + 0x448U)
#define HW_SMMU_NSGFSR(x)            (*(__IO hw_smmu_sgfsr_t *) HW_SMMU_NSGFSR_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_SGFSYNR0 - Global Fault Syndrome Register 0
 ******************************************************************************/

/*!
 * @brief SGFSYNR0 - Global Fault Syndrome Register 0 (RW)
 *
 */
typedef union _hw_smmu_sgfsynr0
{
    uint32_t U;
    struct _hw_smmu_sgfsynr0_bitfields
    {
        uint32_t NESTED : 1;
        uint32_t WNR : 1;
        uint32_t PNU : 1;
        uint32_t IND : 1;
        uint32_t NSSTATE : 1;
        uint32_t NSATTR : 1;
        uint32_t ATS : 1;
        uint32_t RESERVED0 : 25;
    } B;
} hw_smmu_sgfsynr0_t;

/*!
 * @name Constants and macros for entire SMMU_SGFSYNR0 register
 */
/*@{*/
#define HW_SMMU_SGFSYNR0_ADDR(x)        ((uintptr_t)(x) + 0x50U)
#define HW_SMMU_SGFSYNR0(x)             (*(__IO hw_smmu_sgfsynr0_t *) HW_SMMU_SGFSYNR0_ADDR(x))

#define HW_SMMU_NSGFSYNR0_ADDR(x)       ((uintptr_t)(x) + 0x450U)
#define HW_SMMU_NSGFSYNR0(x)            (*(__IO hw_smmu_sgfsynr0_t *) HW_SMMU_NSGFSYNR0_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_SGFSYNR1 - Global Fault Syndrome Register 1
 ******************************************************************************/

/*!
 * @brief SGFSYNR1 - Global Fault Syndrome Register 1 (RW)
 *
 */
typedef union _hw_smmu_sgfsynr1
{
    uint32_t U;
    struct _hw_smmu_sgfsynr1_bitfields
    {
        uint32_t SID : 16;
        uint32_t SSD : 16;
    } B;
} hw_smmu_sgfsynr1_t;

/*!
 * @name Constants and macros for entire SMMU_SGFSYNR1 register
 */
/*@{*/
#define HW_SMMU_SGFSYNR1_ADDR(x)        ((uintptr_t)(x) + 0x54U)
#define HW_SMMU_SGFSYNR1(x)             (*(__IO hw_smmu_sgfsynr1_t *) HW_SMMU_SGFSYNR1_ADDR(x))

#define HW_SMMU_NSGFSYNR1_ADDR(x)       ((uintptr_t)(x) + 0x454U)
#define HW_SMMU_NSGFSYNR1(x)            (*(__IO hw_smmu_sgfsynr1_t *) HW_SMMU_NSGFSYNR1_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_SGFSYNR2 - Global Fault Syndrome Register 2
 ******************************************************************************/

/*!
 * @name Constants and macros for entire SMMU_SGFSYNR2 register
 */
/*@{*/
#define HW_SMMU_SGFSYNR2_ADDR(x)        ((uintptr_t)(x) + 0x58U)
#define HW_SMMU_SGFSYNR2(x)             (*(__IO uint32_t *) HW_SMMU_SGFSYNR2_ADDR(x))

#define HW_SMMU_NSGFSYNR2_ADDR(x)       ((uintptr_t)(x) + 0x458U)
#define HW_SMMU_NSGFSYNR2(x)            (*(__IO uint32_t *) HW_SMMU_NSGFSYNR2_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_STLBIALL - TLB Invalidate All
 ******************************************************************************/

/*!
 * @name Constants and macros for entire SMMU_STLBIALL register
 */
/*@{*/
#define HW_SMMU_STLBIALL_ADDR(x)        ((uintptr_t)(x) + 0x60U)
#define HW_SMMU_STLBIALL(x)             (*(__O uint32_t *) HW_SMMU_STLBIALL_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_TLBIVMID - TLB Invalidate by VMID
 ******************************************************************************/

/*!
 * @brief TLBIVMID - TLB Invalidate by VMID (WO)
 *
 */
typedef union _hw_smmu_tlbivmid
{
    uint32_t U;
    struct _hw_smmu_tlbivmid_bitfields
    {
        uint32_t VMID : 8;
        uint32_t RESERVED0 : 24;
    } B;
} hw_smmu_tlbivmid_t;

/*!
 * @name Constants and macros for entire SMMU_TLBIVMID register
 */
/*@{*/
#define HW_SMMU_TLBIVMID_ADDR(x)        ((uintptr_t)(x) + 0x64U)
#define HW_SMMU_TLBIVMID(x)             (*(__O hw_smmu_sgfsynr1_t *) HW_SMMU_TLBIVMID_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_TLBIALLNSNH - TLB Invalidate All Non-Secure Non-Hyp
 ******************************************************************************/

/*!
 * @name Constants and macros for entire SMMU_TLBIALLNSNH register
 */
/*@{*/
#define HW_SMMU_TLBIALLNSNH_ADDR(x)        ((uintptr_t)(x) + 0x68U)
#define HW_SMMU_TLBIALLNSNH(x)             (*(__O uint32_t *) HW_SMMU_TLBIALLNSNH_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_TLBIALLNSNH - TLB Invalidate All Hyp
 ******************************************************************************/

/*!
 * @name Constants and macros for entire SMMU_TLBIALLH register
 */
/*@{*/
#define HW_SMMU_TLBIALLH_ADDR(x)        ((uintptr_t)(x) + 0x6CU)
#define HW_SMMU_TLBIALLH(x)             (*(__O uint32_t *) HW_SMMU_TLBIALLH_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_STLBGSYNC - Global Synchronize TLB Invalidate
 ******************************************************************************/

/*!
 * @name Constants and macros for entire SMMU_STLBGSYNC register
 */
/*@{*/
#define HW_SMMU_STLBGSYNC_ADDR(x)        ((uintptr_t)(x) + 0x70U)
#define HW_SMMU_STLBGSYNC(x)             (*(__O uint32_t *) HW_SMMU_STLBGSYNC_ADDR(x))

#define HW_SMMU_NSTLBGSYNC_ADDR(x)       ((uintptr_t)(x) + 0x470U)
#define HW_SMMU_NSTLBGSYNC(x)            (*(__O uint32_t *) HW_SMMU_NSTLBGSYNC_ADDR(x))
/*@}*/

/*******************************************************************************
 * HW_SMMU_STLBGSTATUS - Global TLB Status register
 ******************************************************************************/

/*!
 * @brief STLBGSTATUS - Global TLB Status register (RO)
 *
 */
typedef union _hw_smmu_stlbgstatus
{
    uint32_t U;
    struct _hw_smmu_stlbgstatus_bitfields
    {
        uint32_t GSACTIVE : 1;
        uint32_t RESERVED0 : 31;
    } B;
} hw_smmu_stlbgstatus;

/*!
 * @name Constants and macros for entire STLBGSTATUS register
 */
/*@{*/
#define HW_SMMU_STLBGSTATUS_ADDR(x)        ((uintptr_t)(x) + 0x74U)
#define HW_SMMU_STLBGSTATUS(x)             (*(__I _hw_smmu_stlbgstatus *) HW_SMMU_STLBGSTATUS_ADDR(x))

#define HW_SMMU_NSTLBGSTATUS_ADDR(x)       ((uintptr_t)(x) + 0x474U)
#define HW_SMMU_NSTLBGSTATUS(x)            (*(__I _hw_smmu_stlbgstatus *) HW_SMMU_NSTLBGSTATUS_ADDR(x))
/*@}*/


/*******************************************************************************
 * HW_SMMU_SMR - Stream Match Register
 ******************************************************************************/

/*!
 * @brief HW_SMMU_SMR - Stream Match Register (RW)
 *
 */
typedef union _hw_smmu_smr
{
    uint32_t U;
    struct _hw_smmu_smr_bitfields1
    {
        uint32_t ID : 15;
        uint32_t RESERVED0 : 1;
        uint32_t MASK : 15;
        uint32_t VALID : 1;
    } B1;
    struct _hw_smmu_smr_bitfields2
    {
        uint32_t EXID : 16;
        uint32_t EXMASK : 16;
    } B2;    
} hw_smmu_smr_t;

/*!
 * @name Constants and macros for entire SMMU_SMR register
 */
/*@{*/
#define HW_SMMU_SMR_ADDR(x, n)     ((uintptr_t)(x) + 0x800U + (0x4U * (n)))
#define HW_SMMU_SMR(x, n)          (*(__IO hw_smmu_smr_t *) HW_SMMU_SMR_ADDR(x, n))
/*@}*/

/*******************************************************************************
 * HW_SMMU_S2CR - Stream-to-Context Register
 ******************************************************************************/

/*!
 * @brief HW_SMMU_S2CR - Stream-to-Context Register (RW)
 *
 */
typedef union _hw_smmu_s2cr
{
    uint32_t U;
    struct _hw_smmu_s2cr_bitfields0
    {
        uint32_t CBNDX : 8;
        uint32_t SHCFG : 2;
        uint32_t EXIDVALID : 1;
        uint32_t MTCFG : 1;
        uint32_t MEMATTR : 4;
        uint32_t TYPE : 2;
        uint32_t NSCFG : 2;
        uint32_t RACFG : 2;
        uint32_t WACFG : 2;
        uint32_t PRIVCFG : 2;
        uint32_t INSTCFG : 2;
        uint32_t TRANSIENTCFG : 2;
        uint32_t RESERVED0 : 2;
    } B0;
    struct _hw_smmu_s2cr_bitfields1
    {
        uint32_t VMID : 8;
        uint32_t SHCFG : 2;
        uint32_t EXIDVALID : 1;
        uint32_t MTCFG : 1;
        uint32_t MEMATTR : 4;
        uint32_t TYPE : 2;
        uint32_t NSCFG : 2;
        uint32_t RACFG : 2;
        uint32_t WACFG : 2;
        uint32_t BSU : 2;
        uint32_t FB : 1;
        uint32_t RESERVED0 : 1;
        uint32_t TRANSIENTCFG : 2;
        uint32_t RESERVED1 : 2;
    } B1;
    struct _hw_smmu_s2cr_bitfields2
    {
        uint32_t RESERVED0 : 16;
        uint32_t TYPE : 2;
        uint32_t RESERVED1 : 14;
    } B2;
} hw_smmu_s2cr_t;

/*!
 * @name Constants and macros for entire SMMU_S2CR register
 */
/*@{*/
#define HW_SMMU_S2CR_ADDR(x, n)     ((uintptr_t)(x) + 0xC00U + (0x4U * (n)))
#define HW_SMMU_S2CR(x, n)          (*(__IO hw_smmu_s2cr_t *) HW_SMMU_S2CR_ADDR(x, n))
/*@}*/

/*******************************************************************************
 * HW_SMMU_CBAR - Stream-to-Context Register
 ******************************************************************************/

/*!
 * @brief HW_SMMU_CBAR - Context Bank Attribute Register (RW)
 *
 */
typedef union _hw_smmu_cbar
{
    uint32_t U;
    struct _hw_smmu_cbar_bitfields0
    {
        uint32_t VMID : 8;
        uint32_t RESERVED0 : 8;
        uint32_t TYPE : 2;
        uint32_t SBZ : 2;
        uint32_t RESERVED1 : 4;
        uint32_t IRPTNDX : 8;
    } B0;
    struct _hw_smmu_cbar_bitfields1
    {
        uint32_t VMID : 8;
        uint32_t BPSHCFG : 2;
        uint32_t HYPC : 1;
        uint32_t FB : 1;
        uint32_t MEMATTR : 4;
        uint32_t TYPE : 2;
        uint32_t BSU : 2;
        uint32_t RACFG : 2;
        uint32_t WACFG : 2;
        uint32_t IRPTNDX : 8;
    } B1;
    struct _hw_smmu_cbar_bitfields2
    {
        uint32_t VMID : 8;
        uint32_t RESERVED0 : 8;
        uint32_t TYPE : 2;
        uint32_t SBZ : 2;
        uint32_t RESERVED1 : 4;
        uint32_t IRPTNDX : 8;
    } B2;
    struct _hw_smmu_cbar_bitfields3
    {
        uint32_t VMID : 8;
        uint32_t CBNDX : 8;
        uint32_t TYPE : 2;
        uint32_t SBZ : 2;
        uint32_t RESERVED0 : 4;
        uint32_t IRPTNDX : 8;
    } B3;
} hw_smmu_cbar_t;

/*!
 * @name Constants and macros for entire SMMU_CBAR register
 */
/*@{*/
#define HW_SMMU_CBAR_ADDR(x, n)     ((uintptr_t)(x) + 0x10000U + (0x4U * (n)))
#define HW_SMMU_CBAR(x, n)          (*(__IO hw_smmu_cbar_t *) HW_SMMU_CBAR_ADDR(x, n))
/*@}*/

/*******************************************************************************
 * HW_SMMU_CBFRSYNRA - Context Bank Fault Restricted Syndrome Register A
 ******************************************************************************/

/*!
 * @brief CBFRSYNRA - Context Bank Fault Restricted Syndrome Register A (RW)
 *
 */
typedef union _hw_smmu_cbfrsynra
{
    uint32_t U;
    struct _hw_smmu_cbfrsynra_bitfields
    {
        uint32_t SID : 16;
        uint32_t SSD : 16;
    } B;
} hw_smmu_cbfrsynra_t;

/*!
 * @name Constants and macros for entire SMMU_CBFRSYNRA register
 */
/*@{*/
#define HW_SMMU_CBFRSYNRA_ADDR(x, n)     ((uintptr_t)(x) + 0x10400U + (0x4U * (n)))
#define HW_SMMU_CBFRSYNRA(x, n)          (*(__IO hw_smmu_cbfrsynra_t *) HW_SMMU_CBFRSYNRA_ADDR(x, n))
/*@}*/

/*******************************************************************************
 * HW_SMMU_CBA2R - Stream-to-Context Register 2
 ******************************************************************************/

/*!
 * @brief HW_SMMU_CBA2R - Context Bank Attribute Register 2 (RW)
 *
 */
typedef union _hw_smmu_cba2r
{
    uint32_t U;
    struct _hw_smmu_cba2r_bitfields0
    {
        uint32_t VA64 : 1;
        uint32_t MONC : 1;
        uint32_t RESERVED0 : 30;
    } B;
} hw_smmu_cba2r_t;

/*!
 * @name Constants and macros for entire SMMU_CBA2R register
 */
/*@{*/
#define HW_SMMU_CBA2R_ADDR(x, n)     ((uintptr_t)(x) + 0x10800U + (0x4U * (n)))
#define HW_SMMU_CBA2R(x, n)          (*(__IO hw_smmu_cba2r_t *) HW_SMMU_CBA2R_ADDR(x, n))
/*@}*/

/*******************************************************************************
 * hw_smmu_t - module struct
 ******************************************************************************/
/*!
 * @brief All SMMU module registers.
 */
#pragma pack(1)
typedef struct _hw_smmu
{
    /* GR0 - 64K */
    __IO hw_smmu_scr0_t SCR0;
    __IO hw_smmu_scr0_t SCR1;
    __IO hw_smmu_scr0_t SCR2;
         uint32_t _reserved0;   
    __IO hw_smmu_sacr_t SACR;
         uint32_t _reserved1[3];   
    __I  hw_smmu_idr0_t IDR0;
    __I  hw_smmu_idr1_t IDR1;
    __I  hw_smmu_idr2_t IDR2;
    __I  uint32_t IDR3;
    __I  uint32_t IDR4;
    __I  uint32_t IDR5;
    __I  uint32_t IDR6;
    __I  hw_smmu_idr7_t IDR7;
    __IO uint32_t SGFAR[2];
    __IO hw_smmu_sgfsr_t SGFSR;
    __O  uint32_t SFSRRESTORE;
    __IO hw_smmu_sgfsynr0_t SGFSYNR0;
    __IO hw_smmu_sgfsynr1_t SGFSYNR1;
    __IO uint32_t SGFSYNR2;
         uint32_t _reserved2; 
    __O  uint32_t STLBIALL;
    __O  hw_smmu_tlbivmid_t TLBIVMID;
    __O  uint32_t TLBIALLNSNH;
    __O  uint32_t TLBIALLH;
    __O  uint32_t STLBGSYNC;
    __I  hw_smmu_stlbgstatus STLBGSTATUS;
    __O  uint32_t TLBIVAH[2];
    __IO uint32_t DBGRPTRTBU;
    __I  uint32_t DBGRDATATBU;
    __IO uint32_t DBGRPTRTCU;
    __I  uint32_t DBGRDATATCU;  
         uint32_t _reserved3[4]; 
    __O  uint32_t STLBIVALM[2];
    __O  uint32_t STLBIVAM[2];
    __O  uint32_t TLBIVALH64[2];
    __O  uint32_t TLBIVMIDS1;
    __O  uint32_t STLBIALLM;
    __O  uint32_t TLBIVAH64[2];
         uint32_t _reserved4[14]; 
    __O  uint32_t SGATS1UR[2];
    __O  uint32_t SGATS1UW[2];
    __O  uint32_t SGATS1PR[2];
    __O  uint32_t SGATS1PW[2];
    __O  uint32_t SGATS12UR[2];
    __O  uint32_t SGATS12UW[2];
    __O  uint32_t SGATS12PR[2];
    __O  uint32_t SGATS12PW[2]; 
         uint32_t _reserved5[16]; 
    __IO uint32_t SGPAR[2];
    __IO uint32_t SGATSR;
        uint32_t _reserved6[157]; 
    __IO hw_smmu_scr0_t NSCR0;
         uint32_t _reserved7; 
    __IO hw_smmu_scr2_t NSCR2;
         uint32_t _reserved8; 
    __IO hw_smmu_sacr_t NSACR;  
         uint32_t _reserved9[11];
    __IO uint32_t NSGFAR[2];
    __IO hw_smmu_sgfsr_t NSGFSR;
    __O  uint32_t NSGFSRRESTORE;
    __IO hw_smmu_sgfsynr0_t NSGFSYNR0;
    __IO hw_smmu_sgfsynr1_t NSGFSYNR1;
    __IO uint32_t NSGFSYNR2;
         uint32_t _reserved10[5]; 
    __O  uint32_t NSTLBGSYNC;
    __I  hw_smmu_stlbgstatus NSTLBGSTATUS;
         uint32_t _reserved11[34]; 
    __O  uint32_t NSGATS1UR[2];
    __O  uint32_t NSGATS1UW[2];
    __O  uint32_t NSGATS1PR[2];
    __O  uint32_t NSGATS1PW[2];
    __O  uint32_t NSGATS12UR[2];
    __O  uint32_t NSGATS12UW[2];
    __O  uint32_t NSGATS12PR[2];
    __O  uint32_t NSGATS12PW[2]; 
         uint32_t _reserved12[16]; 
    __IO uint32_t NSGPAR[2];
    __IO uint32_t NSGATSR;
         uint32_t _reserved13[157]; 
    __IO hw_smmu_smr_t SMR[128];    
         uint32_t _reserved14[128];   
    __IO hw_smmu_s2cr_t S2CR[128];    
         uint32_t _reserved15[128];
         uint8_t _reserved16[SMMU_PAGESIZE - 4096];
    /* GR 1 - 64K */
    __IO hw_smmu_cbar_t CBAR[128];    
         uint32_t _reserved20[128];
         hw_smmu_cbfrsynra_t CBFRSYNRA[128];
         uint32_t _reserved21[128];        
    __IO hw_smmu_cba2r_t CBA2R[128];    
         uint32_t _reserved22[384];
         uint8_t _reserved23[SMMU_PAGESIZE - 4096];
    /* Blank Global */
         uint8_t _reserved30[SMMU_NUM_CB - 2][SMMU_PAGESIZE];    
    /* CB */
         uint8_t _reserved40[SMMU_NUM_CB][SMMU_PAGESIZE];    
} hw_smmu_t;
#pragma pack()

/*! @brief Macro to access all SMMU registers. */
/*! @param x SMMU module instance base address. */
/*! @return Reference (not a pointer) to the registers struct. To get a pointer to the struct,
 *     use the '&' operator, like <code>&HW_SMMU(SMMU0_BASE)</code>. */
#define HW_SMMU(x)      (*(hw_smmu_t *)(x))

#define SMMU0   ((hw_smmu_t*) SMMU0_BASE)

#endif /* HW_SMMU_REGISTERS_H */

