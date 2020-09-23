/*
** ###################################################################
**
**     Copyright (c) 2016 Freescale Semiconductor, Inc.
**     Copyright 2017-2018 NXP
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
 * Header file for the system controller main. Contains defines, macros,
 * and types shared by main and test implementations.
 */
/*==========================================================================*/

#ifndef SC_MAIN_H
#define SC_MAIN_H

/* Includes */

#include "main/scfw.h"
#include "main/types.h"
#include "main/ipc.h"
#include "board/board_common.h"
#include "config/config.h"
#include "main/debug.h"
#include "ss/inf/inf.h"

/* Defines */

#define SC_NA               0U

#define SC_NUM_SS_GRP       (SC_SS_GRP_LAST + 1U)
#define SC_NUM_RESOURCE     (SC_R_LAST)
#define SC_NUM_SUBSYS       ((sc_sub_t) (((uint8_t) SC_SUBSYS_LAST) + 1U))
#define SC_NUM_DSC          ((sc_dsc_t) (((uint8_t) SC_DSC_LAST) + 1U))

#define SC_MSIZE_W          5U
#define SC_MSLOT_W          2U
#define SC_MSLOT_SHF        30U
#define SC_SSLOT_W          6U
#define SC_SSLOT_SHF        24U

#define NOP                 {}

#ifndef SIMU
    #define ENTER_CS        SystemEnterCS()
    #define EXIT_CS         SystemExitCS()
#else
    #define ENTER_CS        NOP
    #define EXIT_CS         NOP
#endif

#define DONT_TOUCH_RSRC       0U
#define LEAVE_RSRC_ON         1U
#define TURN_RSRC_OFF         2U
#define SET_RSRC_STBY         3U

/*! Macro to get minimum */
#if !defined(MIN)
    #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#if !defined(MAX)
    /*! Macro to get maximum */
    #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/*! Macro to get absolute value */
#define ABS(X)              ((X) < 0 ? -(X) : (X))

/*! Macro to create bit field */
#define BIT(X)              (U32(1U) << (U32(X) % U32(32U)))

/*! Macro to return bit position */
#define BIT_POS(X)          (U32(X) % U32(32U))

/*! Macro to create bit field */
#define BIT8(X)             (U8(1U) << (U8(X) % U8(8U)))

/*! Macro to create bit field */
#define BIT16(X)            (U16(1U) << (U16(X) % U16(16U)))

/*! Macro to create bit field */
#define BIT32(X)            (U32(1U) << (U32(X) % U32(32U)))

/*! Macro to create bit field */
#define BIT64(X)            (U64(1U) << (U64(X) % U64(64U)))

/*! Macro to extract from field */
#define MASK(X)             ((U32(1U) << U32(X)) - U32(1U))

/*! Macro to extract from bit field */
#define EX_BIT(X, Y)        ((U32(X) >> (U32(Y) % 32U)) & U32(1U))

/*! Macro to extract from bool field */
#define EX_BOOL(X, Y)        (U2B32(EX_BIT((X), (Y))))

/*! Macro to extract from field */
#define EX_FIELD(X, Y, Z)   (((X) >> (U32(Y) % 32U)) & MASK(Z))

/*! Macro to create clear field */
#define CLR_FIELD(X, Y)     (MASK(Y) << (U32(X) % U32(32U)))

/*! Macro to create field */
#define INS_FIELD(X, Y, Z)  ((U32(Z) & MASK(Y)) << (U32(X) % 32U))

/*! Macro to create register index */
#define REG(X)              (U32(X) / U32(32U))

/*! Macro to create 32-bit index from register and bit */
#define REGBIT(X, Y)        ((U32(X) * U32(32U)) + (U32(Y) % 32U))

/*! Macro to create 64-bit index from register and bit */
#define REGBIT64(X, Y)      ((U64(X) * U64(32U)) + (U64(Y) % 32U))

/*! Macro to get upper 32 bits of a 64-bit value */
#define UINT64_H(X)         (U32((U64(X) >> 32U) & U64(0x0FFFFFFFFU)))

/*! Macro to get lower 32 bits of a 64-bit value */
#define UINT64_L(X)         (U32(U64(X) & U64(0x0FFFFFFFFU)))

/*! Macro to get upper 16 bits of a 32-bit value */
#define UINT32_H(X)         (U16((U32(X) >> 16U) & U32(0xFFFFU)))

/*! Macro to get lower 16 bits of a 32-bit value */
#define UINT32_L(X)         (U16(U32(X) & U32(0xFFFFU)))

#define ASRT(X)                         \
    if (!(X))                           \
    {                                   \
        return;                         \
    }

#define ASRT_ERR(X,ERROR)               \
    if (!(X))                           \
    {                                   \
        return (ERROR);                 \
    }

#define ASRT_C(X)                       \
    if (!(X))                           \
    {                                   \
        continue;                       \
    }

#define RTN_ERR(X)                      \
    if ((err = (X)) != SC_ERR_NONE)     \
    {                                   \
        return err;                     \
    }

#define RTN(X)                          \
    if ((X) != SC_ERR_NONE)             \
    {                                   \
        return;                         \
    }

#define HALT                            \
    do                                  \
    {                                   \
    }                                   \
    while(true)

/*!
 * @name Parameter checking macros
 */
/*@{*/
#define BOUND_PT(X)           ASRT_ERR((X) < SC_RM_NUM_PARTITION, SC_ERR_PARM)
#define BOUND_PT_F(X)         ASRT_ERR((X) < SC_RM_NUM_PARTITION, SC_FALSE)
#define BOUND_PT_V(X)         ASRT((X) < SC_RM_NUM_PARTITION)
#define BOUND_RSRC(X,I)       ASRT_ERR(rm_check_map_ridx((X), &(I)) != SC_FALSE, SC_ERR_PARM)
#define BOUND_RSRC_F(X,I)     ASRT_ERR(rm_check_map_ridx((X), &(I)) != SC_FALSE, SC_FALSE)
#define BOUND_RSRC_C(X,I)     ASRT_C(rm_check_map_ridx((X), &(I)) != SC_FALSE)
#define BOUND_MR(X)           ASRT_ERR((X) < SC_RM_NUM_MEMREG, SC_ERR_PARM)
#define BOUND_MR_F(X)         ASRT_ERR((X) < SC_RM_NUM_MEMREG, SC_FALSE)
#define BOUND_PAD(X)          ASRT_ERR((X) < SC_NUM_PAD, SC_ERR_PARM)
#define BOUND_PAD_F(X)        ASRT_ERR((X) < SC_NUM_PAD, SC_FALSE)
#define USED_PT(X)            ASRT_ERR(rm_is_partition_used(X) != SC_FALSE, SC_ERR_PARM)
#define USED_PT_V(X)          ASRT(rm_is_partition_used(X) != SC_FALSE)
#define ANCESTOR(X)           ASRT_ERR(rm_check_ancestor(caller_pt, (X)) != SC_FALSE, SC_ERR_NOACCESS)
#define ANCESTOR_C(X)         ASRT_C(rm_check_ancestor(caller_pt, (X)) != SC_FALSE)
#define OWNED(X)              ASRT_ERR(rm_is_resource_owned(caller_pt, (X)) != SC_FALSE, SC_ERR_NOACCESS)
#define NOT_SC_PT(X)          ASRT_ERR((X) != SC_PT, SC_ERR_NOACCESS)
#define MASTER(X)             ASRT_ERR(rm_is_ridx_master(X) != SC_FALSE, SC_ERR_PARM)
#define MASTER_C(X)           ASRT_C(rm_is_ridx_master(X) != SC_FALSE)
#define PERIPHERAL(X)         ASRT_ERR(rm_is_ridx_peripheral(X) != SC_FALSE, SC_ERR_PARM)
#define PERIPHERAL_C(X)       ASRT_C(rm_is_ridx_peripheral(X) != SC_FALSE)
#define ACCESS_ALLOWED(X,I)   ASRT_ERR(rm_is_ridx_access_allowed((X), (I)) != SC_FALSE, SC_ERR_NOACCESS)
#define ACCESS_ALLOWED_C(X,I) ASRT_C(rm_is_ridx_access_allowed((X), (I)) != SC_FALSE)
/*@}*/

/*!
 * @name Conversion Macros
 */
/*@{*/
#define I8(X)     ((int8_t) (X))
#define I16(X)    ((int16_t) (X))
#define I32(X)    ((int32_t) (X))
#define I64(X)    ((int64_t) (X))
#define U8(X)     ((uint8_t) (X))
#define U16(X)    ((uint16_t) (X))
#define U32(X)    ((uint32_t) (X))
#define U64(X)    ((uint64_t) (X))
/*@}*/

#define U2B(X)    (((X) != 0U) ? SC_TRUE : SC_FALSE)
#define U2B32(X)  (((X) != 0UL) ? SC_TRUE : SC_FALSE)
#define B2U8(X)   (((X) != SC_FALSE) ? U8(0x01U) : U8(0x00U))
#define B2U16(X)  (((X) != SC_FALSE) ? U16(0x01U) : U16(0x00U))
#define B2U32(X)  (((X) != SC_FALSE) ? U32(0x01U) : U32(0x00U))

/* Types */

typedef uint8_t sc_msize_t;
typedef uint8_t sc_mslot_t;
typedef uint8_t sc_sslot_t;

typedef const char * const strings[];

typedef union
{
    int8_t with_sign;
    uint8_t no_sign;
} cint8_t;

typedef union
{
    int16_t with_sign;
    uint16_t no_sign;
} cint16_t;

typedef union
{
    int32_t with_sign;
    uint32_t no_sign;
} cint32_t;

/*!
 * This type is used to declare constant subsystem information.
 */
typedef struct sc_ss_info_s
{
    sc_bool_t present  : SC_BOOL_W;      //!< True if SS present
    sc_ss_inst_t inst  : SC_SS_INST_W;   //!< Instance number
    sc_sub_t db_ssi    : SC_PGP_W;       //!< DB SSI this subsystem uses
    sc_sub_t parent    : SC_SUBSYS_W;    //!< Parent SS if this is a sub-SS
    sc_rm_idx_t r_ofs  : SC_RM_IDX_W;    //!< Offset into parent's resources
    sc_rm_idx_t r2_ofs : SC_RM_IDX_W;    //!< Second offset into parent's resources
    sc_dsc_t dsc       : SC_DSC_W;       //!< Primary DSC this subsystem uses
} sc_ss_info_t;

/*!
 * This type is used to declare constant memmap information. The
 * address range is the top-level memory map as seen by the AP cores.
 */
typedef struct sc_memmap_s
{
    sc_faddr_t start   : SC_FADDR_W;     //!< Start of subsystem address range
    sc_faddr_t len     : SC_FADDR_W;     //!< Length of of subsystem address range
    sc_bool_t mem      : SC_BOOL_W;      //!< Contains memory
    sc_bool_t ss_prot  : SC_BOOL_W;      //!< Protected via SS XRDC components
    sc_msize_t size    : SC_MSIZE_W;     //!< ZADDR size
    sc_mslot_t slot    : SC_MSLOT_W;     //!< ZADDR slot
    sc_sslot_t subslot : SC_SSLOT_W;     //!< ZADDR subslot
    sc_sub_t ss        : SC_SUBSYS_W;    //!< Associated subsystem
} sc_memmap_t;

/*!
 * This type is used to declare a u-boot's aarch64 image header.
 */
typedef struct ap_boot_img_s
{
    uint32_t branch[2];
    uint32_t dest_addr;
    uint32_t reserved;
    uint32_t size;
} ap_boot_img_t;

/*!
 * This type declares all the memory areas that can be used for boot.
 */
 typedef struct sc_boot_mem_s
{
    sc_faddr_t start  : SC_FADDR_W;     //!< Start of  memory address range
    sc_faddr_t len    : SC_FADDR_W;     //!< End of  memory address range
    sc_rsrc_t  rsrc   : SC_RSRC_W;      //!< Resource associated with memory.
    sc_sub_t   ss     : SC_SUBSYS_W;    //!< Subsystem associated with resource.
}sc_boot_mem_t;

/* Functions */

sc_err_t main_banners(void);
sc_err_t main_drv_test(void);
sc_err_t main_init(void);
sc_err_t main_sc_test(void);
sc_err_t main_config(sc_rm_pt_t *pt_boot,  sc_bool_t *early);
sc_err_t main_ddr(sc_bool_t early);
uint32_t main_dcd(void);
void main_prep_cpu(sc_rsrc_t boot_cpu, sc_rsrc_t boot_mu);
void main_dump(void);
void main_config_debug_uart(LPUART_Type *base, uint32_t rate);
void main_pll_debug(void);
void main_exit(int32_t status);

/* Externs */

extern sc_bool_t rom_loaded;
extern const char * const rnames[SC_NUM_RSRC];
extern const char * const pnames[SC_NUM_PAD];
extern const char * const snames[SC_SUBSYS_LAST + 1U];
extern const sc_ss_info_t sc_ss_info[SC_SUBSYS_LAST + 1U];
extern const sc_memmap_t sc_memmap[];
extern const ss_base_info_t * const ss_base_info[SC_SUBSYS_LAST + 1U];
extern volatile sc_bool_t rpc_debug;
extern sc_rm_idx_t rom_boot_rsrc[SC_NUM_RSRC];

#ifdef DEBUG
char term_emul_getc(void);
void term_emul_putc(char c);
#endif

#endif /* SC_MAIN_H */

