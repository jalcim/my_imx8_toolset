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
 * Header file with prototypes so main can call test functions.
 */
/*==========================================================================*/

#ifndef SC_TEST_H
#define SC_TEST_H

/* Includes */

#include "main/types.h"

/* Defines */

#ifdef DEBUG
#ifdef HAS_TEST_PTIM
    #define CHECK_NERR(X)   {SC_PTIM_SETREF(testProf, __FILE__, __LINE__);  \
                            err = (X);                                      \
                            if (err != SC_ERR_NONE)                         \
                            {                                               \
                                error_print("error @ line %d: %d\n",        \
                                    __LINE__, err);                         \
                                main_exit(err);                             \
                            }}
#else
    #define CHECK_NERR(X)   {err = (X);                                     \
                            if (err != SC_ERR_NONE)                         \
                            {                                               \
                                error_print("error @ line %d: %d\n",        \
                                    __LINE__, err);                         \
                                main_exit(err);                             \
                            }}
#endif // HAS_TEST_PTIM
    #ifdef CONFIRM_ERR
#ifdef HAS_TEST_PTIM
        #define CHECK_ERR(X)    {SC_PTIM_SETREF(testProf, __FILE__, __LINE__);  \
                                err = (X);                                      \
                                if ((err = (X)) == SC_ERR_NONE)                 \
                                {                                               \
                                    error_print("non error @ line %d: %d\n",    \
                                       __LINE__, err);                          \
                                    main_exit(err);                             \
                                }}
#else
        #define CHECK_ERR(X)    {err = (X);                                     \
                                if (err == SC_ERR_NONE)                         \
                                {                                               \
                                    error_print("non error @ line %d: %d\n",    \
                                       __LINE__, err);                          \
                                    main_exit(err);                             \
                                }}
#endif
        #else
        #define CHECK_ERR(X)    NOP
    #endif

#else
    #define CHECK_NERR(X)   X
    #define CHECK_ERR(X)    X
#endif

#define ASRT_TST_ERR(X,ERROR)                       \
    if (!(X))                                       \
    {                                               \
        error_print("error @ line %d: %d\n",        \
           __LINE__, ERROR);                        \
        main_exit(ERROR);                           \
    }

#if HAS_SS_LSIO || HAS_SS_HSLSIO
    #define CHECK_LSIO(X)       NOP
#else    
    #define CHECK_LSIO(X)       return (X)
#endif

#if HAS_SS_M4_0
    #define CHECK_M4_0(X)       NOP
#else    
    #define CHECK_M4_0(X)       return (X)
#endif

#if HAS_SS_M4_1
    #define CHECK_M4_1(X)       NOP
#else    
    #define CHECK_M4_1(X)       return (X)
#endif

#define TEST_DRV_START(X) \
    test_print(TL1, "\n*** %s DRV Test *********************\n\n", X);  

#define TEST_DRV_END \
    test_print(TL1, "\n*************************************\n\n")

#define TEST_SC_START(X) \
    test_print(TL1, "\n*** %s SC Test **********************\n\n", X); \
    sc_pm_power_mode_t dblogic_mode; \
    (void) pm_get_resource_power_mode(SC_PT, SC_R_DBLOGIC, &dblogic_mode); \
    (void) pm_force_resource_power_mode(SC_R_DBLOGIC, SC_PM_PW_MODE_ON); \
    sc_pm_power_mode_t db_mode; \
    (void) pm_get_resource_power_mode(SC_PT, SC_R_DB, &db_mode); \
    (void) pm_force_resource_power_mode(SC_R_DB, SC_PM_PW_MODE_ON)

#define TEST_SC_END \
    if (db_mode == SC_PM_PW_MODE_OFF) \
        (void) pm_force_resource_power_mode(SC_R_DB, SC_PM_PW_MODE_OFF); \
    if (dblogic_mode == SC_PM_PW_MODE_OFF) \
        (void) pm_force_resource_power_mode(SC_R_DBLOGIC, SC_PM_PW_MODE_OFF); \
    test_print(TL1, "\n*************************************\n\n")

#define TEST_AP_START(X) \
    sc_ipc_t ipc_sc; \
    test_print(TL1, "\n*** %s AP Test **********************\n\n", X); \
    sc_ipc_open(&ipc_sc, (sc_ipc_id_t) MU_SC_1A); \
    (void) sc_rm_set_peripheral_permissions(ipc_sc, SC_R_MU_0A, SC_PT, SC_RM_PERM_FULL)

#define TEST_AP_END \
        sc_ipc_close(ipc_sc); \
        test_print(TL1, "\n*************************************\n\n")

#define TEST_DRV(X)     sc_err_t test_drv_ ## X(sc_bool_t *const stop)
#define TEST_SC(X)      sc_err_t test_sc_ ## X(sc_bool_t *const stop)
#define TEST_AP(X)      sc_err_t test_ap_ ## X(sc_bool_t *const stop)

#define TEST_PROTO(X)   sc_err_t test_drv_ ## X(sc_bool_t *const stop); \
                        sc_err_t test_sc_ ## X(sc_bool_t *const stop);  \
                        sc_err_t test_ap_ ## X(sc_bool_t *const stop);

#define DBGSTR_PU   "Power up %s\n"
#define DBGSTR_PD   "Power down %s\n"

/* Types */

/* Enums */

/* Sub Includes */

/* Functions */

#ifdef HAS_TEST
TEST_PROTO(a5x)              
TEST_PROTO(all)              
TEST_PROTO(audio)            
TEST_PROTO(boot)             
TEST_PROTO(cci)              
TEST_PROTO(conn)             
TEST_PROTO(critsec)          
TEST_PROTO(db)               
TEST_PROTO(dblogic)          
TEST_PROTO(dc)               
TEST_PROTO(ddr)              
TEST_PROTO(ddr_selfrefresh)  
TEST_PROTO(ddr_retention)
TEST_PROTO(ddr_stress)       
TEST_PROTO(dma)              
TEST_PROTO(drc)              
TEST_PROTO(drv)              
TEST_PROTO(dsc)              
TEST_PROTO(examples)         
TEST_PROTO(fusedump)
TEST_PROTO(fusewrite)
TEST_PROTO(gpu)              
TEST_PROTO(hexdump)          
TEST_PROTO(hmppm)            
TEST_PROTO(hsio)             
TEST_PROTO(img)              
TEST_PROTO(isi)
TEST_PROTO(lpi2c)
TEST_PROTO(lsio)
TEST_PROTO(m4)
TEST_PROTO(membw)
TEST_PROTO(nmi_ecc)
TEST_PROTO(nmi_lockup)
TEST_PROTO(nmi_msi)
TEST_PROTO(obd)
TEST_PROTO(otp)
TEST_PROTO(pllchar)
TEST_PROTO(pm)
TEST_PROTO(pmic)
TEST_PROTO(profile)
TEST_PROTO(reboot)
TEST_PROTO(refgencal)
TEST_PROTO(rm)
TEST_PROTO(scgpio)
TEST_PROTO(seco)
TEST_PROTO(temp)
TEST_PROTO(timer)
TEST_PROTO(timers)
TEST_PROTO(uboot)
TEST_PROTO(vpu)
TEST_PROTO(smmu)
#endif

/* External Variables. */
extern sc_bool_t dma_up_down;
extern sc_bool_t xrdc_enable;

#endif /* SC_TEST_H */

