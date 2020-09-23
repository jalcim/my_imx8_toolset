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
 * Common code used by all board ports. Common shims for object package
 * recompile (tests, monitor, etc.) Some common DRC code.
 */
/*==========================================================================*/

/* Includes */

#include "main/main.h"
#include "main/board.h"
#include "main/monitor.h"
#include "test/test.h"

/* Forced Error */

#ifdef FORCE_COMPILE_ERROR
    #error Forced error for testing
#endif

/* Local Defines */

#define CALL_TEST_DRV(X) TEMP_TEST_DRV(X)
#define TEMP_TEST_DRV(X) test_drv_##X

#define CALL_TEST_SC(X) TEMP_TEST_SC(X)
#define TEMP_TEST_SC(X) test_sc_##X

#define CALL_TEST_AP(X) TEMP_TEST_AP(X)
#define TEMP_TEST_AP(X) test_ap_##X

/* Local Functions */

/* Local Variables */

static sc_bool_t ddr_tick_enable = SC_FALSE;

/* Global Variables */

#ifdef DEBUG
    sc_bool_t debug = SC_TRUE;
#else
    sc_bool_t debug = SC_FALSE;
#endif

#ifdef HAS_TEST
    sc_bool_t has_test = SC_TRUE;
#else
    sc_bool_t has_test = SC_FALSE;
#endif

#ifdef TEST_ALL
    sc_bool_t test_all = SC_TRUE;
#else
    sc_bool_t test_all = SC_FALSE;
#endif

#if defined(MONITOR) || defined(EXPORT_MONITOR)
    sc_bool_t has_monitor = SC_TRUE;
#else
    sc_bool_t has_monitor = SC_FALSE;
#endif

/*--------------------------------------------------------------------------*/
/* Driver test shim                                                         */
/*--------------------------------------------------------------------------*/
sc_err_t test_drv(sc_bool_t *const stop)
{
    #ifdef HAS_TEST
        CALL_TEST_DRV(TEST)(stop);
    #else
        *stop = SC_FALSE;
    #endif

    return SC_ERR_NONE;
}

/*--------------------------------------------------------------------------*/
/* SCU test shim                                                            */
/*--------------------------------------------------------------------------*/
sc_err_t test_sc(sc_bool_t *const stop)
{
    #ifdef HAS_TEST
        CALL_TEST_SC(TEST)(stop);
    #else
        *stop = SC_FALSE;
    #endif

    return SC_ERR_NONE;
}

/*--------------------------------------------------------------------------*/
/* AP test shim                                                             */
/*--------------------------------------------------------------------------*/
sc_err_t test_ap(sc_bool_t *const stop)
{
    #ifdef HAS_TEST
        CALL_TEST_AP(TEST)(stop);
    #else
        *stop = SC_FALSE;
    #endif

    return SC_ERR_NONE;
}

/*--------------------------------------------------------------------------*/
/* Run monitor                                                              */
/*--------------------------------------------------------------------------*/
void board_monitor(void)
{
    #ifdef MONITOR
        monitor();
    #endif
}

/*--------------------------------------------------------------------------*/
/* DDR periodic enable                                                      */
/*--------------------------------------------------------------------------*/
void board_ddr_periodic_enable(sc_bool_t enb)
{
    if (board_ddr_period_ms != 0U)
    {
        ddr_tick_enable = enb;
    }
}

/*--------------------------------------------------------------------------*/
/* Board tick                                                               */
/*--------------------------------------------------------------------------*/
void board_tick(uint16_t msec)
{
    if (ddr_tick_enable != SC_FALSE)
    {
        static uint32_t ddr_mseconds = 0U;

        /* Tick fake RTC */
        ddr_mseconds += msec;

        if (ddr_mseconds >= board_ddr_period_ms)
        {
            ddr_mseconds = 0U;

            (void) board_ddr_config(SC_FALSE, BOARD_DDR_PERIODIC);
        }
    }
}

