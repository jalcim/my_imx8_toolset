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
 * Header file containing the board API.
 *
 * @addtogroup BRD_SVC (BRD) Board Interface
 *
 * Module for the Board interface.
 *
 * @{
 */
/*==========================================================================*/

#ifndef SC_BOARD_API_H
#define SC_BOARD_API_H

/* Types */

/*!
 * Board config parameter types.
 */
typedef enum board_parm_e
{
    BOARD_PARM_PCIE_PLL         = 0,
    BOARD_PARM_KS1_RESUME_USEC  = 1,    /*!< Supply ramp delay in usec for KS1 exit */
    BOARD_PARM_KS1_RETENTION    = 2,    /*!< Controls if retention is applied during KS1 */
    BOARD_PARM_KS1_ONOFF_WAKE   = 3     /*!< Controls if ONOFF button can wake from KS1 */
} board_parm_t;

/*!
 * Board config parameter returns.
 */
typedef uint32_t board_parm_rtn_t;

#define BOARD_PARM_RTN_NOT_USED             0U
#define BOARD_PARM_RTN_USED                 1U
#define BOARD_PARM_RTN_EXTERNAL             2U
#define BOARD_PARM_RTN_INTERNAL             3U

#define BOARD_PARM_KS1_RETENTION_DISABLE    0U   /*!< Disable retention during KS1 */
#define BOARD_PARM_KS1_RETENTION_ENABLE     1U   /*!< Enable retention during KS1 */

#define BOARD_PARM_KS1_ONOFF_WAKE_DISABLE   0U   /*!< Disable ONOFF wakeup during KS1 */
#define BOARD_PARM_KS1_ONOFF_WAKE_ENABLE    1U   /*!< Enable ONOFF wakeup during KS1 */

/*!
 * Board reset event types for CPUs.
 */
typedef enum board_cpu_rst_ev_e
{
    BOARD_CPU_RESET_SELF        = 0,
    BOARD_CPU_RESET_WDOG        = 1,
    BOARD_CPU_RESET_LOCKUP      = 2,
    BOARD_CPU_RESET_MEM_ERR     = 3
} board_cpu_rst_ev_t;

/*!
 * DDR actions (power state transitions, etc.)
 */
typedef enum board_ddr_action_e
{
    BOARD_DDR_COLD_INIT         = 0,    /*!< Init DDR from POR */
    BOARD_DDR_PERIODIC          = 1,    /*!< Run periodic training */
    BOARD_DDR_SR_DRC_ON_ENTER   = 2,    /*!< Enter self-refresh (leave DRC on) */
    BOARD_DDR_SR_DRC_ON_EXIT    = 3,    /*!< Exit self-refresh (DRC was on) */
    BOARD_DDR_SR_DRC_OFF_ENTER  = 4,    /*!< Enter self-refresh (turn off DRC) */
    BOARD_DDR_SR_DRC_OFF_EXIT   = 5     /*!< Exit self-refresh (DRC was off) */
} board_ddr_action_t;

/* Macros */

/*!
 * @name Macros for DCD processing
 */
/*@{*/
#define DATA4(A, V) *((volatile uint32_t*)(A)) = U32(V)
#define SET_BIT4(A, V) *((volatile uint32_t*)(A)) |= U32(V)
#define CLR_BIT4(A, V) *((volatile uint32_t*)(A)) &= ~(U32(V))
#define CHECK_BITS_SET4(A, M) while((*((volatile uint32_t*)(A))         \
    & U32(M)) != ((uint32_t)(M))){}
#define CHECK_BITS_CLR4(A, M) while((*((volatile uint32_t*)(A))         \
    & U32(M)) != U32(0U)){}
#define CHECK_ANY_BIT_SET4(A, M) while((*((volatile uint32_t*)(A))      \
    & U32(M)) == U32(0U)){}
#define CHECK_ANY_BIT_CLR4(A, M) while((*((volatile uint32_t*)(A))      \
    & U32(M)) == U32(M)){}
/*@}*/

/*!
 * @name Macro for debug of board calls
 */
#define BRD_ERR(X)  {err = (X);                                 \
                    if (err != SC_ERR_NONE)                     \
                    {                                           \
                        board_print(2, "error @ line %d: %d\n", \
                            __LINE__, err);                     \
                        return err;                             \
                    }}

/* Functions */

/*!
 * @name Initialization Functions
 * @{
 */

/*!
 * This function initalizes the board.
 *
 * @param[in]     phase       boot phase
 *
 * There are four phases to board intialization.  The fist phase is
 * the API phase (\a phase = 0) and initializes all of the board
 * interface data structures. The second phase (\a phase = 1) is the
 * HW phase and this initializes the board hardware. The third phase
 * (\a phase = 2) is the final boot phase and is used to wrap up any
 * needed init. A test phase (\a phase = 3) is called only when an SCFW
 * image is built with unit tests and is called just before any tests
 * are run. All are called from main_init() only.
 */
void board_init(uint8_t phase);

/*!
 * This function returns the debug UART info.
 *
 * @param[in]     inst        UART instance
 * @param[in]     baud        UART baud rate
 *
 * @return Pointer to the debug UART type.
 */
LPUART_Type *board_get_debug_uart(uint8_t *inst, uint32_t *baud);

/*!
 * This function initalizes the debug UART.
 *
 * @param[in]     early_phase flag indicating phase
 */
void board_config_debug_uart(sc_bool_t early_phase);

/*!
 * This function configures SCU resources.
 *
 * @param[in]     pt_sc       SCU partition
 *
 * By default, the SCFW keeps most of the resources found in the SCU
 * subsystem. It also keeps the SCU/PMIC pads required for the main
 * code to function. Any additional resources or pads required for
 * the board code to run should be kept here. This is done by marking
 * them as not movable.
 */
void board_config_sc(sc_rm_pt_t pt_sc);

/*!
 * This function returns board configuration info.
 *
 * @param[in]     parm        parameter to return
 *
 * This function is used to return board configuration info. Parameters
 * define if various how various SoC connections are made at the
 * board-level. For example, the external PCIe clock input.
 *
 * See example code (board.c) for parameter/returns options.
 *
 * @return Returns the paramter value.
 */
board_parm_rtn_t board_parameter(board_parm_t parm);

/*!
 * This function returns resource availability info.
 *
 * @param[in]     rsrc        resource to check
 *
 * This function is used to return board configuration info. It reports
 * if resources are functional on this board. For example, which DDR
 * controllers are used.
 *
 * See example code (board.c) for more details.
 *
 * @return Returns SC_TRUE if available.
 */
sc_bool_t board_rsrc_avail(sc_rsrc_t rsrc);

/*!
 * This function initalizes DDR.
 *
 * @param[in]     early             phase of init
 * @param[in]     ddr_initialized   True if ROM initialized the DDR
 *
 * This function may be called twice. The early parameter is SC_TRUE when
 * called prior to M4 start and SC_FALSE when called after. This
 * allows the implementation to decide when DDR init needs to be
 * done.
 *
 * Note the first call will not occur unless SC_BD_FLAGS_EARLY_CPU_START
 * is set in bd_flags of the boot container.
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t board_init_ddr(sc_bool_t early, sc_bool_t ddr_initialized);

/*!
 * This function cinfigures the DDR.
 *
 * Because this may be called by the ROM before the SCFW init is run,
 * this code cannot make any calls to any other SCFW APIs unless properly
 * conditioned with rom_caller.
 *
 * @param[in]     rom_caller  is ROM the caller?
 * @param[in]     action      perform this action on DDR
 *
 * @return Returns SC_ERR_NONE if successful.
 */
sc_err_t board_ddr_config(bool rom_caller, board_ddr_action_t action);

/*!
 * This function allows the board file to do SCFW configuration.
 *
 * @param[in]     early       phase of init
 * @param[in]     pt_boot     boot partition
 *
 * This function may be called twice. The early parameter is SC_TRUE when
 * called prior to M4 start and SC_FALSE when called after. This allows
 * the implementation to decide when to do configuration processing.
 *
 * Note the first call will not occur unless SC_BD_FLAGS_EARLY_CPU_START
 * is set in bd_flags of the boot container.
 *
 * Typical actions here include creating a resource partition for an M4,
 * powering up a board component, or configuring a shared clock.
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t board_system_config(sc_bool_t early, sc_rm_pt_t pt_boot);

/*!
 * This function returns SC_TRUE for early CPUs.
 *
 * @param[in]     cpu         CPU
 *
 * This function should return SC_TRUE if the CPU in question may
 * be started early. This early start is before power on of later
 * CPU subsystems. It would normally return SC_TRUE for CM4 cores that
 * need to run early. Only SC_R_M4_0_PID0 and SC_R_M4_1_PID0 can
 * return SC_TRUE.
 *
 * Note CPUs will only get started early if SC_BD_FLAGS_EARLY_CPU_START
 * is set in bd_flags of the boot container.
 *
 * @return Returns SC_TRUE if CPU should start early.
 */
sc_bool_t board_early_cpu(sc_rsrc_t cpu);

/* @} */

/*!
 * @name Power Functions
 * @{
 */

/*!
 * This function transitions the power state for an external board-
 * level supply which goes to the i.MX8.
 *
 * @param[in]     ss          subsystem using supply
 * @param[in]     pd          power domain
 * @param[in]     from_mode   power mode transitioning from
 * @param[in]     to_mode     power mode transitioning to
 *
 * This function is used to transition a board power supply that is
 * used by the SoC. It allows mapping of subsystem power domains
 * to board supplies.
 *
 * Note that the base code will enable/disable isolators after
 * changing the state of internal power domains. External supplies
 * sometimes supply a domain connected via an isolator to a domain
 * passed here. In this case, this function needs to also control the
 * connected domain's supply. For example, when LVDS SS PD (PD1) is
 * powered toggled, the external supply for the LVDS PHY must be
 * toggled here. MIPI and CSI SS PD are similar.
 */
void board_set_power_mode(sc_sub_t ss, uint8_t pd,
    sc_pm_power_mode_t from_mode, sc_pm_power_mode_t to_mode);

/*!
 * This function sets the voltage for a PMIC controlled SS.
 *
 * @param[in]     ss          subsystem
 * @param[in]     new_volt    voltage value to be set
 * @param[in]     wait        if SC_TRUE, wait for the PMIC to change
 *                            the voltage.
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t board_set_voltage(sc_sub_t ss, uint32_t new_volt, sc_bool_t wait);

/*!
 * This function transitions the power state for an external board-
 * level supply which goes to a board component.
 *
 * @param[in]     idx         board-relative resource index
 * @param[in]     rsrc_idx    unified resource index
 * @param[in]     from_mode   power mode transitioning from
 * @param[in]     to_mode     power mode transitioning to
 *
 * This function is used to transition a board power supply that is
 * used by a board component. It allows mapping of board resources
 * (e.g. SC_R_BOARD_R0) to board supplies.
 *
 * \a idx should be used to identify the resource. It is 0-n and is
 * associated with the board reosurces PMIC_0 through BOARD_R7.
 *
 * \a rsrc_idx is only useful for debug output of a resource name.
 * 
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t board_trans_resource_power(sc_rm_idx_t idx, sc_rm_idx_t rsrc_idx,
    sc_pm_power_mode_t from_mode,sc_pm_power_mode_t to_mode);

/* @} */

/*!
 * @name Misc Functions
 * @{
 */

/*!
 * This function is used to set the board power.
 *
 * @param[in]     mode        power mode to apply
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid mode
 */
sc_err_t board_power(sc_pm_power_mode_t mode);

/*!
 * This function is used to reset the system.
 *
 * @param[in]     type        reset type
 * @param[in]     reason      cause of reset
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid type
 *
 * If this function returns, then the reset did not occur due to an
 * invalid parameter.
 */
sc_err_t board_reset(sc_pm_reset_type_t type, sc_pm_reset_reason_t reason);

/*!
 * This function is called when a CPU encounters a reset
 * event.   
 *
 * @param[in]     resource    CPU resource
 * @param[in]     reset_event CPU reset event
 */
void board_cpu_reset(sc_rsrc_t resource, board_cpu_rst_ev_t reset_event);

/*!
 * This function is called when a DSC reports a panic temp alarm.
 *
 * @param[in]     dsc         dsc reporting alarm
 *
 * Note this function would normally request a board reset.
 */
void board_panic(sc_dsc_t dsc);

/*!
 * This function is called when a fault is detected or the SCFW
 * returns from main().
 *
 * @param[in]     restarted   SC_TRUE if called on restart
 *
 * Note this function would normally request a board reset. For
 * debug builds it is common to disable the watchdog and loop.
 *
 * The \a restarted paramter is SC_TRUE if this error is pending from
 * the last restart.
 */
void board_fault(sc_bool_t restarted);

/*!
 * This function is called when a security violation is reported by
 * the SECO or SNVS.
 *
 * Note this function would normally request a board reset. For
 * debug builds it is common to do nothing.
 */
void board_security_violation(void);

/*!
 * This function is used to return the current status of the ON/OFF
 * button.
 *
 * @return Returns the status
 */
sc_bool_t board_get_button_status(void);

/*!
 * This function sets a miscellaneous control value.
 *
 * @param[in]     resource    resource
 * @param[in]     idx         board resource index
 * @param[in]     rsrc_idx    unified resource index
 * @param[in]     ctrl        control to write
 * @param[in]     val         value to write
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Note this function can be used to set voltages for both SoC
 * resources and board resources (e.g. SC_R_BOARD_R0).
 */
sc_err_t board_set_control(sc_rsrc_t resource, sc_rm_idx_t idx,
    sc_rm_idx_t rsrc_idx, uint32_t ctrl, uint32_t val);

/*!
 * This function gets a miscellaneous control value.
 *
 * @param[in]     resource    resource
 * @param[in]     idx         board resource index
 * @param[in]     rsrc_idx    unified resource index
 * @param[in]     ctrl        control to read
 * @param[out]    val         pointer to return value
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t board_get_control(sc_rsrc_t resource, sc_rm_idx_t idx,
    sc_rm_idx_t rsrc_idx, uint32_t ctrl, uint32_t *val);

/*!
 * Interrupt handler for the PMIC.
 */
void PMIC_IRQHandler(void);

/*!
 * Interrupt handler for the SNVS button.
 */
void SNVS_Button_IRQHandler(void);

/* @} */

/* Externs */

/*!
 * External variable for accessing the number of board resources.
 */
extern const sc_rm_idx_t board_num_rsrc;

/*!
 * External variable for accessing the board resource map.
 */
extern const sc_rsrc_map_t board_rsrc_map[BRD_NUM_RSRC_BRD];

/*!
 * External variable for specing DDR periodic training.
 */
extern const uint32_t board_ddr_period_ms;

#endif /* SC_BOARD_API_H */

/**@}*/

