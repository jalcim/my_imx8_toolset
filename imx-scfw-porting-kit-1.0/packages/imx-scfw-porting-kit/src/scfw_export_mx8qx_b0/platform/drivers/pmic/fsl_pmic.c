/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

/* Includes */

#include "main/scfw.h"
#include "main/main.h"
#include "fsl_device_registers.h"
#include "drivers/pmic/fsl_pmic.h"
#include "drivers/lpi2c/fsl_lpi2c.h"
#include "fsl_common.h"

/* Local Defines */
#define POLY         0x1DU /* SAE J1850 Polynomial */

/* Local Functions */

uint8_t pf8x00_crc_data(uint8_t addr, uint8_t reg, uint8_t *data, 
    uint8_t msg_size);

/* Local Variables */

/*--------------------------------------------------------------------------*/
/* Write to the PMIC via the I2C                                            */
/*--------------------------------------------------------------------------*/
status_t i2c_write_sub(uint8_t device_addr, uint8_t reg, void *data,
                              uint32_t dataLength)
{ 
    status_t status = I32(kStatus_Success);

    do
    {
            status = LPI2C_MasterStart(LPI2C_PMIC, device_addr,
                kLPI2C_Write);
            if (status != I32(kStatus_Success))
            {
                break;
            }

            status = LPI2C_MasterSend(LPI2C_PMIC, &reg, 1);
            if (status != I32(kStatus_Success))
            {
                break;
            }

            status = LPI2C_MasterSend(LPI2C_PMIC, data, dataLength);
            if (status != I32(kStatus_Success))
            {
                break;
            }

            status = LPI2C_MasterStop(LPI2C_PMIC);
            /* if we get this far we have succeeded */
    } while (SC_FALSE);

    return status;
}

/*--------------------------------------------------------------------------*/
/* Write to the PMIC via the I2C (with clock management)                    */
/*--------------------------------------------------------------------------*/
status_t i2c_write(uint8_t device_addr, uint8_t reg, void *data,
    uint32_t dataLength)
{
#ifdef SC_MANAGE_LPI2C_CLK
    // Enable clocking using LPCG
    if (LPI2C_PMIC == LPI2C_SC)
    {
        CLOCK_EnableClockEx(kCLOCK_LPI2C_SC);
    }

    // Do the transfer
    status_t stat = i2c_write_sub(device_addr, reg, data, dataLength);

    if(stat != I32(kStatus_Success))/* do a MasterStop if we ever fail */
    {
        pmic_print(1, "I2C Error: %d \n", stat);
        (void) LPI2C_MasterStop(LPI2C_PMIC);
    }

    if (LPI2C_PMIC == LPI2C_SC)
    {
        // Enter exclusive attempt to disable clock
        uint32_t lpcgVal = CLOCK_DisableClockExEnter(kCLOCK_LPI2C_SC);

        // Check if conditions allow clock gating
        if (!LPI2C_MasterGetBusIdleState(LPI2C_PMIC))
        {
            // Mask to disable IPG and BAUD clocks
            uint32_t lpcgMask = ~(BIT(LPCG__SS_SCU__IPG_CLK__SWEN) |
                                 BIT(LPCG__SS_SCU__PER_CLK__SWEN));

            lpcgVal &= lpcgMask;
        }
        
        // Leave exclusive attempt to disable clock
        CLOCK_DisableClockExLeave(kCLOCK_LPI2C_SC, lpcgVal);
    }
    
    return stat;
#else
    return i2c_write_sub(device_addr, reg, data, dataLength);
#endif
}

/*--------------------------------------------------------------------------*/
/* Write to the PMIC via the I2C with CRC and clock management              */
/*--------------------------------------------------------------------------*/
status_t i2c_j1850_write(uint8_t device_addr, uint8_t reg, void *data,
                          uint8_t dataLength)
{   status_t status = I32(kStatus_Success);
    uint8_t crc;

#ifdef SC_MANAGE_LPI2C_CLK
    // Enable clocking using LPCG
    if (LPI2C_PMIC == LPI2C_SC)
    {
        CLOCK_EnableClockEx(kCLOCK_LPI2C_SC);
    }
#endif
    /* calculate the CRC for data */
    crc = pf8x00_crc_data((device_addr << 1), reg, data, dataLength);

    // Do the transfer
    do
    { /* use a do while loop to jump out if error */

        status = LPI2C_MasterStart(LPI2C_PMIC, device_addr, kLPI2C_Write);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterSend(LPI2C_PMIC, &reg, 1);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterSend(LPI2C_PMIC, data, dataLength);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterSend(LPI2C_PMIC, &crc, 1);
        if (status != I32(kStatus_Success))
        { 
            break;
        }
        
        status = LPI2C_MasterStop(LPI2C_PMIC);
        /* if we got here we succeeded */
    } while (SC_FALSE);

    if (status != I32(kStatus_Success))
    {
        pmic_print(1, "LPI2C error kStatus =  %x", status);
        (void)LPI2C_MasterStop(LPI2C_PMIC);
    }

#ifdef SC_MANAGE_LPI2C_CLK
    // Disable Clock after write
    if (LPI2C_PMIC == LPI2C_SC)
    {
        // Enter exclusive attempt to disable clock
        uint32_t lpcgVal = CLOCK_DisableClockExEnter(kCLOCK_LPI2C_SC);

        // Check if conditions allow clock gating
        if (!LPI2C_MasterGetBusIdleState(LPI2C_PMIC))
        {
            // Mask to disable IPG and BAUD clocks
            uint32_t lpcgMask = ~(BIT(LPCG__SS_SCU__IPG_CLK__SWEN) |
                                 BIT(LPCG__SS_SCU__PER_CLK__SWEN));

            lpcgVal &= lpcgMask;
        }

        // Leave exclusive attempt to disable clock
        CLOCK_DisableClockExLeave(kCLOCK_LPI2C_SC, lpcgVal);
    }
#endif

    return status;
}

/*--------------------------------------------------------------------------*/
/* Read from the PMIC via the I2C with CRC and clock management             */
/*--------------------------------------------------------------------------*/
status_t i2c_j1850_read(uint8_t device_addr, uint8_t reg, void *data,
                          uint8_t dataLength)
{   
    status_t status = I32(kStatus_Success);
    uint32_t flags;
    uint8_t crc;

#ifdef SC_MANAGE_LPI2C_CLK
    // Enable clocking using LPCG
    if (LPI2C_PMIC == LPI2C_SC)
    {
        CLOCK_EnableClockEx(kCLOCK_LPI2C_SC);
    }
#endif
    /* calculate the CRC for address + register */
    crc = pf8x00_crc_data(((device_addr << 1) | 0x1U), reg, data, 0);

    // Do the transfer
    do
    {/* use a true while loop to jump out if error */

        status = LPI2C_MasterStart(LPI2C_PMIC, device_addr, kLPI2C_Write);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterSend(LPI2C_PMIC, &reg, 1);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterSend(LPI2C_PMIC, &crc, 1);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterStart(LPI2C_PMIC, device_addr, kLPI2C_Read);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterReceive(LPI2C_PMIC, data, dataLength);
        if (status != I32(kStatus_Success)) { break;}

        while (SC_TRUE)
        {
            flags = LPI2C_MasterGetStatusFlags(LPI2C_PMIC);

            if ((flags & U32(kLPI2C_MasterNackDetectFlag)) == 0U)
            {
                break;
            }

            if ((flags & i2c_error_flags) != 0U) {
                pmic_print(1, "i2c error flag: %d \n", flags);
            }
        }

        status = LPI2C_MasterStop(LPI2C_PMIC);
        /* if we got here we succeeded */
    } while (SC_FALSE);

    if (status != I32(kStatus_Success))
    {
        pmic_print(1, "LPI2C error kStatus =  %x", status);
        (void)LPI2C_MasterStop(LPI2C_PMIC);
    }

#ifdef SC_MANAGE_LPI2C_CLK
    // Disable Clock after write
    if (LPI2C_PMIC == LPI2C_SC)
    {
        // Enter exclusive attempt to disable clock
        uint32_t lpcgVal = CLOCK_DisableClockExEnter(kCLOCK_LPI2C_SC);

        // Check if conditions allow clock gating
        if (!LPI2C_MasterGetBusIdleState(LPI2C_PMIC))
        {
            // Mask to disable IPG and BAUD clocks
            uint32_t lpcgMask = ~(BIT(LPCG__SS_SCU__IPG_CLK__SWEN) |
                                 BIT(LPCG__SS_SCU__PER_CLK__SWEN));

            lpcgVal &= lpcgMask;
        }

        // Leave exclusive attempt to disable clock
        CLOCK_DisableClockExLeave(kCLOCK_LPI2C_SC, lpcgVal);
    }
#endif
    return status;
}

/*--------------------------------------------------------------------------*/
/* Read from the PMIC via I2C                                               */
/*--------------------------------------------------------------------------*/
status_t i2c_read_sub(uint8_t device_addr, uint8_t reg, void *data,
                             uint32_t dataLength)
{
    status_t status = I32(kStatus_Success);
    uint32_t flags;

    do
    {
        status = LPI2C_MasterStart(LPI2C_PMIC, device_addr, kLPI2C_Write);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterSend(LPI2C_PMIC, &reg, 1);
        if (status != I32(kStatus_Success) )
        {
            break;
        }

        status = LPI2C_MasterStart(LPI2C_PMIC, device_addr, kLPI2C_Read);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        status = LPI2C_MasterReceive(LPI2C_PMIC, data, dataLength);
        if (status != I32(kStatus_Success))
        {
            break;
        }

        while (SC_TRUE)
        {
            flags = LPI2C_MasterGetStatusFlags(LPI2C_PMIC);

            if ((flags & U32(kLPI2C_MasterNackDetectFlag)) == 0U)
            {
                break;
            }

            if ((flags & i2c_error_flags) != 0U)
            {
                pmic_print(1, "i2c error flag: %d \n", flags);
            }
        }
        status =  LPI2C_MasterStop(LPI2C_PMIC);
        /* if we got here we had a succesful read */
    } while (SC_FALSE);

    return status;
}

/*--------------------------------------------------------------------------*/
/* Read from the PMIC via I2C (with clock management)                       */
/*--------------------------------------------------------------------------*/
status_t i2c_read(uint8_t device_addr, uint8_t reg, void *data,
                         uint32_t dataLength)
{
#ifdef SC_MANAGE_LPI2C_CLK
    // Enable clocking using LPCG
    if (LPI2C_PMIC == LPI2C_SC)
    {
        CLOCK_EnableClockEx(kCLOCK_LPI2C_SC);
    }

    // Do the transfer
    status_t stat = i2c_read_sub(device_addr, reg, data, dataLength);

    if(stat != I32(kStatus_Success))/* do a MasterStop if we ever fail */
    {
        pmic_print(1, "I2C error : %d \n", stat);
        (void) LPI2C_MasterStop(LPI2C_PMIC);
    }

    if (LPI2C_PMIC == LPI2C_SC)
    {
        // Enter exclusive attempt to disable clock
        uint32_t lpcgVal = CLOCK_DisableClockExEnter(kCLOCK_LPI2C_SC);

        // Check if conditions allow clock gating
        if (!LPI2C_MasterGetBusIdleState(LPI2C_PMIC))
        {
            // Mask to disable IPG and BAUD clocks
            uint32_t lpcgMask = ~(BIT(LPCG__SS_SCU__IPG_CLK__SWEN) |
                                 BIT(LPCG__SS_SCU__PER_CLK__SWEN));

            lpcgVal &= lpcgMask;
        }
        
        // Leave exclusive attempt to disable clock
        CLOCK_DisableClockExLeave(kCLOCK_LPI2C_SC, lpcgVal);
    }
    
    return stat;
#else
    return i2c_read_sub(device_addr, reg, data, dataLength);
#endif
}

/*--------------------------------------------------------------------------*/
/* Get PMIC device ID                                                       */
/*--------------------------------------------------------------------------*/
uint8_t pmic_get_device_id(uint8_t address)
{
    status_t status;
    uint32_t flags;
    uint8_t dev_id_read = 0U;
    uint8_t dev_reg = 0U;

    #ifdef SC_MANAGE_LPI2C_CLK
        // Enable clocking using LPCG
        if (LPI2C_PMIC == LPI2C_SC)
        {
            CLOCK_EnableClockEx(kCLOCK_LPI2C_SC);
        }
    #endif

    do
    {
        /* Send master blocking data to slave */
        status = LPI2C_MasterStart(LPI2C_PMIC, address, kLPI2C_Write);
        if (status != I32(kStatus_Success)) /* check if status valid */
        {
            break;
        }

        status = LPI2C_MasterSend(LPI2C_PMIC, &dev_reg, 1);/* send reg 0 address */
        if (status != I32(kStatus_Success)) /* check if status valid */
        {
            break;
        }

        status = LPI2C_MasterStart(LPI2C_PMIC, address, kLPI2C_Read);
        if (status != I32(kStatus_Success)) /* check if status valid */
        {
            break;
        }

        status = LPI2C_MasterReceive(LPI2C_PMIC, &dev_id_read, 1);
        if (status != I32(kStatus_Success)) /* check if status valid */
        {
            break;
        }

        while (SC_TRUE)
        {
            flags = LPI2C_MasterGetStatusFlags(LPI2C_PMIC);

            if ((flags & U32(kLPI2C_MasterNackDetectFlag)) == 0U)
            {
                break;
            }

            if ((flags & i2c_error_flags) != 0U)
            {
                pmic_print(1, "i2c error flag: %d \n", flags);
            }
        }

        status = LPI2C_MasterStop(LPI2C_PMIC);
        /* if we got here it was successful up until stop */
    } while (SC_FALSE);

    if ((status != I32(kStatus_Success)) && (status != I32(kStatus_LPI2C_Nak)))
    {/* reset if there was an error */
        /* LPI2C_MasterInit(LPI2C_PMIC, &lpi2c_masterConfig, SC_24MHZ);*/
        pmic_print(1, "I2C Error %d \n", status);
        (void)LPI2C_MasterStop(LPI2C_PMIC);
        dev_id_read = 0U;
    }

    #ifdef SC_MANAGE_LPI2C_CLK
        if (LPI2C_PMIC == LPI2C_SC)
        {
            // Enter exclusive attempt to disable clock
            uint32_t lpcgVal = CLOCK_DisableClockExEnter(kCLOCK_LPI2C_SC);

            // Check if conditions allow clock gating
            if (!LPI2C_MasterGetBusIdleState(LPI2C_PMIC))
            {
                // Mask to disable IPG and BAUD clocks
                uint32_t lpcgMask = ~(BIT(LPCG__SS_SCU__IPG_CLK__SWEN) |
                                     BIT(LPCG__SS_SCU__PER_CLK__SWEN));

                lpcgVal &= lpcgMask;
            }

            // Leave exclusive attempt to disable clock
            CLOCK_DisableClockExLeave(kCLOCK_LPI2C_SC, lpcgVal);
        }
    #endif

   return dev_id_read;
}

/*--------------------------------------------------------------------------*/
/* Helper Function for j1850 crc write or read                              */
/*--------------------------------------------------------------------------*/
uint8_t pf8x00_crc_data(uint8_t addr, uint8_t reg, uint8_t *data,
    uint8_t msg_size)
{
    uint8_t i, j, crc = 0xFFU;

    if (data == NULL)
    {
        return 0U;
    }

    /* crc the address first */
    crc ^= addr;
    for (i = 0U; i < 8U; i++)
    {
        crc = ((crc & 0x80U) != 0U) ? ((crc << 1U) ^ POLY ) : U8(crc << 1U);
    }

    /* crc the register address next */
    crc ^= reg;
    for (i = 0U; i < 8U; i++)
    {
        crc = ((crc & 0x80U) != 0U) ? ((crc << 1U) ^ POLY ) : U8(crc << 1U);
    }

    j = 0U; /* finally do the data */
    while (j < msg_size)
    {
        crc ^= *data++;
        for (i = 0U; i < 8U; i++)
        {
            crc = ((crc & 0x80U) != 0U) ? ((crc << 1U) ^ POLY ) : U8(crc << 1U);
        }
        j++;
    }

    return crc;
}

