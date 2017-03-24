/**
 * \file common.c
 * \brief Contains Mykonos API common wrapper functions for user hardware platform drivers
 */

#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include "common.h" 
#include "spi.h"
#include "fpga_axi.h"
#include "timer.h"
#include "logging.h"

static uint8_t _writeBitPolarity = 1;
static uint8_t _longInstructionWord = 1;
static uint8_t _chipSelectIndex = 0;

ADI_LOGLEVEL CMB_LOGLEVEL = ADIHAL_LOG_NONE;

commonErr_t CMB_closeHardware(void)
{
    HAL_closeSpi();
    HAL_closeLogFile();

    return(COMMONERR_OK);
}

commonErr_t CMB_setGPIO(uint32_t GPIO)
{
    return(COMMONERR_OK);
}

commonErr_t CMB_hardReset(uint8_t spiChipSelectIndex)
{
    uint32_t readData = 0;
    uint32_t devResetBit = 0;
    uint32_t error = 0;

    if ((spiChipSelectIndex >= 1) && (spiChipSelectIndex <= 3))
    {
        switch(spiChipSelectIndex)
        {
            case 1:
                devResetBit = 0x10000;
                break;
            case 2:
                devResetBit = 0x20000;
                break;
            case 3:
                devResetBit = 0x40000;
                break;
            default:
                devResetBit = 0;
                break;
        }

        HAL_writeToLogFile("ResetDut at index %d", spiChipSelectIndex);

        /* toggle FPGA reg bit that goes out to FMC reset pin. FPGA reg
         * is active high, but signal to pins is active low.
         */
        error = CMB_regRead(0x14, &readData);
        error |= CMB_regWrite(0x14, (readData | devResetBit));
        error |= CMB_wait_ms(1);
        error |= CMB_regWrite(0x14, (readData & ~devResetBit));

        if(error)
        {
            return(COMMONERR_FAILED);
        }
    }
    else
    {
        HAL_writeToLogFile("ERROR: ResetDut at index %d is out of range", spiChipSelectIndex);
    }
    return(COMMONERR_OK);
}

commonErr_t CMB_setSPIOptions(spiSettings_t *spiSettings)
{
    int32_t error = 0;
    _writeBitPolarity = spiSettings->writeBitPolarity & 0x01;
    _longInstructionWord = spiSettings->longInstructionWord & 0x01;

    error = HAL_initSpi((spiSettings->chipSelectIndex), (uint8_t)((spiSettings->CPOL <<1) | (spiSettings->CPHA)), (spiSettings->spiClkFreq_Hz));

    if(error < 0)
    {
        return(COMMONERR_FAILED);
    }

    return(COMMONERR_OK);
}

commonErr_t CMB_setSPIChannel(uint16_t chipSelectIndex )
{
    /* 0 = all chip selects de-asserted */
    _chipSelectIndex = (uint8_t)chipSelectIndex;

    return(COMMONERR_OK);
}

commonErr_t CMB_SPIWriteByte(spiSettings_t *spiSettings, uint16_t addr, uint8_t data)
{
    int32_t retval = 0;
    unsigned char txbuf[] = {0x00,0x00,0x00};

    if (_chipSelectIndex != spiSettings->chipSelectIndex)
    {
        if(CMB_setSPIOptions(spiSettings))
        {
            return(COMMONERR_FAILED);
        }

        if(CMB_setSPIChannel(spiSettings->chipSelectIndex))
        {
            return(COMMONERR_FAILED);
        }
    }

    if(CMB_LOGLEVEL & ADIHAL_LOG_SPI)
    {
        HAL_writeToLogFile("SPIWrite: CS:%2d, ADDR:0x%03X, DATA:0x%02X \n", spiSettings->chipSelectIndex,addr, data);
    }

    if (spiSettings->longInstructionWord){
        txbuf[0] = ((_writeBitPolarity & 1) << 7) | ((addr >> 8) & 0x7F);
        txbuf[1] = addr & 0xFF;
        txbuf[2] = data;
        retval = HAL_spiWrite(txbuf, 3);

        if (retval < 0)
        {
            printf("Error writing SPI");
            return(COMMONERR_FAILED);
        }
    }
    else
    {
        txbuf[0] = ((_writeBitPolarity & 1) << 7) | (addr  & 0x7F);
        txbuf[1] = data;
        HAL_spiWrite(txbuf, 2);

        if (retval < 0)
        {
            printf("Error writing SPI");
            return(COMMONERR_FAILED);
        }
    }

    return(COMMONERR_OK);
}

commonErr_t CMB_SPIWriteBytes(spiSettings_t *spiSettings, uint16_t *addr, uint8_t *data, uint32_t count)
{
    uint32_t i = 0;
    uint32_t txBufIndex = 0;
    int32_t retval = 0;
    unsigned char txbuf[SPIARRAYSIZE] = {0x00};
    uint32_t spiArrayTripSize = SPIARRAYTRIPSIZE;

    if (_chipSelectIndex != spiSettings->chipSelectIndex)
    {
        if(CMB_setSPIOptions(spiSettings))
        {
            return(COMMONERR_FAILED);
        }

        if(CMB_setSPIChannel(spiSettings->chipSelectIndex))
        {
            return(COMMONERR_FAILED);
        }
    }

    if (spiSettings->longInstructionWord)
    {
        if (spiSettings->enSpiStreaming)
        {

        }
        else
        {
            txBufIndex = 0;

            for (i = 0; i < count; i++)
            {
                txbuf[txBufIndex++] = ((_writeBitPolarity & 1) << 7) | ((addr[i] >> 8) & 0x7F);
                txbuf[txBufIndex++] = (addr[i] & 0xFF);
                txbuf[txBufIndex++] = data[i];

                if(CMB_LOGLEVEL & ADIHAL_LOG_SPI)
                {
                    HAL_writeToLogFile("SPIWrite: CS:%2d, ADDR:0x%03X, DATA:0x%02X \n", spiSettings->chipSelectIndex, addr[i], data[i]);
                }

                /* Send full buffer when possible */
                if (txBufIndex >= spiArrayTripSize)
                {
                    /* Send full buffer when possible */
                    retval = HAL_spiWrite(txbuf, txBufIndex);
                    txBufIndex = 0;
                }
            }

            /* Send any data that was not sent as a full buffer before */
            if (txBufIndex > 0)
            {
                retval = HAL_spiWrite(txbuf, txBufIndex);
                txBufIndex = 0;
            }

        }

        if (retval < 0)
        {
            printf("Error writing SPI");
            return(COMMONERR_FAILED);
        }
    }
    else
    {
        /* 8bit instruction word */
        if (spiSettings->enSpiStreaming)
        {

        }
        else
        {

            txBufIndex = 0;
            spiArrayTripSize = ((SPIARRAYSIZE / 2) * 2);

            for (i = 0; i < count; i++)
            {
                txbuf[txBufIndex++] = ((_writeBitPolarity & 1) << 7) | (addr[i]  & 0x7F);
                txbuf[txBufIndex++] = data[i];

                if(CMB_LOGLEVEL & ADIHAL_LOG_SPI)
                {
                    HAL_writeToLogFile("SPIWrite: CS:%2d, ADDR:0x%03X, DATA:0x%02X \n", spiSettings->chipSelectIndex, addr[i], data[i]);
                }

                if (txBufIndex >= spiArrayTripSize)
                {
                    /* Send full buffer when possible */
                    retval = HAL_spiWrite(txbuf, txBufIndex);
                    txBufIndex = 0;
                }
            }

            /* Send any data that was not sent as a full buffer before */
            if (txBufIndex > 0)
            {
                retval = HAL_spiWrite(txbuf, txBufIndex);
                txBufIndex = 0;
            }

            if (retval < 0)
            {
                printf("Error writing SPI");
                return(COMMONERR_FAILED);
            }
        }
    }

    return(COMMONERR_OK);
}

commonErr_t CMB_SPIReadByte(spiSettings_t *spiSettings, uint16_t addr, uint8_t *readdata)
{
    uint8_t data=0;
    int32_t retval = 0;
    unsigned char txbuf[] = {0x00,0x00,0x00};

    if(_chipSelectIndex != spiSettings->chipSelectIndex)
    {
        if(CMB_setSPIOptions(spiSettings))
        {
            return(COMMONERR_FAILED);
        }
        if(CMB_setSPIChannel(spiSettings->chipSelectIndex))
        {
            return(COMMONERR_FAILED);
        }
    }
    if(spiSettings->longInstructionWord)
    {
        txbuf[0] = ((~_writeBitPolarity & 1) << 7) | ((addr >> 8) & 0x7F);
        txbuf[1] = addr & 0xFF;
        retval = HAL_spiRead(txbuf, 2, &data);
        if (retval < 0)
        {
            printf("Error writing SPI");
            return(COMMONERR_FAILED);
        }
        else
        {
            *readdata =(uint8_t)data;
        }
    }
    else
    {
        txbuf[0] = ((~_writeBitPolarity & 1) << 7) | (addr & 0x7F);
        retval = HAL_spiRead(txbuf, 1, &data);
        if (retval < 0)
        {
            printf("Error writing SPI");
            return(COMMONERR_FAILED);
        }
        else
        {
            *readdata =(uint8_t)data;
        }
    }
    if(CMB_LOGLEVEL & ADIHAL_LOG_SPI)
    {
        HAL_writeToLogFile("SPIRead: CS:%2d, ADDR:0x%03X, ReadData:0x%02X\n", spiSettings->chipSelectIndex, addr, *readdata);
    }

    return(COMMONERR_OK);
}

commonErr_t CMB_SPIWriteField(spiSettings_t *spiSettings, uint16_t addr, uint8_t field_val, uint8_t mask, uint8_t start_bit)
{
    uint8_t Val=0;

    if(CMB_LOGLEVEL & ADIHAL_LOG_SPI)
    {
        HAL_writeToLogFile("SPIWriteField: CS:%2d, ADDR:0x%03X, FIELDVAL:0x%02X, MASK:0x%02X, STARTBIT:%d \n", spiSettings->chipSelectIndex,addr, field_val, mask, start_bit);
    }
    if(CMB_SPIReadByte(spiSettings, addr, &Val))
    {
        return(COMMONERR_FAILED);
    }
    Val = (Val & ~mask) | ((field_val << start_bit) & mask);
    if(CMB_SPIWriteByte(spiSettings, addr, Val))
    {
        return(COMMONERR_FAILED);
    }

    return(COMMONERR_OK);
}

/* read a field in a single register space (not multibyte fields) */
commonErr_t CMB_SPIReadField(spiSettings_t *spiSettings, uint16_t addr, uint8_t *field_val, uint8_t mask, uint8_t start_bit)
{
    uint8_t data;

    if(CMB_SPIReadByte(spiSettings, addr, &data))
    {
        return(COMMONERR_FAILED);
    }
    *field_val =(uint8_t)((data & mask) >> start_bit);

    if(CMB_LOGLEVEL & ADIHAL_LOG_SPI)
    {
        HAL_writeToLogFile("SPIReadField: CS:%2d, ADDR:0x%03X, MASK:0x%02X, STARTBIT:%d, FieldVal:0x%02X\n", spiSettings->chipSelectIndex,addr, mask, start_bit, *field_val);
    }

    return(COMMONERR_OK);
}

commonErr_t CMB_writeToLog(ADI_LOGLEVEL level, uint8_t deviceIndex, uint32_t errorCode, const char *comment){

    if((CMB_LOGLEVEL & ADIHAL_LOG_ERROR) && (level == ADIHAL_LOG_ERROR))
    {
        HAL_writeToLogFile("ERROR: %d: %s", (int)errorCode, comment);
    }
    else if((CMB_LOGLEVEL & ADIHAL_LOG_WARNING) && (level == ADIHAL_LOG_WARNING))
    {
        HAL_writeToLogFile("WARNING: %d: %s",(int)errorCode, comment);
    }
    else if((CMB_LOGLEVEL & ADIHAL_LOG_MESSAGE) && (level == ADIHAL_LOG_MESSAGE))
    {
        HAL_writeToLogFile("MESSAGE: %d: %s",(int)errorCode, comment);
    }
    else if(CMB_LOGLEVEL == ADIHAL_LOG_NONE )
    {
        /*HAL_writeToLogFile("MESSAGE: %d: %s",(int)errorCode, comment);*/
    }
    else
    {
        HAL_writeToLogFile("Undefined Log Level: %0x%X", level);
    }

    return(COMMONERR_OK);
}

/* if filename null, a default path will be used in logging.c */
commonErr_t CMB_openLog(const char *filename)
{
    HAL_openLogFile(filename);

    return(COMMONERR_OK);
}

commonErr_t CMB_closeLog(void)
{
    HAL_closeLogFile();

    return(COMMONERR_OK);
}

commonErr_t CMB_flushLog(void)
{
    HAL_flushLogFile();

    return(COMMONERR_OK);
}

commonErr_t CMB_wait_ms(uint32_t time_ms)
{
    /* nanosleep can be interrupted, if so, call until total time has completed */
    struct timespec t0;
    struct timespec t1;
    struct timespec *temp;
    struct timespec *waitTime = &t0;
    struct timespec *remaining = &t1;

    waitTime->tv_sec = time_ms/1000;
    waitTime->tv_nsec = (time_ms % 1000) * (1000000);

    while((nanosleep(waitTime, remaining) == (-1)) && (errno == EINTR))
    {
        /* if nanosleep returned early due to a interrupt, nanosleep again using remaining time */
        temp = waitTime;
        waitTime = remaining;
        remaining = temp;
    }

    return(COMMONERR_OK);
}

commonErr_t CMB_wait_us(uint32_t time_us)
{
    return(CMB_wait_ms( time_us / 1000));
}

commonErr_t CMB_setTimeout_ms(uint32_t timeOut_ms)
{
    HAL_setTimeout_ms(timeOut_ms);

    return(COMMONERR_OK);
}

commonErr_t CMB_setTimeout_us(uint32_t timeOut_us)
{
    HAL_setTimeout_us(timeOut_us);

    return(COMMONERR_OK);
}

commonErr_t CMB_hasTimeoutExpired()
{
    uint8_t retval = 0;
    retval = HAL_hasTimeoutExpired();

    if (retval > 0)
    {
        return (COMMONERR_FAILED);
    }
    else 
    {
        return(COMMONERR_OK);
    }
}

commonErr_t CMB_regRead(uint32_t offset, uint32_t *data)
{
    char *uiod = "/dev/uio0";
    const uint32_t map_size = 0x1000;
    uint32_t error = 0;

    error = fpgaAxiReadWrite(uiod, map_size, 1, offset, 0, data);

    if(CMB_LOGLEVEL & ADIHAL_LOG_AXI_REG)
    {
        HAL_writeToLogFile("FPGA Register Read: OFFSET_ADDR:0x%08X, READ_DATA:0x%08X\n", offset, *data);
    }  
    if (error != 0)
    {
        return (COMMONERR_FAILED);
    }
    else 
    {
        return(COMMONERR_OK);
    }
    
}

commonErr_t CMB_regWrite(uint32_t offset, uint32_t data)
{
    char *uiod = "/dev/uio0";
    const uint32_t map_size = 0x1000;
    uint32_t error = 0;

    error = fpgaAxiReadWrite(uiod, map_size, 1, offset, 1, &data);

    if(CMB_LOGLEVEL & ADIHAL_LOG_AXI_REG)
    {
        HAL_writeToLogFile("FPGA Register Write: OFFSET_ADDR:0x%08X, WRITE_DATA:0x%08X\n", offset, data);
    }

    if (error != 0)
    {
        return (COMMONERR_FAILED);
    }
    else 
    {
        return(COMMONERR_OK);
    }
}

commonErr_t CMB_memRead(uint32_t offset, uint32_t *data, uint32_t len)
{
    char *uiod = "/dev/uio1";
    const uint32_t map_size = 0x40000000;
    uint32_t error = 0;

    error = fpgaAxiReadWrite(uiod, map_size, len, offset, 0, data);

    if(CMB_LOGLEVEL & ADIHAL_LOG_AXI_MEM)
    {
        HAL_writeToLogFile("Memory Read: OFFSET_ADDR:0x%08X, LENGTH:0x%08X\n", offset, len);
    }

    if (error != 0)
    {
        return (COMMONERR_FAILED);
    }
    else 
    {
        return(COMMONERR_OK);
    }
}

commonErr_t CMB_memWrite(uint32_t offset, uint32_t *data, uint32_t len)
{
    char *uiod = "/dev/uio1";
    const uint32_t map_size = 0x40000000;
    uint32_t error = 0;

    error = fpgaAxiReadWrite(uiod, map_size, len, offset, 1, data);

    if(CMB_LOGLEVEL & ADIHAL_LOG_AXI_MEM)
    {
        HAL_writeToLogFile("Memory Write: OFFSET_ADDR:0x%08X, LENGTH:0x%08X\n", offset, len);
    }

    if (error != 0)
    {
        return (COMMONERR_FAILED);
    }
    else 
    {
        return(COMMONERR_OK);
    }
}
