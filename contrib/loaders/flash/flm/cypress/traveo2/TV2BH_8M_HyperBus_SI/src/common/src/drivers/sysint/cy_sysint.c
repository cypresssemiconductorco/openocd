/***************************************************************************//**
* \file
* \version 1.0
*
* \brief
* Provides an API implementation of the SysInt driver.
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_sysint.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*
  If the System IRQ user table is linked to RAM in the project/core specific
  cy_interrupt_map.c the following weak symbol is assigned the base address
  of the System IRQ user table in RAM, otherwise it stays NULL and APIs can 
  use this information to allow/prevent their service                         */
CY_WEAK cy_systemIntr_Handler * Cy_SysInt_SystemIrqUserTableRamPointer = NULL;


/*******************************************************************************
* Function Name: Cy_SysInt_Init
****************************************************************************//**
*
* \brief Initializes the referenced CPU interrupt by setting the priority and the
* interrupt vector. Note that the interrupt vector will only be relocated if the
* vector table was moved to __ramVectors in SRAM. Otherwise it is ignored.
*
* Use the CMSIS core function NVIC_EnableIRQ(config.intrSrc) to enable it.
* And also for IRQs, use the Cy_SysInt_InitIRQ function to map and enable
* system interupt to CPU interrupt.
*
* \param config
* Interrupt configuration structure
*
* \param userIsr
* Address of the ISR. If 0 <= config->intrSrc < Internal0_IRQn userIsr has no meaning.
*
* \return
* Initialization status
*
*******************************************************************************/
cy_en_sysint_status_t Cy_SysInt_Init(const cy_stc_sysint_t * config, cy_israddress userIsr)
{
    cy_en_sysint_status_t status = CY_SYSINT_SUCCESS;

    if(NULL != config)
    {
        /* Only set thte new vector if it is CPU interrupte (not CPU Intterrupt) */
        if ((config->intrSrc < 0) || (config->intrSrc >= Internal0_IRQn))
        {
            /* Only set the new vector if it was moved to __ramVectors */
            if (SCB->VTOR == (uint32_t)&__ramVectors)
            {
                (void)Cy_SysInt_SetVector(config->intrSrc, userIsr);
            }
        }
        else
        {
            return CY_SYSINT_BAD_PARAM; // Vector of system handler have to be set at ROM table.
        }
        NVIC_SetPriority(config->intrSrc, config->intrPriority);
    }
    else
    {
        status = CY_SYSINT_BAD_PARAM;
    }

    return(status);
}

/*******************************************************************************
* Function Name: Cy_SysInt_InitIRQ
****************************************************************************//**
*
* \brief Initializes the referenced system interrupt by setting the interrupt
* vector.
*
* \param config
* Interrupt configuration structure
*
* \param userIsr
* Address of the ISR
*
* \return
* Initialization status
*
*******************************************************************************/
cy_en_sysint_status_t Cy_SysInt_InitIRQ(const cy_stc_sysint_irq_t* config)
{
    cy_en_sysint_status_t status = CY_SYSINT_SUCCESS;

    #if (CY_CPU_CORTEX_M0P)
        un_CPUSS_CM0_SYSTEM_INT_CTL_t unIntCtl = { 0 };
    #else
        #if defined (tviibe1m) || defined (tviibe2m)
            un_CPUSS_CM4_SYSTEM_INT_CTL_t unIntCtl = { 0 };
        #elif defined (tviibh8m) || defined (tviic2d6m)
            un_CPUSS_CM7_0_SYSTEM_INT_CTL_t unIntCtl0 = { 0 };
            un_CPUSS_CM7_1_SYSTEM_INT_CTL_t unIntCtl1 = { 0 };
        #endif
    #endif
    if(NULL != config)
    {
        #if (CY_CPU_CORTEX_M0P) //rmkn u3CM0_CPU_INT_IDX->u3CPU_INT_IDX
            #if defined (tviibe1m) || defined (tviibe2m)
                unIntCtl.stcField.u3CPU_INT_IDX = (uint8_t)config->intIdx;
            #elif defined (tviibh8m) || defined (tviic2d6m)
                unIntCtl.stcField.u3CM0_CPU_INT_IDX = (uint8_t)config->intIdx;
            #endif
            unIntCtl.stcField.u1CPU_INT_VALID = config->isEnabled ? 1u : 0u;
            CPUSS->unCM0_SYSTEM_INT_CTL[config->sysIntSrc].u32Register = unIntCtl.u32Register;
        #else
            #if defined (tviibe1m) || defined (tviibe2m)
                unIntCtl.stcField.u3CPU_INT_IDX = (uint8_t)config->intIdx;
                unIntCtl.stcField.u1CPU_INT_VALID = config->isEnabled ? 1u : 0u;
                CPUSS->unCM4_SYSTEM_INT_CTL[config->sysIntSrc].u32Register = unIntCtl.u32Register;
            #elif defined (tviibh8m) || defined (tviic2d6m)
                if(CPUSS->unIDENTITY.stcField.u4MS == CPUSS_MS_ID_CM7_0)
                {
                    unIntCtl0.stcField.u4CPU_INT_IDX = (uint8_t)config->intIdx;
                    unIntCtl0.stcField.u1CPU_INT_VALID = config->isEnabled ? 1u : 0u;
                    CPUSS->unCM7_0_SYSTEM_INT_CTL[config->sysIntSrc].u32Register = unIntCtl0.u32Register;
                }
                else // should be CPUSS_MS_ID_CM7_1
                {
                    unIntCtl1.stcField.u4CPU_INT_IDX = (uint8_t)config->intIdx;
                    unIntCtl1.stcField.u1CPU_INT_VALID = config->isEnabled ? 1u : 0u;
                    CPUSS->unCM7_1_SYSTEM_INT_CTL[config->sysIntSrc].u32Register = unIntCtl1.u32Register;
                }
            #endif
        #endif
    }
    else
    {
        status = CY_SYSINT_BAD_PARAM;
    }

    return(status);
}

/*******************************************************************************
* Function Name: Cy_SysInt_SetVector
****************************************************************************//**
*
* \brief Changes the ISR vector for the Interrupt.
*
* Note that this function relies on the assumption that the vector table is
* relocated to __ramVectors[RAM_VECTORS_SIZE] in SRAM. Otherwise it will
* return the address of the default ISR location in Flash vector table.
*
* \param intrSrc
* Interrrupt source
*
* \param userIsr
* Address of the ISR to set in the interrupt vector table
*
* \return none
*
*******************************************************************************/
void  Cy_SysInt_SetVector(IRQn_Type intrSrc, cy_israddress userIsr)
{
    /* Only set the new vector if it was moved to __ramVectors */
    if (SCB->VTOR == (uint32_t)&__ramVectors)
    {
        __ramVectors[CY_INT_IRQ_BASE + intrSrc] = userIsr;
    }
}

/*******************************************************************************
* Function Name: Cy_SysInt_GetVector
****************************************************************************//**
*
* \brief Gets the address of the current ISR vector for the Interrupt.
*
* Note that this function relies on the assumption that the vector table is
* relocated to __ramVectors[RAM_VECTORS_SIZE] in SRAM.
*
* \param intrSrc
* Interrupt source
*
* \return
* Address of the ISR in the interrupt vector table
*
*******************************************************************************/
cy_israddress Cy_SysInt_GetVector(IRQn_Type intrSrc)
{
    cy_israddress currIsr;

    /* Only return the SRAM ISR address if it was moved to __ramVectors */
    if (SCB->VTOR == (uint32_t)&__ramVectors)
    {
        currIsr = __ramVectors[CY_INT_IRQ_BASE + intrSrc];
    }
    else
    {
        currIsr = __Vectors[CY_INT_IRQ_BASE + intrSrc];
    }

    return currIsr;
}


/*******************************************************************************
* Function Name: Cy_SysInt_SetSystemIrqVector
****************************************************************************//**
*
* \brief Changes the User ISR vector for the System Interrupt.
*
* \param sysIntSrc
* Interrrupt source
*
* \param userIsr
* Address of the ISR to set in the interrupt vector table
*
* \return none
*
*******************************************************************************/
void  Cy_SysInt_SetSystemIrqVector(cy_en_intr_t sysIntSrc, cy_systemIntr_Handler userIsr)
{
    if (Cy_SysInt_SystemIrqUserTableRamPointer != NULL)
    {
        Cy_SysInt_SystemIrqUserTableRamPointer[sysIntSrc] = userIsr;
    }
}


/*******************************************************************************
* Function Name: Cy_SysInt_GetSystemIrqVector
****************************************************************************//**
*
* \brief Gets the address of the current user ISR vector for the System Interrupt.
*
* \param sysIntSrc
* Interrupt source
*
* \return
* Address of the ISR in the interrupt vector table
*
*******************************************************************************/
cy_systemIntr_Handler Cy_SysInt_GetSystemIrqVector(cy_en_intr_t sysIntSrc)
{
    if (Cy_SysInt_SystemIrqUserTableRamPointer != NULL)
    {
        return Cy_SysInt_SystemIrqUserTableRamPointer[sysIntSrc];
    }
    else
    {
        return NULL;
    }
}


/*******************************************************************************
* Function Name: Cy_SysInt_EnableIRQ
****************************************************************************//**
*
* \brief Enable system inerurupt.
*
* \param sysintSrc
* System interrrupt source to be enabled.
*
* \return none
*
*******************************************************************************/
void Cy_SysInt_EnableIRQ(cy_en_intr_t sysIntSrc)
{
#if (CY_CPU_CORTEX_M0P)
    CPUSS->unCM0_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID = 1u;
#else
    #if defined (tviibe1m) || defined (tviibe2m)
        CPUSS->unCM4_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID = 1u;
    #elif defined (tviibh8m) || defined (tviic2d6m)
        if(CPUSS->unIDENTITY.stcField.u4MS == CPUSS_MS_ID_CM7_0)
        {
            CPUSS->unCM7_0_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID = 1u;
        }
        else // should be CPUSS_MS_ID_CM7_1
        {
            CPUSS->unCM7_1_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID = 1u;
        }
    #endif
#endif
}

/*******************************************************************************
* Function Name: Cy_SysInt_DisableIRQ
****************************************************************************//**
*
* \brief Disable system inerurupt.
*
* \param sysintSrc
* System interrrupt source to be disabled.
*
* \return none
*
*******************************************************************************/
void Cy_SysInt_DisableIRQ(cy_en_intr_t sysIntSrc)
{
#if (CY_CPU_CORTEX_M0P)
    CPUSS->unCM0_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID = 0u;
    /* Read after write to ensure disabling the interrupt after returning from this function. */
    (void)CPUSS->unCM0_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID;
#else
    #if defined (tviibe1m) || defined (tviibe2m)
        CPUSS->unCM4_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID = 0u;
        /* Read after write to ensure disabling the interrupt after returning from this function. */
        (void)CPUSS->unCM4_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID;
    #elif defined (tviibh8m) || defined (tviic2d6m)
        if(CPUSS->unIDENTITY.stcField.u4MS == CPUSS_MS_ID_CM7_0)
        {
            CPUSS->unCM7_0_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID = 0u;
            /* Read after write to ensure disabling the interrupt after returning from this function. */
            (void)CPUSS->unCM7_0_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID;
        }
        else // should be CPUSS_MS_ID_CM7_1
        {
            CPUSS->unCM7_1_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID = 0u;
            /* Read after write to ensure disabling the interrupt after returning from this function. */
            (void)CPUSS->unCM7_1_SYSTEM_INT_CTL[sysIntSrc].stcField.u1CPU_INT_VALID;
        }
    #endif
#endif
}

/*******************************************************************************
* Function Name: Cy_SysInt_GetIntStatus
****************************************************************************//**
*
* \brief This interface returns lowest activated system interrupt index for
*        corresponding CPU interrupt index.
*        Return value can be used to determine which system interrupt activated
*        the interrupt handler.
*        Typical usage:
*        In case of one interrupt handler handles 2 or more system interrupts.
*        In that case, user can determine which system interrupt should be handled
*        in the handler with the retrun value by this I/F.
*
* \param intrSrc
* Number of CPU interrupt index. Only CPUIntIdx0_IRQn to CPUIntIdx7_IRQn
* are valid.
*
* \return
* System interrupt index that activated the interrupt handler.
* See \ref cy_en_intr_t.
* If invalid IRQ number is passed, then return disconnected_IRQn.
*
*******************************************************************************/
cy_en_intr_t Cy_SysInt_GetIntStatus(IRQn_Type intrSrc)
{
    cy_en_intr_t ret;
    #if CY_CPU_CORTEX_M0P
            switch(intrSrc)
            {
                case CPUIntIdx0_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM0_INT0_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx1_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM0_INT1_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx2_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM0_INT2_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx3_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM0_INT3_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx4_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM0_INT4_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx5_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM0_INT5_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx6_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM0_INT6_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx7_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM0_INT7_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                default:
                    /* Invalid parameter */
                    ret = disconnected_IRQn;
                    break;
            }
    #else
        #if defined (tviibe1m) || defined (tviibe2m)
            switch(intrSrc)
            {
                case CPUIntIdx0_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM4_INT0_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx1_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM4_INT1_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx2_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM4_INT2_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx3_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM4_INT3_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx4_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM4_INT4_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx5_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM4_INT5_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx6_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM4_INT6_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                case CPUIntIdx7_IRQn:
                    ret = (cy_en_intr_t)CPUSS->unCM4_INT7_STATUS.stcField.u10SYSTEM_INT_IDX;
                    break;
                default:
                    /* Invalid parameter */
                    ret = disconnected_IRQn;
                    break;
            }
        #elif defined (tviibh8m) || defined (tviic2d6m)
            if(CPUSS->unIDENTITY.stcField.u4MS == CPUSS_MS_ID_CM7_0)
            {
                ret = (cy_en_intr_t)CPUSS->unCM7_0_INT_STATUS[intrSrc].stcField.u10SYSTEM_INT_IDX;
            }
            else // should be CPUSS_MS_ID_CM7_1
            {
                ret = (cy_en_intr_t)CPUSS->unCM7_1_INT_STATUS[intrSrc].stcField.u10SYSTEM_INT_IDX;
            }            
        #endif
    #endif

    return ret;
}


#if defined(__cplusplus)
}
#endif


/* [] END OF FILE */
