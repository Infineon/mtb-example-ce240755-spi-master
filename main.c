/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of SPI
* resource as Master. The SPI master sends command packetsto the SPI
* slave to control an user LED.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"

/******************************************************************************
* Macros
*******************************************************************************/
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY           (1000UL)
/* SPI transfer bits per frame */
#define BITS_PER_FRAME             (8)

#define _SPI_SSEL_ACTIVATE            false
#define _SPI_SSEL_DEACTIVATE          true

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_spi_context_t context;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* The main function.
*   1. Initializes the board, retarget-io and led
*   2. Configures the SPI Master to send command packet to the slave
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t cmd_send = CYBSP_LED_STATE_OFF;
    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize retarget-io to use the debug UART port */
     Cy_SCB_UART_Init(UART_HW, &UART_config, NULL);
     Cy_SCB_UART_Enable(UART_HW);
     cy_retarget_io_init(UART_HW);

    /* Initialize retarget-io for uart logs */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("*************** "
           "PDL: SPI Master "
           "*************** \r\n\n");

    printf("Configuring SPI master...\r\n");

    /* using Device-configurator instead of hal for initial SPI master */
    result = Cy_SCB_SPI_Init(SPI_M_HW,&SPI_M_config,&context);
    handle_error(result);

    /* Set the SPI baud rate */
    Cy_SCB_SPI_Enable(SPI_M_HW);/* Enable SPI */

    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
        uint32 count = 0;
    /* Toggle the slave LED state */
        cmd_send = (cmd_send == CYBSP_LED_STATE_OFF) ?
                     CYBSP_LED_STATE_ON : CYBSP_LED_STATE_OFF;

        /* Clear FIFOs */
        Cy_SCB_SPI_ClearTxFifo(SPI_M_HW);
        Cy_SCB_SPI_ClearRxFifo(SPI_M_HW);
        while (count == 0){
            count = Cy_SCB_SPI_Write(SPI_M_HW, cmd_send);
        }
        while (Cy_SCB_SPI_IsTxComplete(SPI_M_HW) == false) { }
        while (Cy_SCB_SPI_GetNumInRxFifo(SPI_M_HW) == 0) { } /* Wait for RX FIFO not empty */
        (void)Cy_SCB_SPI_Read(SPI_M_HW);
        handle_error(result);

        /* Give delay between commands */
        Cy_SysLib_Delay(CMD_TO_CMD_DELAY);
    }
}


/* [] END OF FILE */
