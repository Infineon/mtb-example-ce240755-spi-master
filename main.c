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
 * (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"

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

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    UART_context;           /** UART context */
static mtb_hal_uart_t               UART_hal_obj;           /** Debug UART HAL object  */

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

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&UART_hal_obj, &UART_hal_config, &UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&UART_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

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
