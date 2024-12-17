/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Class-B Safety test code
*              example for TCPWM block, for ModusToolbox.
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

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "SelfTest.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_TIMEOUT_MS (10u)      /* in milliseconds */

/* Available commands */
#define SELFTEST_CMD_TIMER ('1')
#define SELFTEST_CMD_PWM ('2')
#define SELFTEST_CMD_PWM_GATE_KILL ('3')

#if COMPONENT_CAT1C
    #define PWM_IN_PIN_PORT    NULL
    #define PWM_IN_PIN_NUM     0u
#endif

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_en_tcpwm_status_t  api_status;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_test(void);
void pwm_test(void);
void pwm_gate_kill(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It performs Class-B safety test for TCPMW block.
* SelfTest is performed for Timer/Counter, PWM and PWM gate Kill based on the
* user command.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint8_t cmd;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();


    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CYBSP_DEBUG_UART_CTS,CYBSP_DEBUG_UART_RTS,CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "CLass-B: TCPWM SAFETY TEST "
           "****************** \r\n\n");


    /* Display available commands */
    printf("Available commands \r\n");
    printf("1 : Run SelfTest for Timer/ Counter\r\n");
    printf("2 : Run SelfTest for PWM\r\n");
    printf("3 : Run SelfTest for PWM Gate Kill\r\n\n");

    for (;;)
    {
        result = cyhal_uart_getc(&cy_retarget_io_uart_obj, &cmd, UART_TIMEOUT_MS);
        if (result != CY_RSLT_ERR_CSP_UART_GETC_TIMEOUT)
        {
            if (SELFTEST_CMD_TIMER == cmd)
            {
                printf("\r\n[Command] : Run SelfTest for Timer/ Counter\r\n");
                timer_test();

            }
            else if (SELFTEST_CMD_PWM == cmd)
            {
                printf("\r\n[Command] : Run SelfTest for PWM\r\n");
                pwm_test();
            }
            else if (SELFTEST_CMD_PWM_GATE_KILL == cmd)
            {
                printf("\r\n[Command] : Run SelfTest for PWM Gate Kill\r\n");
                pwm_gate_kill();

            }
            else
            {
                printf("\r\nEnter a valid command\r\n");
            }
        }


    }
}


/*******************************************************************************
* Function Name: timer_test
********************************************************************************
* Summary:
* This function creates configures the block to timer/counter personality and
* also the input clock to the CPU clock. The test verifies, if the counter is 
* incrementing the count value, and the count value falls within the expected 
* thresholds. 
*
* Parameters:
*  none
*
* Return :
*  void
*
*******************************************************************************/
void timer_test(void)
{
    SelfTest_Timer_Counter_init(CYBSP_TIMER_COUNTER_TEST_HW,
            CYBSP_TIMER_COUNTER_TEST_NUM, &CYBSP_TIMER_COUNTER_TEST_config,
            ((IRQn_Type)CYBSP_TIMER_COUNTER_TEST_IRQ));


    /* Run Timer/Counter Self Test... */
    if (OK_STATUS != SelfTest_Counter_Timer())
    {
        /* Process error */
        printf("Error: Timer Counter\r\n");
    }
    else
    {
        printf("Success: Timer Counter\r\n");
    }
}

/*******************************************************************************
* Function Name: pwm_test
********************************************************************************
* Summary:
* The function configures a 32-bit PWM to run at 1/3 duty ON, 2/3 OFF duty cycle 
* with a 1 millisecond period, and then start the PWM. The CPU is then run in a 
* loop for 5 milliseconds and the output is polled continuously in the loop. 
* The instances of `0` (low) and `1` (high) are counted. The on/off ratio is
* then calculated and checked if it falls within the expected thresholds.
*
* Parameters:
*  none
*
* Return:
*  void
*******************************************************************************/
void pwm_test(void)
{
    if (SelfTest_PWM_init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config,
            (IRQn_Type)CYBSP_PWM_IRQ) != 0)
    {
        __disable_irq();
    }

    /* Run PWM Self Test... */
    if (OK_STATUS != SelfTest_PWM(PWM_IN_PIN_PORT, PWM_IN_PIN_NUM))
    {
        /* Process error */
        printf("Error: PWM FAIL\r\n");
        printf("Ensure PWM_IN_PIN is connected to PWM line -"
                " Refer Hardware setup section in Readme\r\n");
    }
    else
    {
        /* Process success */
        printf("Success: PWM\r\n");
    }
}

/*******************************************************************************
* Function Name: pwm_gate_kill
********************************************************************************
* Summary:
* The function configures Kill mode of the TCPWM block as `Stop on Kill`. For 
* PSoC™ 6 MCUs, the low-power comparator output is routed to the Kill signal of 
* TCPWM indicating overvoltage/overcurrent condition if the voltage on the
*  positive terminal is greater than the voltage on the negative terminal. 
* For XMC7000 MCUs, a Voltage range violation of SAR ADC is routed to Kill signal
* of TCPWM indicating over-voltage/over-current condition. If an overvoltage or 
* overcurrent condition occurs, then it will Kill the PWM output. 
* The TCPWM base and CntNum are passed to check whether the counter is stopped
* or not. If the counter is not incrementing/decrementing, the PWM output is 
* inactive that the PWM is killed.
*
* Parameters:
*  none
*
* Return:
*  void
*******************************************************************************/
void pwm_gate_kill(void)
{
    /* Configure the TCPWM for PWM operation. */
     api_status = Cy_TCPWM_PWM_Init(CYBSP_PWM_GATEKILL_HW,
             CYBSP_PWM_GATEKILL_NUM, &CYBSP_PWM_GATEKILL_config);
    if( api_status != CY_TCPWM_SUCCESS)
    {
        __disable_irq();
    }

#if COMPONENT_CAT1C
    uint32_t PWM_PERIOD = (((Cy_SysClk_ClkPeriGetFrequency() / 1000000)) * (PWM_TIME));

    Cy_TCPWM_PWM_SetPeriod0(CYBSP_PWM_GATEKILL_HW, CYBSP_PWM_GATEKILL_NUM, (PWM_PERIOD-1));
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_GATEKILL_HW, CYBSP_PWM_GATEKILL_NUM, (PWM_PERIOD));
#endif


    Cy_TCPWM_PWM_Enable(CYBSP_PWM_GATEKILL_HW, CYBSP_PWM_GATEKILL_NUM);
    Cy_TCPWM_TriggerReloadOrIndex_Single(CYBSP_PWM_GATEKILL_HW,
            CYBSP_PWM_GATEKILL_NUM);

#if COMPONENT_CAT1A
        /*Initialize the LPCOMP with device configurator generated structure*/
        Cy_LPComp_Init(LPCOMP_HW, LPCOMP_CHANNEL, &LPCOMP_config);


        /* Apply higher voltage to the LPCOMP +ve pin, will kill the PWM */
        Cy_LPComp_Enable(LPCOMP_HW, LPCOMP_CHANNEL);
        while (0 == Cy_LPComp_GetCompare(LPCOMP_HW, LPCOMP_CHANNEL))
        {}

#elif COMPONENT_CAT1C
        /* Initialize the SAR2 module */
        Cy_SAR2_Init(CYBSP_DUT_SAR_ADC_HW, &CYBSP_DUT_SAR_ADC_config);

        /* Set ePASS MMIO reference buffer mode for bangap voltage */
        Cy_SAR2_SetReferenceBufferMode(PASS0_EPASS_MMIO, CY_SAR2_REF_BUF_MODE_ON);

        /* Voltage range violation of SAR ADC is routed to Kill signal of TCPWM
         * indicating over-voltage/over-current condition */
        do
        {
            Cy_SAR2_Channel_SoftwareTrigger(CYBSP_DUT_SAR_ADC_HW, 0x00);
            do
            {
                Cy_SysLib_DelayUs(1u);
            } while (CY_SAR2_INT_GRP_DONE != (CY_SAR2_INT_GRP_DONE &
                    Cy_SAR2_Channel_GetInterruptStatus(CYBSP_DUT_SAR_ADC_HW, 0x00)));

            Cy_SAR2_Channel_ClearInterrupt(CYBSP_DUT_SAR_ADC_HW, 0x00,
                    CY_SAR2_INT_GRP_DONE);

        } while (CY_SAR2_INT_CH_RANGE != (CY_SAR2_INT_CH_RANGE &
                Cy_SAR2_Channel_GetInterruptStatus(CYBSP_DUT_SAR_ADC_HW, 0x00)));

        Cy_SAR2_Channel_ClearInterrupt(CYBSP_DUT_SAR_ADC_HW, 0x00,
                (CY_SAR2_INT_CH_RANGE | CY_SAR2_INT_GRP_DONE));

#endif

         /* Run PWM GateKill Self Test... */
          if (OK_STATUS != SelfTest_PWM_GateKill(CYBSP_PWM_GATEKILL_HW,
                  CYBSP_PWM_GATEKILL_NUM))
           {

                 /* Process error */
                 printf("Error: PWM GateKill FAIL\r\n");
                 printf("Ensure LPCOMP +ve pin is connected to VCC -"
                " Refer Hardware setup section in Readme\r\n");

           }
           else
           {
                 /* Process success */
                 printf("Success: PWM GateKill\r\n");
           }

}

/* [] END OF FILE */
