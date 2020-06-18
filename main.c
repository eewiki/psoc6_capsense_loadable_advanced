/******************************************************************************
* File Name: main.c
*
* Description: This code example features a 5-segment CapSense slider and two
*              CapSense buttons. Button 0 turns the LED ON, Button 1 turns the
*              LED OFF and the slider controls the brightness of the LED. The
*              code example also features interfacing with Tuner GUI using I2C
*              interface.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "led.h"
#include "cycfg_ble.h"
#include "cy_dfu.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_INTR_PRIORITY      (7u)
#define EZI2C_INTR_PRIORITY         (6u) /* EZI2C interrupt priority must be
                                          * higher than CapSense interrupt */

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static cy_status initialize_capsense(void);
static void process_touch(void);
static void initialize_capsense_tuner(void);
static void capsense_isr(void);
static void capsense_callback();
static void handle_ezi2c_tuner_event(void *callback_arg, cyhal_ezi2c_status_t event);
void handle_error(void);
void IasEventHandler(uint32 event, void *eventParam);
void AppCallBack(uint32_t event, void* eventParam);

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;
cyhal_ezi2c_t sEzI2C;
cyhal_ezi2c_slave_cfg_t sEzI2C_sub_cfg;
cyhal_ezi2c_cfg_t sEzI2C_cfg;
bool capsense_scan_complete = false;

volatile uint8_t alertLevel;
cy_stc_ble_conn_handle_t appConnHandle;

/* BLESS interrupt configure structure (for single CPU mode) */
static cy_stc_sysint_t blessIsrCfg =
{
	/* The BLESS interrupt */
	.intrSrc = (IRQn_Type)bless_interrupt_IRQn,

	/* The interrupt priority number */
	.intrPriority = 1u
};


/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}


/*******************************************************************************
* Function:     BlessInterrupt
* Author:       Cypress Semiconductor
* Description:    It is used used when BLE middleware operates in BLE single CM4
* Date:
*******************************************************************************/
void BlessInterrupt(void)
{
    /* Call interrupt processing */
    Cy_BLE_BlessIsrHandler();
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - initialize tuner communication
*  - scan touch input continuously and update the LED accordingly.
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_status status;
    cy_rslt_t result;

    /* Define the .cy_app_signature section */
    CY_SECTION(".cy_app_signature") __USED static const uint32_t cy_dfu_appSignature[1];

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    initialize_led();
    initialize_capsense_tuner();
    status = initialize_capsense();

    if (CYRET_SUCCESS != status)
    {
        /* Halt the CPU if CapSense initialization failed */
        CY_ASSERT(0);
    }

    /* Hook interrupt service routines for BLESS */
    (void) Cy_SysInt_Init(&blessIsrCfg, &BlessInterrupt);

    /* Store pointer to blessIsrCfg in BLE configuration structure */
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;

    /* Start BLE component and register generic event handler */
    /* Register the generic event handler */
    Cy_BLE_RegisterEventCallback(&AppCallBack);

    /* Initialize the BLE host */
    (void)Cy_BLE_Init(&cy_ble_config);

    /* Enable BLE Low Power Mode (LPM)*/
    Cy_BLE_EnableLowPowerMode();

    /* Enable BLE */
    (void)Cy_BLE_Enable();

    /* Register the IAS CallBack */
    Cy_BLE_IAS_RegisterAttrCallback(IasEventHandler);

    /* Initiate first scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for (;;)
    {
        if (capsense_scan_complete)
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Establishes synchronized operation between the CapSense
             * middleware and the CapSense Tuner tool.
             */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            capsense_scan_complete = false;
        }

        if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Initiate next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        }

        Cy_BLE_ProcessEvents();

        /* If alert level written */
        if (alertLevel != 0)
        {
            /* Switch to bootloader (app0) */
            Cy_DFU_ExecuteApp(0u);
        }
    }
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    uint32_t button0_status;
    uint32_t button1_status;
    cy_stc_capsense_touch_t *slider_touch_info;
    uint16_t slider_pos;
    uint8_t slider_touch_status;
    bool led_update_req = false;

    static uint32_t button0_status_prev;
    static uint32_t button1_status_prev;
    static uint16_t slider_pos_prev;
    static led_data_t led_data = {LED_ON, LED_MAX_BRIGHTNESS};

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON0_WDGT_ID,
        CY_CAPSENSE_BUTTON0_SNS0_ID,
        &cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON1_WDGT_ID,
        CY_CAPSENSE_BUTTON1_SNS0_ID,
        &cy_capsense_context);

    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;

    /* Detect new touch on Button0 */
    if ((0u != button0_status) &&
        (0u == button0_status_prev))
    {
        led_data.state = LED_ON;
        led_update_req = true;
    }

    /* Detect new touch on Button1 */
    if ((0u != button1_status) &&
        (0u == button1_status_prev))
    {
        led_data.state = LED_OFF;
        led_update_req = true;
    }

    /* Detect the new touch on slider */
    if ((0 != slider_touch_status) &&
        (slider_pos != slider_pos_prev))
    {
        led_data.brightness = (slider_pos * 100)
                / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;
        led_update_req = true;
    }

    /* Update the LED state if requested */
    if (led_update_req)
    {
        update_led_state(&led_data);
    }

    /* Update previous touch status */
    button0_status_prev = button0_status;
    button1_status_prev = button1_status;
    slider_pos_prev = slider_pos;
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
*******************************************************************************/
static cy_status initialize_capsense(void)
{
    cy_status status = CYRET_SUCCESS;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
        {
            .intrSrc = CYBSP_CSD_IRQ,
            .intrPriority = CAPSENSE_INTR_PRIORITY,
        };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
    NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
    NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Assign a callback function to indicate end of CapSense scan. */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
            capsense_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: capsense_callback()
********************************************************************************
* Summary:
*  This function sets a flag to indicate end of a CapSense scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t* : pointer to active sensor details.
*
*******************************************************************************/
void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    capsense_scan_complete = true;
}

/*******************************************************************************
* Function Name: handle_ezi2c_tuner_event
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from EZI2C block.
*
* Parameters:
*  callback_arg : extra argument that can be passed to callback
*  event        : EzI2C event
*
* Return:
*  void
*
*******************************************************************************/
static void handle_ezi2c_tuner_event(void *callback_arg, cyhal_ezi2c_status_t event)
{
    cyhal_ezi2c_status_t status;
    cy_stc_scb_ezi2c_context_t *context = &ezi2c_context;

    /* Get the slave interrupt sources */
    status = cyhal_ezi2c_get_activity_status(&sEzI2C);

    /* Handle the error conditions */
    if (0UL != (CYHAL_EZI2C_STATUS_ERR & status))
    {
        handle_error();
    }

    /* Handle the receive direction (master writes data) */
    if (0 != (CYHAL_EZI2C_STATUS_READ1 & status))
    {
        cyhal_i2c_slave_config_write_buffer((cyhal_i2c_t *)&sEzI2C, context->curBuf, context->bufSize);
    }
    /* Handle the transmit direction (master reads data) */
    if (0 != (CYHAL_EZI2C_STATUS_WRITE1 & status))
    {
        cyhal_i2c_slave_config_read_buffer((cyhal_i2c_t *)&sEzI2C, context->curBuf, context->bufSize);
    }
}

/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
*  Initializes interface between Tuner GUI and PSoC 6 MCU.
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_rslt_t result;

    /* Configure Capsense Tuner as EzI2C Slave */
    sEzI2C_sub_cfg.buf = (uint8 *)&cy_capsense_tuner;
    sEzI2C_sub_cfg.buf_rw_boundary = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.buf_size = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.slave_address = 8U;

    sEzI2C_cfg.data_rate = CYHAL_EZI2C_DATA_RATE_400KHZ;
    sEzI2C_cfg.enable_wake_from_sleep = false;
    sEzI2C_cfg.slave1_cfg = sEzI2C_sub_cfg;
    sEzI2C_cfg.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
    sEzI2C_cfg.two_addresses = false;
    
    result = cyhal_ezi2c_init(&sEzI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL, &sEzI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    cyhal_ezi2c_register_callback(&sEzI2C, handle_ezi2c_tuner_event, NULL);
    cyhal_ezi2c_enable_event(&sEzI2C,
                             (CYHAL_EZI2C_STATUS_ERR | CYHAL_EZI2C_STATUS_WRITE1 | CYHAL_EZI2C_STATUS_READ1),
                             EZI2C_INTR_PRIORITY, true);
}


/*******************************************************************************
* Function:     IasEventHandler
* Author:       Cypress Semiconductor
* Description:    This is an event callback function to receive events from the
*               BLE Component, which are specific to Immediate Alert Service.
* Date:         03-23-20
* Parameters:
*   event:       Write Command event from the BLE component.
*   eventParams: A structure instance of CY_BLE_GATT_HANDLE_VALUE_PAIR_T type
*******************************************************************************/
void IasEventHandler(uint32 event, void *eventParam)
{
    (void) eventParam;
    uint8_t alert;

    /* Alert Level Characteristic write event */
    if(event == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Read the updated Alert Level value from the GATT database */
        Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL, sizeof(alert), &alert);
        alertLevel = alert;
    }
}


/*******************************************************************************
* Function:     AppCallBack
* Author:       Cypress Semiconductor (modified by Matt Mielke)
* Description:    This is an event callback function to receive events from the
*               BLE Component. Used in Cy_DFU_TransportStart()
* Date:         03-23-20
*******************************************************************************/
void AppCallBack(uint32_t event, void* eventParam)
{
    static cy_stc_ble_gap_sec_key_info_t keyInfo =
    {
        .localKeysFlag    = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST |
                            CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST |
                            CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST,
        .exchangeKeysFlag = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST |
                            CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST |
                            CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_ENC_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_IRK_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_CSRK_KEY_DIST,
    };

    switch(event)
    {
        /**********************************************************************
         * General events
         *********************************************************************/

        /* This event is received when the BLE stack is started */
        case CY_BLE_EVT_STACK_ON:
        {
            /* Enter into discoverable mode so that remote can search it. */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, 0u);

            Cy_BLE_GAP_GenerateKeys(&keyInfo);
            break;
        }

        /**********************************************************************
         * GAP events
         *********************************************************************/

        case CY_BLE_EVT_GAP_AUTH_REQ:
        {
            if (cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].security
                == (CY_BLE_GAP_SEC_MODE_1 | CY_BLE_GAP_SEC_LEVEL_1))
            {
               cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].authErr =
                   CY_BLE_GAP_AUTH_ERROR_PAIRING_NOT_SUPPORTED;
            }

            cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].bdHandle =
               ((cy_stc_ble_gap_auth_info_t *)eventParam)->bdHandle;

            Cy_BLE_GAPP_AuthReqReply(&cy_ble_configPtr->authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX]);
            break;
        }

        /* This event is triggered instead of 'CY_BLE_EVT_GAP_DEVICE_CONNECTED',
        * if Link Layer Privacy is enabled in component customizer
        */
        case CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE:
        {
            /* sets the security keys that are to be exchanged with a peer
             * device during key exchange stage of the authentication procedure
             */
            keyInfo.SecKeyParam.bdHandle =
                (*(cy_stc_ble_gap_enhance_conn_complete_param_t *)eventParam).bdHandle;

            Cy_BLE_GAP_SetSecurityKeys(&keyInfo);
            break;
        }

        /* This event indicates security key generation complete */
        case CY_BLE_EVT_GAP_KEYS_GEN_COMPLETE:
        {
            keyInfo.SecKeyParam = (*(cy_stc_ble_gap_sec_key_param_t *)eventParam);
            Cy_BLE_GAP_SetIdAddress(&cy_ble_deviceAddress);
            break;
        }

        /* This event is generated when disconnected from remote device or
         * failed to establish connection
         */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
            /* Enter into discoverable mode so that remote can search it. */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, 0u);

            break;
        }

        /**********************************************************************
         * GATT events
         *********************************************************************/

        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device
         */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            appConnHandle = *(cy_stc_ble_conn_handle_t *)eventParam;
            break;
        }

        default:
        {
            break;
        }
    }
}

/* [] END OF FILE */
