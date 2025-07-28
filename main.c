/*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.2
 *
 * @version Package Version: 3.1.2
 */

/*
� [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
 */
#include "mcc_generated_files/system/system.h"
#include "../libosdp-pic.X/libosdp/include/osdp.h"
#include "string.h"
/*
    Main application
 */

/*
 * User code
 */
uint32_t tickMs = 0;

void incrementTickMs() {
    tickMs++;
}

int64_t osdp_millis_now() {
    return tickMs;
}

enum osdp_pd_e {
    OSDP_PD_1,
    OSDP_PD_2,
    OSDP_PD_SENTINEL,
};

#if defined(PD_MODE)

int sample_pd_send_func(void *data, uint8_t *buf, int len) {
    (void) (data);
    (void) (buf);

    // TODO (user): send buf of len bytes, over the UART channel.
    IO_RC5_SetHigh();
    for (uint8_t i = 0; i < len; i++) {
        while (!UART1.IsTxReady()) {
        }
        UART1_Write(buf[i]);
    }
    while (!UART1.IsTxDone()) {
    }
    IO_RC5_SetLow();
    return len;
}

int sample_pd_recv_func(void *data, uint8_t *buf, int len) {
    (void) (data);
    (void) (buf);
    (void) (len);

    // TODO (user): read from UART channel into buf, for upto len bytes.
    uint8_t i = 0;
    while (UART1_IsRxReady() && i <= len)
        buf[i++] = UART1_Read();
    return i;
}

struct {
    uint8_t head;
    uint8_t tail;
    struct osdp_cmd_led rb[6];
} osdp_pd_led_rb = {
    .head = 0,
    .tail = 0,
    .rb =
    {0},
};

struct {
    int remainingTogle;
    uint32_t lastToggleTick;
    struct osdp_cmd_led led_setting;
} osdp_pd_led_state;

int pd_led_rb_push(struct osdp_cmd_led * led_cmd_data) {
    uint8_t tempHead = (osdp_pd_led_rb.head + 1) % 6;
    if (tempHead == osdp_pd_led_rb.tail) return -1; // full, do not override
    memcpy(&osdp_pd_led_rb.rb[tempHead], led_cmd_data, sizeof (struct osdp_cmd_led));
    osdp_pd_led_rb.head = tempHead;
    return tempHead;
}

int pd_led_rb_pop(struct osdp_cmd_led * buf) {
    uint8_t tempTail = osdp_pd_led_rb.tail;
    if (tempTail == osdp_pd_led_rb.head) return -1;
    memcpy(buf, &osdp_pd_led_rb.rb[tempTail], sizeof (struct osdp_cmd_led));
    osdp_pd_led_rb.tail = (tempTail + 1) % 6;
    return tempTail;
}

void pd_led_task() {
//    if (osdp_pd_led_state.remainingTogle != 0) {
//        uint8_t lastState = osdp_pd_led_state.remainingTogle % 2 == 0;
//        // do temporary state
//        if (lastState == 1 && tickMs - osdp_pd_led_state.lastToggleTick > osdp_pd_led_state.led_setting.permanent.on_count) {
//            // set it on
//        } else if (lastState == 0 && tickMs - osdp_pd_led_state.lastToggleTick > osdp_pd_led_state.led_setting.permanent.off_count) {
//            // set it off
//        }
//        osdp_pd_led_state.remainingTogle--;
//    } else {
//        // do permanent state
//        osdp_pd_led_state.
//    }
}

int pd_command_handler(void *arg, struct osdp_cmd *cmd) {
    (void) (arg);
    //    printf("PD: CMD: %d\n", cmd->id);
    switch (cmd->id) {
        case OSDP_CMD_LED:
            osdp_pd_led_state.lastToggleTick = tickMs;
            osdp_pd_led_state.remainingTogle = cmd->led.temporary.timer_count / (cmd->led.temporary.off_count + cmd->led.temporary.on_count);
            //            pd_led_rb_pop(&(cmd->led));
            break;
        default:
            break;
    }
    return 0;
}
#endif

#if defined(CP_MODE)

int sample_cp_send_func(void *data, uint8_t *buf, int len) {
    (void) (data);
    (void) (buf);

    // TODO (user): send buf of len bytes, over the UART channel.
    IO_RC5_SetHigh();
    for (uint8_t i = 0; i < len; i++) {
        while (!UART1.IsTxReady()) {
        }
        UART1_Write(buf[i]);
    }
    while (!UART1.IsTxDone()) {
    }
    IO_RC5_SetLow();
    return len;
}

int sample_cp_recv_func(void *data, uint8_t *buf, int len) {
    (void) (data);
    (void) (buf);
    (void) (len);

    // TODO (user): read from UART channel into buf, for upto len bytes.
    // TODO (user): read from UART channel into buf, for upto len bytes.
    uint8_t i = 0;
    while (UART1_IsRxReady() && i <= len)
        buf[i++] = UART1_Read();
    return i;
}
#endif

/*
 * User code
 */

int main(void) {
    SYSTEM_Initialize();
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts 
    // Use the following macros to: 

    // Enable the Global Interrupts 
    INTERRUPT_GlobalInterruptEnable();
    // Disable the Global Interrupts
    // INTERRUPT_GlobalInterruptDisable(); 

    TMR0_PeriodMatchCallbackRegister(incrementTickMs);

#if defined(PD_MODE)
    osdp_pd_info_t info_pd = {
        .address = 0,
        .baud_rate = 9600,
        .flags = 0,
        .channel.send = sample_pd_send_func,
        .channel.recv = sample_pd_recv_func,
        .id =
        {
            .version = 1,
            .model = 153,
            .vendor_code = 31337,
            .serial_number = 0x01020304,
            .firmware_version = 0x0A0B0C0D,
        },
        .cap = (struct osdp_pd_cap [])
        {
            {
                .function_code = OSDP_PD_CAP_READER_LED_CONTROL,
                .compliance_level = 1,
                .num_items = 1
            },
            {
                .function_code = OSDP_PD_CAP_READER_AUDIBLE_OUTPUT,
                .compliance_level = 1,
                .num_items = 1
            },
            { (uint8_t) - 1, 0, 0} /* Sentinel */
        },
        .scbk = NULL,
    };

    osdp_t *pd_ctx;
    pd_ctx = osdp_pd_setup(&info_pd);
    if (pd_ctx == NULL) {
        return -1;
    }
    osdp_pd_set_command_callback(pd_ctx, pd_command_handler, NULL);
#endif

#if defined(CP_MODE)
    osdp_pd_info_t pd_info[] = {
        {
            .address = 0,
            .baud_rate = 9600,
            .flags = 0,
            .channel.send = sample_cp_send_func,
            .channel.recv = sample_cp_recv_func,
            .scbk = NULL,
        },
        {
            .address = 1,
            .baud_rate = 9600,
            .flags = 0,
            .channel.send = sample_cp_send_func,
            .channel.recv = sample_cp_recv_func,
            .scbk = NULL,
        }
    };

    osdp_t *cp_ctx;
    cp_ctx = osdp_cp_setup(2, pd_info);
    if (cp_ctx == NULL) {
        return -1;
    }
#endif

    while (1) {
#if defined(PD_MODE)
        osdp_pd_refresh(pd_ctx);
        pd_led_task();
#endif
#if defined(CP_MODE)
        osdp_cp_refresh(cp_ctx);
#endif

    }
}