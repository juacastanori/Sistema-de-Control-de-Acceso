// Access Control System Implementation

#include "gpio.h"
#include "systick.h"
#include "uart.h"

#define TEMP_UNLOCK_DURATION 5000 // Duration in ms for temporary unlock

typedef enum {
    LOCKED,
    TEMP_UNLOCK,
    PERM_UNLOCK
} DoorState_t;



DoorState_t current_state = LOCKED;
uint32_t unlock_timer = 0;

void run_state_machine(void) {
    switch (current_state) {
        case LOCKED:
            // No periodic action in locked state
            break;
        case TEMP_UNLOCK:
            if (systick_GetTick() - unlock_timer >= TEMP_UNLOCK_DURATION) {
                gpio_set_door_led_state(0); // Turn off door state LED
                current_state = LOCKED;
            }
            break;
        case PERM_UNLOCK:
            // No periodic action in permanent unlock state
            break;
    }
}

void handle_event(uint8_t event) {
    if (event == 1) { // Single button press
        usart2_send_string("Single User Button press\r\n");
        gpio_set_door_led_state(1); // Turn on door state LED
        current_state = TEMP_UNLOCK;
        unlock_timer = systick_GetTick();
    } else if (event == 2) { // Double button press
        usart2_send_string("Double User Button press\r\n");
        gpio_set_door_led_state(1); // Turn on door state LED
        current_state = PERM_UNLOCK;
    } else if (event == 'O') { // UART OPEN command
        usart2_send_string("Door Unlocked by Open Command\r\n");
        gpio_set_door_led_state(1); // Turn on door state LED
        current_state = PERM_UNLOCK;
        unlock_timer = systick_GetTick();
    } else if (event == 'C') { // UART CLOSE command
        usart2_send_string("Door Unlocked by Close Command\r\n");
        gpio_set_door_led_state(0); // Turn off door state LED
        current_state = LOCKED;
    } else if (current_state == PERM_UNLOCK && (event==1 || event==2)) { // Turn off door unlocked by pushing the button one time
        usart2_send_string("Single User Button press\r\n");
        gpio_set_door_led_state(0); 
        current_state = LOCKED;
    }//
}

int main(void) {
    configure_systick_and_start();
    configure_gpio();
    usart2_init();

    usart2_send_string("System Initialized\r\n");

    uint32_t heartbeat_tick = 0;
    while (1) {
        if (systick_GetTick() - heartbeat_tick >= 500) {
            heartbeat_tick = systick_GetTick();
            gpio_toggle_heartbeat_led();
        }

        uint8_t button_pressed = button_driver_get_event();
        if (button_pressed != 0) {
            usart2_send_string("Button press detected\r\n");
            handle_event(button_pressed);
            button_pressed = 0;
        }

        uint8_t rx_byte = usart2_get_command();
        if (rx_byte != 0) {
            usart2_send_string("Command received\r\n");
            handle_event(rx_byte);
            rx_byte = 0;
        }

        run_state_machine();
    }
}
