workspace {
    !identifiers hierarchical
    // definition of the model *********************************************************
    model {
        nucleo = softwareSystem "Nucleo Board" "Contains STM32 microcontroller and peripherals for control and communication" {
            stm32 = container "STM32L476RG Microcontroller" "Handles processing, GPIO, UART, and state management" {
                button_handler = component "Button Handler" "Captures button press events and passes them to the State Machine"
                uart_receiver = component "UART Command Receiver" "Processes 'Open' and 'Close' commands"
                state_machine = component "State Machine" "Central logic that manages system states"
                systick = component "SysTick Timer" "Provides millisecond timing precision and triggers timed state changes"
                led_handler = component "LED Handler" "Manages LED states based on the output of the State Machine"
                uart_debug = component "UART Debug" "Sends debug information based on state transitions and events"
            }
            user_button = container "User Button (PC13)" "Captures button press events"
            stlink = container "ST-Link Debugger & UART Interface" "Provides debugging and UART/USB communication" {
                uart_to_usb = component "UART-to-USB Bridge" "Converts UART data to USB format for PC communication"
                debug_interface = component "Debug Interface" "Handles debugging via SWD/JTAG"
            }
            led_heartbeat = container "LED Heartbeat (PA5)" "Indicates system activity"
            led_door = container "Door State LED (PA4)" "Shows the door's Locked/Unlocked state"
        }

        // external parties
        user = person "User" "Interacts with the system through the PC and button"
        pc = softwareSystem "PC" "Sends UART commands to control the Nucleo board"

        // relationships: user interaction
        user -> pc "Sends UART commands using terminal interface" "USB"
        user -> nucleo.user_button "Presses button to trigger events" "Physical Interaction"

        // relationships: inputs to state machine
        pc -> nucleo.stlink.uart_to_usb "Sends UART commands through USB" "USB"
        nucleo.stlink.uart_to_usb -> nucleo.stm32.uart_receiver "Converts USB commands to UART for STM32" "UART"
        nucleo.user_button -> nucleo.stm32.button_handler "Generates button press events" "GPIO"
        nucleo.stm32.uart_receiver -> nucleo.stm32.state_machine "Processes UART commands as events" "Internal Signal"
        nucleo.stm32.button_handler -> nucleo.stm32.state_machine "Passes button press events" "Internal Signal"
        nucleo.stm32.systick -> nucleo.stm32.state_machine "Triggers timed state transitions" "Internal Signal"

        // relationships: outputs from state machine
        nucleo.stm32.state_machine -> nucleo.stm32.led_handler "Triggers LED changes based on state" "Internal Signal"
        nucleo.stm32.state_machine -> nucleo.stm32.uart_debug "Sends debug messages for state changes" "Internal Signal"
        nucleo.stm32.led_handler -> nucleo.led_heartbeat "Controls LED for heartbeat" "GPIO"
        nucleo.stm32.led_handler -> nucleo.led_door "Controls LED for door state" "GPIO"
        nucleo.stm32.uart_debug -> nucleo.stlink.uart_to_usb "Forwards debug messages to the PC" "UART"
        nucleo.stlink.uart_to_usb -> pc "Displays debug messages in the terminal" "USB"
    }

    // definition of the views: context and component **********************************
    views {
        theme default
        systemContext nucleo "Nucleo_Context" {
            include *
            autolayout lr
        }
        container nucleo "Nucleo_Container" {
            include *
            autolayout lr
        }
        component nucleo.stm32 "STM32_Components" {
            include *
            autolayout lr
        }
        styles {
            element "MCU" {
                background #80bb80
            }
            element "Peripheral" {
                background #b0c4de
            }
            element "External System" {
                background #a0a0a0
            }
        }
    }
}
