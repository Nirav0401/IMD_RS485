# Insulation Monitoring Device (IMD) with RS485 Communication
This project is designed to interface with an Insulation Monitoring Device (IMD) using a UART interface. It utilizes STM32 microcontrollers to communicate with the IMD and fetch various parameters such as DC voltage, total insulation resistance, positive bus insulation resistance, and negative bus insulation resistance.

### Features:
* UART Communication: Uses USART5 for debugging and logging purposes.
* RS485 Communication: Uses RS485 communication (USART4) with the IMD.
* Logging: Sends the processed data over UART for logging and debugging.

## How to Use Example

### Hardware Required
* Any STM32 Microcontroller based Development Board(Tested on STM32F4xx series)
* Insulation Monitoring Device (IMD)
* USB to UART converter (for debugging)

### Hardware Setup:
* Connect the IMD device to the STM32 microcontroller via RS485 interfaces.
* Connect the USB to UART converter to the STM32 microcontroller for debugging.

## Example Output

````bash
- After flashing the project to the STM32 microcontroller, the IMD will start monitoring the insulation and sending data over UART.
- Open a serial terminal (e.g., Tera Term, PuTTY) to view the logged data.
- The data includes DC voltage, total insulation resistance, positive bus insulation resistance, and negative bus insulation resistance.

````
