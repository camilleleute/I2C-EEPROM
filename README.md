# A8 - I2C EEPROM

## Reference Materials
- STM32L4xxxx Reference Manual (RM) – I2C
- STM32L4x6xx Datasheet (DS) – Pinouts
- NUCLEO-L4x6xG Users Manual (UM) – Pin Diagram (L476RG, L4A6ZG, L496ZG)
- Microchip 24LC256 EEPROM Datasheet

## I2C - Inter Integrated Circuit
I2C is a common half-duplex, 2-wire bus used by a variety of digital devices. The I2C protocol is more complex than SPI or RS-232 allowing for 2-way acknowledgments and addressed devices, The STM32L4 provides multiple I2C outputs via the I2C peripherals. Configuration options can be reviewed in the corresponding TRM chapter. 

## EEPROM - Electronic Erasable Programmable Read-Only Memory
EEPROM is a type of nonvolatile memory that can be erased electronically. Memory is considered nonvolatile if the data is retained between power cycles. More prevalent flash memory is a type of EEPROM, but the designation EEPROM is used to distinguish memory that is byte erasable. Flash memory can only be erased in larger blocks. The 24LC256 EEPROM is a 256 Kbit (32k x 8), so it is byte addressable with 32k (215) addresses. It has a configurable device address that ranges from 0x50 to 0x57 based on how it is wired. The EEPROM communicates with the microcontroller via I2C bus with a maximum speed of 400 kHz. Details on how data can be read from and written to the EEPROM are in its datasheet. 

## Instructions
### Interface STM32L4 and EEPROM
- Review the EEPROM datasheet and select a device address to use in the range 0x51-0x57. (Do NOT use 0x50)
- Draw a schematic connecting the 24LC256 EEPROM to the STM32L4 with I2C. The datasheet (DS) can be used to find which pins are selectable for I2C operations. 
- Wire the schematic on the breadboard
- Use STM32IDE to calculate the I2C timing factor for the clock and I2C speed. Do NOT save the configuration, just make note of the timing factor!
### Write a program to use the EEPROM
- Write some functions to utilize the EEPROM and verify they function properly
  - EEPROM_init - initialize an I2C peripheral to communicate with the EEPROM
  - EEPROM_read - read a byte of data from a given 15-bit address
  - EEPROM_write - write a given byte of data to a given 15-bit address
- Use the above functions in a program to write a random byte of data to the EEPROM at a random 15-bit address. (Your program does not need to generate random values, they can be hard coded into your program, but the values you select should be “random”. You can use https://www.random.org/bytes/ to select random values to use) Wait 5ms for the data to be saved then read a byte of data from the same address. Compare the data read with the data sent to be saved and If the values match, the onboard LED should turn on, and if not, the LED should stay off.
### Analyze the I2C Bus
- Take two screenshots of the SCL (I2C clock) and SDA (I2C data) using the logic analyzer. Write on the screenshots or otherwise mark the start, restart, stop, device address, R/W, ACKs, NACKs, and data being transmitted.
  - Entire transmission for writing a byte of data
  - Entire transmission for reading a byte of data

## Hints
- If errors occur during transmission and a STOP condition is not transmitted, the EEPROM will stay in the same state, waiting for data or a stop condition to release the bus. This can cause the STM32L4 to see the bus as occupied when running or rerunning code on a successive attempt. It will then wait indefinitely before sending out a new START. This can also cause the EEPROM to not acknowledge when its address is sent by the STM32L4, which can affect how the STM32L4 handles sending data after a start condition. To avoid these issues, power to the EEPROM can be quickly cycled by removing and replacing the 3.3 V wire connected to the VCC pin.
- The EEPROM requires a 5 ms delay after writing data before any other operations can occur. Failure to keep this delay after writing will cause the EEPROM to behave unexpectedly.

## Deliverables
- Create a single pdf document (not a full lab report) containing the following:
- Screenshots of I2C bus analysis 
- Properly formatted C source code
