# 4-Cell Li-Ion Battery Management System (BMS) Monitor

This repository contains the hardware design, Proteus simulation, and embedded firmware for a low-cost 4S1P Lithium-Ion Battery Management System (BMS) Monitor. Developed on the STM32F411RE Nucleo platform, this project continuously monitors battery pack conditions, displays real-time telemetry, and autonomously disconnects the load when predefined safety limits are exceeded.

The entire firmware is implemented in **bare-metal C**. It bypasses high-level Hardware Abstraction Layer (HAL) libraries for peripheral operations, utilizing direct register-level access for maximum control and educational demonstration of microcontroller architecture.

## System Architecture & Key Features

### 1. Bare-Metal Firmware Implementation
* **Register-Level Control:** Peripheral configuration, ADC reading, I2C communication, UART transmission, PWM control, and GPIO operations are written directly using STM32 device registers.
* **Polling & DWT Timing:** The system avoids custom ISRs (Interrupt Service Routines), managing ADC, I2C, USART, and sensor data acquisition through polling in a main super-loop. Timing is achieved using the Data Watchpoint and Trace (DWT) cycle counter for precise microsecond delays required by the DHT22 sensor.

### 2. Battery Node & Cell Monitoring
* **Voltage Sensing:** The system monitors four series-connected battery cells. Battery node voltages are scaled down using voltage divider circuits (with a scaling ratio of 5.545) before entering the STM32's ADC inputs. 
* **Cell Voltage Calculation:** Individual cell voltages and State of Charge (SoC) percentages are mathematically derived by subtracting consecutive node voltages.
* **Analog Multiplexing:** A CD74HC4051 analog multiplexer is integrated to allow the reading of multiple voltage channels efficiently.

### 3. Comprehensive Telemetry via I2C & GPIO
* **Current & Power:** An INA226 sensor communicates via I2C to precisely measure the main pack voltage and current.
* **Environmental Data:** A BME280 sensor (via I2C) tracks system temperature and atmospheric pressure, while a DHT22 sensor (via a single GPIO line) monitors ambient humidity.
* **UART Data Logging:** All sensor readings, calculated cell voltages, SoC, and motor status are transmitted at a 1 Hz update rate via USART2 (115200 baud) to a terminal interface (e.g., Tera Term).

### 4. Safety Protection Logic & Actuation
The system mimics the core protection philosophy of commercial BMS ICs by evaluating sensor data against strict safety thresholds.
* **Thresholds:** The load is disconnected if the total pack voltage falls outside the 12.0 V – 17.2 V range, temperature exceeds 30°C, humidity exceeds 68%, pressure is outside 900–1100 hPa, or the motor PWM demand reaches 75%.
* **Actuation & Indication:** In normal operation, a 2N2222 NPN transistor drives a DC motor based on a 10kΩ potentiometer's PWM demand (via TIM2) while a green LED remains active. 
* **Fault State:** Upon detecting a safety violation or the activation of the latched emergency stop button (PB0), the firmware forces the TIM2 PWM output to its stop value, disables the green LED, illuminates the red LED, and outputs a fault message to the terminal.

---

## Hardware Setup & Implementation

The physical circuit was systematically validated through Proteus simulations before being successfully transferred and soldered onto a perfboard (pertinaks). 

### Proteus Simulation Schematic

<img width="571" height="437" alt="proteus" src="https://github.com/user-attachments/assets/4c28564d-b9bf-40ec-9037-c30a49631db9" />

### Physical Circuit Integration (Perfboard)
<img width="2688" height="4499" alt="Bms pertinaks" src="https://github.com/user-attachments/assets/cd03aa26-2ced-4d3f-9d52-62889de37645" />

---

## UART Telemetry Output

Below is a demonstration of the data logged directly from the STM32F411RE via the UART terminal during operation:

<img width="568" height="420" alt="uart2" src="https://github.com/user-attachments/assets/a418660d-8573-4ee6-bf92-2c3d3d615a99" />


<img width="423" height="416" alt="uart1" src="https://github.com/user-attachments/assets/5a9b398d-ba02-497c-b634-2776a5bf98bc" />


---

## Source Code & Register-Level Operations

The core logic of this project resides in the `main.c` file. Below is an overview of the primary operations handled entirely via STM32 registers:

* **ADC Conversions:** Continuous single-channel reads configured via `ADC_SQR1` and `ADC_CR2` registers.
* **I2C Protocol:** Custom implementation to write to and read from the INA226 and BME280 sensors.
* **Safety Loop:** A fast 50 ms loop updates the soft-start PWM and LED states, while a slower 1-second loop recalculates battery stats and triggers safety flags if necessary.

