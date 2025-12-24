**Project README**

This document explains how to use the firmware in `Core/Src/main.c`, the PCB I/O (buttons, OLED, servos, USART), and how to open / build / flash the project using STM32Cube IDE, STM32CubeMX and STM32CubeProgrammer.

**Quick Hardware Map**
- **Buttons:**
  ![PCB buttons SW2 & SW3](PCB/1.jpg)
  - **SW2 (PB12):** Toggle gripper servo. Pressing toggles between open (TIM8 CH1 = 1000) and close (TIM8 CH1 = 1400).
  - **SW3 (PB13):** Toggle motors on/off. When enabled the code sets `left_cmd = 30000` and `right_cmd = 30000` and calls `motor()` to move forward; when disabled it calls `motor(0,0)` to stop.
- **OLED (SSD1306) - row-by-row display:**
  ![OLED rows example](PCB/2.jpg)
  - Row 0 (cursor 0,0, Font_11x18): boot title / voltage & current formatted string (example: "3360 251224" or "x.xV ...A").
  - Row y=16 (Font_6x8): `L:<count> <B|F>` — left encoder count and direction.
  - Row y=24 (Font_6x8): `R:<count> <B|F>` — right encoder count and direction.
  - Row y=32 (Font_6x8): `Motor PWM L:<value>` — left motor PWM command value.
  - Row y=40 (Font_6x8): `Motor PWM R:<value>` — right motor PWM command value.
  - Row y=48 (Font_6x8): `Servo1:<value>` — TIM8 CH1 compare (servo pulsewidth).
  - Row y=56 (Font_6x8): `Servo2:<value>` — TIM8 CH2 compare (servo pulsewidth).
-- **Servo channels:**
  ![Servo channels (TIM8)](PCB/3.jpg)
  - The firmware uses **TIM8** PWM channels:
    - `TIM8 CH1` — Servo 1 (gripper)
    - `TIM8 CH2` — Servo 2 (reserved/second servo)
- **Motor PWM and encoders:**
  - Right motor PWM: `htim12` channels (TIM12 CH1 & CH2).
  - Left motor PWM: `htim4` channels (TIM4 CH3 & CH4).
  - Encoders: `htim2` (left) and `htim5` (right).
-- **USART / USB-TTL:**
  ![USART / USB-TTL wiring](PCB/4.jpg)
  - Firmware uses `USART2` for external control frames and telemetry.
  - USB TTL <--> PCB wiring:
    - USB-TTL `RXD` <--> PCB `TDX`
    - USB-TTL `TDX` <--> PCB `RXD`
    - USB-TTL `GND` <--> PCB `GND`

**How the firmware uses peripherals (summary of `main.c`)**
- UART/Remote control:
  - `USART2` receives single bytes via `HAL_UART_Receive_IT(..., 1)` into a software ring buffer.
  - A framed protocol is parsed: header `0x0D`, footer `0x20`, payload contains motor & servo values (frame length 11 bytes).
  - Motor values `m1`/`m2` (16-bit) and servo values `s1`/`s2` (clamped to 1000..1400) are extracted and applied.
  - `dir` byte sets motor direction bits (bit0 -> motor1/left, bit1 -> motor2/right); 0 = backward, 1 = forward.
- Motors and servos:
  - `motor(left_cmd,right_cmd)` clamps inputs and sets TIM compare registers to control direction and PWM magnitude.
  - Servos are controlled by `__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1/2, value)`.
- ADC and OLED:
  - ADC DMA fills `ADCArray[2]` and a moving average (window size 50) is computed for voltage and current.
  - OLED displays voltage/current and telemetry rows described above.
- Buttons (GPIO EXTI):
  - `HAL_GPIO_EXTI_Callback()` handles `SW2_Pin` and `SW3_Pin` toggles as described above.
- Telemetry:
  - Encoder counts (from `htim2` and `htim5`) are sent periodically by `Send_Encoder_Counts()` via `HAL_UART_Transmit_IT(&huart2, ...)`.

**Open / Build / Flash (STM32Cube IDE + tools)**
1. Install the tools (recommended versions used when creating this project):
   - STM32CubeIDE 2.0: https://www.st.com/en/development-tools/stm32cubeide.html
   - STM32CubeMX 6.16.1: https://www.st.com/en/development-tools/stm32cubemx.html
   - STM32CubeProgrammer 2.21: https://www.st.com/en/development-tools/stm32cubeprog.html
2. Download this repository as a ZIP and extract it to a convenient folder.
3. Open STM32CubeIDE 2.0 -> File -> Import -> General -> "Projects from Folder or Archive". 
  ![IDE import step 1](IDE/1.jpg) ![IDE import step 2](IDE/2.jpg)
4. Click "Directory" and select the extracted repository folder (e.g. C:\Users\admin\Downloads\EIE3360_2526_ControlMCU-main\EIE3360_2526_ControlMCU-main). Click "Finish".
  ![IDE choose directory](IDE/3.jpg)
5. In Project Explorer open `3360_2526.ioc`.
  ![Open IOC in Project Explorer](IDE/4.jpg)
6. STM32CubeMX will open and load the IOC.
  ![STM32CubeMX loads IOC](IDE/5.jpg)
7. Click "GENERATE CODE" (top-right).
  ![Generate code](IDE/6.jpg)
8. After code generation click "Close".
9. In Project Explorer open `3360_2526/Core/Src/main.c`.
  ![Open main.c](IDE/7.jpg)
10. Build: Project -> Build Project.
  ![Build project](IDE/8.jpg)
11. On success you will see "Build Finished".
  ![Build finished](IDE/9.jpg)
12. Right-click Project Explorer -> select `3360_2526/Debug/3360_2526.elf` -> Show In -> System Explorer.
  ![Show ELF in system explorer](IDE/10.jpg)
13. Open STM32CubeProgrammer, click Open File and choose the ELF from step 12.
  ![Open ELF in CubeProgrammer](IDE/11.jpg)
14. Connect the board: use a USB-C to A cable to the board.
  ![Connect board via USB](IDE/12.jpg)
15. To enter bootloader: power on, press-and-hold `BOOT0`, press-and-release `RST`, then release `BOOT0`.
  ![Enter bootloader sequence](IDE/13.jpg)
16. In STM32CubeProgrammer select "USB" and refresh ports. If port not found, repeat step 15.
  ![Select USB in CubeProgrammer](IDE/14.jpg)
17. When port is found press "Connect" and then "Download".
  ![Connect and download](IDE/15.jpg)
18. After download, press "Confirm" and disconnect.
  ![Confirm and disconnect](IDE/16.jpg)
19. Press `RST` to leave download mode and start the program.
20. Firmware is now running.
21. To modify behaviour: edit `Core/Src/main.c`, rebuild (step 10) and reflash (steps 12–19).

**Advanced / Notes**
- The UART frame decoder expects big-endian 16-bit motor/servo values and a frame structure; if integrating a controller, match that format.
- Servo safe range is clamped to 1000..1400 in software.
- Motor commands are capped to ±65535. Direction is controlled by writing either forward or backward compare values to the paired TIM channels.
- If USB-TTL communications fail, verify the RX/TX cross wiring and common ground.

**Files of interest**
- `Core/Src/main.c` — main application logic (buttons, OLED, ADC, UART parsing, motor/servo control).
- `Core/Inc/` & `Core/Src/` — peripheral initialisation (`gpio.c`, `tim.c`, `usart.c`, `adc.c`, `i2c.c` etc.).
- `IDE/` and `PCB/` images — follow the numbered IDE screenshots and PCB labeling for wiring reference.

If you want, I can also:
- Add a short diagram showing TIM/channel ↔ PCB pin mappings.
- Add a sample Python script to send properly-framed UART control packets.

---
Generated to document `main.c` usage and flashing steps.
