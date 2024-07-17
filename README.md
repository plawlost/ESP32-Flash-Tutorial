# ESP32 Flash Tutorial

## Introduction
This guide provides detailed steps to force the ESP32 into bootloader mode using a Raspberry Pi Pico, erase its flash memory, and rewrite its software.

## Prerequisites
1. **Raspberry Pi Pico** with MicroPython firmware.
2. **ESP32** development board.
3. **USB to micro-USB cable** for connecting Pico to the computer.
4. **Jumper wires** for connecting Pico to ESP32.
5. **A computer** with Python and `pyserial` installed.

## Step 1: Flash MicroPython Firmware on Raspberry Pi Pico
1. **Download MicroPython Firmware:**
   - Download the latest MicroPython firmware for Raspberry Pi Pico from [here](https://micropython.org/download/rp2-pico/).

2. **Flash MicroPython Firmware:**
   - Hold the `BOOTSEL` button on the Pico and connect it to the computer via USB to enter mass storage mode.
   - Drag and drop the `MicroPython .uf2` file onto the Pico's mass storage device.

## Step 2: Connect the Pico to the ESP32
1. **Wiring Connections:**
   - **Pico GPIO 0 (TX)** to **ESP32 RX**
   - **Pico GPIO 1 (RX)** to **ESP32 TX**
   - **Pico GND** to **ESP32 GND**
   - **Pico 3.3V** to **ESP32 3.3V** (or Pico 5V to ESP32 5V if needed)
   - **Pico GPIO 2** to **ESP32 GPIO0 (BOOT)**
   - **Pico GPIO 3** to **ESP32 EN (RESET)**

2. **Connect the Pico to Your Computer:**
   - Use a USB to micro-USB cable to connect the Pico to your computer.

## Step 3: Write and Upload MicroPython Script to Pico
1. **Install Thonny IDE:**
   - Download and install Thonny IDE from [here](https://thonny.org/).

2. **Write the MicroPython Script:**
   - Open Thonny IDE.
   - Select `MicroPython (Raspberry Pi Pico)` as the interpreter.
   - Copy and paste the following script to control the ESP32 pins:

     ```python
     from machine import Pin
     import time

     # Define the pins
     boot = Pin(2, Pin.OUT)
     reset = Pin(3, Pin.OUT)

     # Function to put ESP32 in bootloader mode
     def enter_bootloader_mode():
         # Set GPIO0 (BOOT) to LOW
         boot.value(0)
         # Pulse the reset pin
         reset.value(0)
         time.sleep(0.1)
         reset.value(1)
         time.sleep(0.1)
         reset.value(0)
         time.sleep(0.1)
         # Release GPIO0 (BOOT)
         boot.value(1)

     # Enter bootloader mode
     enter_bootloader_mode()
     ```

3. **Upload the Script to Pico:**
   - Save the script with a filename like `bootloader.py`.
   - Click the `Run` button to upload and execute the script on the Pico.

## Step 4: Install Required Software on Computer
1. **Install `pyserial`:**
   ```sh
   pip install pyserial
   ```

2. **Install `minicom` or `screen`:**
   - On Linux:
     ```sh
     sudo apt-get install minicom
     ```
   - On macOS:
     ```sh
     brew install minicom
     ```

## Step 5: Erase the Flash Memory on ESP32
1. **Open a Terminal Session:**
   - For `minicom`:
     ```sh
     minicom -D /dev/ttyACM0 -b 115200
     ```
   - For `screen`:
     ```sh
     screen /dev/ttyACM0 115200
     ```
   - Note: Replace `/dev/ttyACM0` with the correct device path for your Pico (it may be `/dev/ttyUSB0` or another name).

2. **Run a Python Script to Erase Flash:**
   - Create a Python script named `erase_esp32.py` with the following content:
     ```python
     import serial
     import time

     ser = serial.Serial('/dev/ttyACM0', 115200)
     time.sleep(2)  # Wait for the connection to initialize

     # Command to erase flash (equivalent to what esptool.py does)
     # This is a placeholder command; you need the exact binary sequence to erase flash
     erase_command = b'\x07\x07\x12\x20\x00\x08\x0F\x0F'  # Placeholder command
     ser.write(erase_command)

     # Read response (if needed)
     response = ser.read_all()
     print(response)

     ser.close()
     ```

3. **Run the Script:**
   - Execute the script from the terminal:
     ```sh
     python erase_esp32.py
     ```

## Step 6: Rewrite the ESP32 Software
1. **Prepare the New Firmware Binary:**
   - Compile the new firmware using your preferred development environment (e.g., Arduino IDE or PlatformIO) and save the binary file.

2. **Upload the New Firmware:**
   - Modify the Python script to upload the new firmware:
     ```python
     import serial
     import time

     ser = serial.Serial('/dev/ttyACM0', 115200)
     time.sleep(2)  # Wait for the connection to initialize

     # Replace this with the actual path to your firmware binary
     firmware_path = 'path/to/your_firmware.bin'

     # Read the firmware binary
     with open(firmware_path, 'rb') as f:
         firmware = f.read()

     # Write the firmware to the ESP32
     ser.write(firmware)

     # Read response (if needed)
     response = ser.read_all()
     print(response)

     ser.close()
     ```

3. **Run the Script:**
   - Execute the script from the terminal:
     ```sh
     python upload_firmware.py
     ```

## Notes
- Ensure that the ESP32 is in bootloader mode during these operations.
- The placeholder erase command must be replaced with the actual binary command sequence for erasing flash memory.
- This method is a manual and low-level approach; using tools like `esptool.py` is highly recommended for accuracy and ease of use.
