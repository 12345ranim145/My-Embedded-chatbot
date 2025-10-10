from django.shortcuts import render
from django.http import JsonResponse
import json

# ===================== Base de données =====================
CARTE_INFO = {
    # ===================== ESP32 =====================
    "esp32": {
        "type": "Microcontroller",
        "power_supply": "2.7V to 3.6V (3.3V typical)",
        "power_consumption": "80 mA @ 240 MHz (active), ~20 µA (deep sleep)",
        "gpio_logic_voltage": "3.3V",
        "gpio_max_current": "12 mA per pin (source/sink)",
        "cpu": "Tensilica Xtensa dual-core 32-bit LX6, up to 240 MHz",
        "ram": "520 KB SRAM internal",
        "flash": "Up to 16 MB external (SPI)",
        "gpio": "34 digital pins, 18 ADC (12-bit), 2 DAC (8-bit), PWM, I2C, SPI, UART, WiFi, Bluetooth",
        "usage": "IoT, home automation, connected projects, embedded sensors, lightweight robotics",
        "protocols_detail": {
            "WiFi": {
                "description": "802.11 b/g/n, supports local and internet connectivity.",
                "usage": "IoT, embedded web servers, OTA updates.",
                "references": ["https://www.espressif.com/en/products/socs/esp32"]
            },
            "Bluetooth BLE": {
                "description": "Bluetooth 4.2 and BLE for low-power short-range communication.",
                "usage": "Wireless sensors, wearables, point-to-point data exchange.",
                "references": ["https://www.bluetooth.com/specifications/"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for high-speed, full-duplex communication.",
                "usage": "Interfacing with sensors, displays, and memory devices.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            },
            "I2C": {
                "description": "Inter-Integrated Circuit, a two-wire serial bus for short-range communication.",
                "usage": "Connecting multiple sensors and peripherals with minimal pins.",
                "references": ["https://www.nxp.com/docs/en/user-guide/UM10204.pdf"]
            },
            "UART": {
                "description": "Universal Asynchronous Receiver/Transmitter for serial communication.",
                "usage": "Serial communication with modules, sensors, or other microcontrollers.",
                "references": ["https://www.ti.com/lit/an/slaa037/slaa037.pdf"]
            }
        },
        "languages": ["C", "C++", "Python (MicroPython)", "Arduino IDE"],
        "ide": ["VS Code", "Arduino IDE", "PlatformIO", "Espressif IDF"],
        "image": "/static/images/esp32.jpg",
        "firmware_development": {
            "toolchain": "ESP-IDF (Espressif IoT Development Framework), FreeRTOS support",
            "debugging": "JTAG debugging, UART logging",
            "power_optimization": "Deep sleep, ULP coprocessor for low-power tasks"
        },
        "additional_electronics": {
            "pinout": "GPIO0-39 with multiplexed functions; avoid GPIO12 during boot.",
            "voltage_regulator": "Built-in 3.3V regulator, max input 5V via USB.",
            "adc_resolution": "12-bit SAR ADC, non-linear above 2.4V without calibration."
        }
    },

    # ===================== ESP32-CAM =====================
    "esp32 cam": {
        "type": "Microcontroller + Camera",
        "power_supply": "5V via USB or Vin 5V",
        "power_consumption": "160 mA (camera active), ~20 µA (deep sleep)",
        "cpu": "Tensilica Xtensa dual-core 32-bit LX6, up to 240 MHz",
        "ram": "520 KB SRAM internal + 4 MB PSRAM",
        "flash": "4-16 MB",
        "gpio": "10 GPIO (shared with camera and SD card), ADC, PWM",
        "usage": "IoT camera, surveillance, video streaming",
        "protocols_detail": {
            "WiFi": {
                "description": "802.11 b/g/n for local and internet connectivity.",
                "usage": "Video streaming, embedded web server.",
                "references": ["https://www.espressif.com/en/products/socs/esp32"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for high-speed communication.",
                "usage": "Interfacing with camera and SD card.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            }
        },
        "languages": ["C", "C++", "Python (MicroPython)", "Arduino IDE"],
        "ide": ["VS Code", "Arduino IDE", "PlatformIO", "Espressif IDF"],
        "image": "/static/images/esp32 cam.jpg",
        "firmware_development": {
            "toolchain": "ESP-IDF, Arduino framework",
            "debugging": "UART-based debugging",
            "camera": "OV2640 (2MP), supports MJPEG streaming"
        },
        "additional_electronics": {
            "pinout": "Limited GPIOs; GPIO0 for boot mode, GPIO4 for SD card.",
            "voltage_regulator": "5V to 3.3V step-down, careful with current draw for camera.",
            "camera_interface": "Parallel 8-bit interface with DVP protocol."
        }
    },

    # ===================== ESP32 TTGO T-Beam =====================
    "esp32 ttgo t-beam": {
        "type": "Microcontroller + LoRa + GPS",
        "power_supply": "3.3V to 5V (USB or battery)",
        "power_consumption": "100 mA (active with LoRa/GPS), ~20 µA (deep sleep)",
        "gpio_logic_voltage": "3.3V",
        "gpio_max_current": "12 mA per pin (source/sink)",
        "cpu": "Tensilica Xtensa dual-core 32-bit LX6, up to 240 MHz",
        "ram": "520 KB SRAM internal + 8 MB PSRAM",
        "flash": "4-16 MB",
        "gpio": "22 GPIO, 12 ADC (12-bit), PWM, I2C, SPI, UART",
        "usage": "LoRa-based IoT, GPS tracking, remote sensing, smart agriculture",
        "protocols_detail": {
            "LoRa": {
                "description": "Long-range, low-power wireless communication using Semtech SX1276.",
                "usage": "Long-distance IoT, remote sensors, asset tracking.",
                "references": ["https://www.semtech.com/lora/what-is-lora"]
            },
            "WiFi": {
                "description": "802.11 b/g/n for local and internet connectivity.",
                "usage": "IoT, embedded web servers.",
                "references": ["https://www.espressif.com/en/products/socs/esp32"]
            },
            "Bluetooth BLE": {
                "description": "Bluetooth 4.2 and BLE for low-power short-range communication.",
                "usage": "Wireless sensors, wearables.",
                "references": ["https://www.bluetooth.com/specifications/"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for high-speed communication.",
                "usage": "Interfacing with LoRa module and GPS.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            },
            "I2C": {
                "description": "Inter-Integrated Circuit for short-range communication.",
                "usage": "Connecting sensors like BME680.",
                "references": ["https://www.nxp.com/docs/en/user-guide/UM10204.pdf"]
            },
            "UART": {
                "description": "Universal Asynchronous Receiver/Transmitter for serial communication.",
                "usage": "GPS module communication, serial debugging.",
                "references": ["https://www.ti.com/lit/an/slaa037/slaa037.pdf"]
            }
        },
        "languages": ["C", "C++", "Python (MicroPython)", "Arduino IDE"],
        "ide": ["VS Code", "Arduino IDE", "PlatformIO", "Espressif IDF"],
        "image": "/static/images/esp32_ttgo_tbeam.jpg",
        "firmware_development": {
            "toolchain": "ESP-IDF, Arduino framework",
            "debugging": "UART-based debugging, JTAG",
            "power_optimization": "Deep sleep, ULP coprocessor"
        },
        "additional_electronics": {
            "pinout": "GPIO0-39 (some reserved for LoRa/GPS), multiplexed functions.",
            "voltage_regulator": "Built-in 3.3V regulator, supports LiPo battery.",
            "gps_module": "U-blox NEO-6M/7M/8M, supports GPS, GLONASS.",
            "lora_module": "Semtech SX1276, supports 433/868/915 MHz."
        }
    },

    # ===================== STM32F407VG =====================
    "stm32f407vg": {
        "type": "ARM Cortex-M4 Microcontroller",
        "power_supply": "1.8V to 3.6V (3.3V typical)",
        "power_consumption": "20 mA @ 168 MHz (active), ~2 µA (stop mode)",
        "cpu": "32-bit ARM Cortex-M4 with FPU, 168 MHz",
        "ram": "192 KB SRAM",
        "flash": "1 MB",
        "gpio": "82 GPIO (16 ports), 12-bit ADC, 8-bit DAC, PWM, UART, SPI, I2C, CAN, USB OTG",
        "usage": "Advanced embedded projects, robotics, industrial systems",
        "protocols_detail": {
            "CAN": {
                "description": "Controller Area Network for robust, real-time communication.",
                "usage": "Automotive, industrial control, inter-device communication.",
                "references": ["https://www.st.com/resource/en/application_note/an2606.pdf"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for high-speed, full-duplex communication.",
                "usage": "Interfacing with sensors, displays, and memory devices.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            },
            "I2C": {
                "description": "Inter-Integrated Circuit, a two-wire serial bus for short-range communication.",
                "usage": "Connecting multiple sensors and peripherals with minimal pins.",
                "references": ["https://www.nxp.com/docs/en/user-guide/UM10204.pdf"]
            },
            "UART": {
                "description": "Universal Asynchronous Receiver/Transmitter for serial communication.",
                "usage": "Serial communication with modules, sensors, or other microcontrollers.",
                "references": ["https://www.ti.com/lit/an/slaa037/slaa037.pdf"]
            }
        },
        "languages": ["C", "C++", "Python (MicroPython)", "HAL/LL STM32"],
        "ide": ["STM32CubeIDE", "VS Code", "Keil uVision", "CooCox CoIDE"],
        "frameworks": ["STM32 HAL", "FreeRTOS", "mbed OS"],
        "image": "/static/images/stm32f407vg.jpg",
        "tutorials": [
            "STM32F4 Getting Started: https://deepbluembedded.com/stm32f4-discovery-board-tutorials/",
            "STM32 CAN Tutorial: https://controllerstech.com/stm32-can-tutorial/"
        ],
        "firmware_development": {
            "toolchain": "STM32CubeIDE, ARM GCC",
            "debugging": "SWD/JTAG, ST-Link",
            "power_optimization": "Low-power modes (sleep, stop, standby)"
        },
        "additional_electronics": {
            "pinout": "Ports A-P with alternate functions; VDD/VSS for power domains.",
            "voltage_regulator": "Internal LDO for core, external for peripherals if needed.",
            "adc_resolution": "12-bit, up to 2.4 MSPS, with DMA support."
        }
    },

    # ===================== Arduino Uno =====================
    "arduino uno": {
        "type": "AVR Microcontroller",
        "power_supply": "5V via USB or Vin 7-12V",
        "power_consumption": {"active": "50 mA", "sleep": "0.5 mA"},
        "cpu": "ATmega328P, 16 MHz",
        "ram": "2 KB SRAM",
        "flash": "32 KB (0.5 KB used by bootloader)",
        "gpio": "14 digital, 6 analog (10-bit ADC), PWM, UART, SPI, I2C",
        "usage": "Prototyping, simple IoT, educational robotics",
        "protocols_detail": {
            "UART": {
                "description": "Universal Asynchronous Receiver/Transmitter for serial communication.",
                "usage": "Serial communication with modules, sensors, or other microcontrollers.",
                "references": ["https://www.ti.com/lit/an/slaa037/slaa037.pdf"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for high-speed, full-duplex communication.",
                "usage": "Interfacing with sensors, displays, and memory devices.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            },
            "I2C": {
                "description": "Inter-Integrated Circuit, a two-wire serial bus for short-range communication.",
                "usage": "Connecting multiple sensors and peripherals with minimal pins.",
                "references": ["https://www.nxp.com/docs/en/user-guide/UM10204.pdf"]
            },
            "1-Wire": {
                "description": "Simple protocol for sensors like DS18B20.",
                "usage": "Temperature measurement, unique sensor addressing.",
                "references": ["https://www.maximintegrated.com/en/products/1-wire.html"]
            }
        },
        "languages": ["C", "C++", "Arduino IDE"],
        "ide": ["Arduino IDE", "VS Code + PlatformIO"],
        "image": "/static/images/arduino uno.jpg",
        "firmware_development": {
            "toolchain": "AVR-GCC, Arduino framework",
            "debugging": "Serial monitor, limited debugging support",
            "power_optimization": "Sleep modes via AVR libraries"
        },
        "additional_electronics": {
            "pinout": "Digital 0-13, Analog A0-A5; PWM on 3,5,6,9,10,11.",
            "voltage_regulator": "On-board 5V and 3.3V regulators.",
            "adc_resolution": "10-bit, reference voltage 5V or external."
        }
    },

    # ===================== Arduino Mega =====================
    "arduino mega": {
        "type": "AVR Microcontroller",
        "power_supply": "5V via USB or Vin 7-12V",
        "power_consumption": {"active": "70 mA", "sleep": "0.5 mA"},
        "cpu": "ATmega2560, 16 MHz",
        "ram": "8 KB SRAM",
        "flash": "256 KB (8 KB used by bootloader)",
        "gpio": "54 digital, 16 analog (10-bit ADC), PWM, 4 UART, SPI, I2C",
        "usage": "Complex projects, robotics, automation",
        "protocols_detail": {
            "UART": {
                "description": "Universal Asynchronous Receiver/Transmitter for serial communication.",
                "usage": "Serial communication with modules, sensors, or other microcontrollers.",
                "references": ["https://www.ti.com/lit/an/slaa037/slaa037.pdf"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for high-speed, full-duplex communication.",
                "usage": "Interfacing with sensors, displays, and memory devices.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            },
            "I2C": {
                "description": "Inter-Integrated Circuit, a two-wire serial bus for short-range communication.",
                "usage": "Connecting multiple sensors and peripherals with minimal pins.",
                "references": ["https://www.nxp.com/docs/en/user-guide/UM10204.pdf"]
            }
        },
        "languages": ["C", "C++", "Arduino IDE"],
        "ide": ["Arduino IDE", "VS Code + PlatformIO"],
        "image": "/static/images/arduino_mega.png",
        "firmware_development": {
            "toolchain": "AVR-GCC, Arduino framework",
            "debugging": "Serial monitor, limited debugging support",
            "power_optimization": "Sleep modes via AVR libraries"
        },
        "additional_electronics": {
            "pinout": "Digital 0-53, Analog A0-A15; Multiple UARTs (0-3).",
            "voltage_regulator": "On-board 5V and 3.3V regulators.",
            "adc_resolution": "10-bit, with AREF pin for custom reference."
        }
    },

    # ===================== Raspberry Pi 4 =====================
    "raspberry pi 4": {
        "type": "Single-Board Computer",
        "power_supply": "5V DC via USB-C connector (minimum 3A)",
        "power_consumption": "Idle: ~3W, Load: up to 7W",
        "gpio_logic_voltage": "3.3V",
        "gpio_max_current": "16 mA per pin (total max 50 mA)",
        "cpu": "Broadcom BCM2711, Quad core Cortex-A72 (ARM v8) 64-bit SoC @ 1.8GHz",
        "ram": "1GB, 2GB, 4GB or 8GB LPDDR4-3200 SDRAM",
        "flash": "Micro-SD card slot",
        "gpio": "40 pin GPIO header, I2C, SPI, UART",
        "usage": "General-purpose computing, IoT, robotics, media centers, embedded projects",
        "protocols_detail": {
            "WiFi": {
                "description": "2.4 GHz and 5.0 GHz IEEE 802.11ac wireless.",
                "usage": "Wireless networking.",
                "references": ["https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/"]
            },
            "Bluetooth": {
                "description": "Bluetooth 5.0, BLE.",
                "usage": "Short-range wireless communication.",
                "references": ["https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/"]
            },
            "Ethernet": {
                "description": "Gigabit Ethernet.",
                "usage": "Wired networking.",
                "references": ["https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for high-speed communication.",
                "usage": "Interfacing with sensors, displays, and memory devices.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            },
            "I2C": {
                "description": "Inter-Integrated Circuit for short-range communication.",
                "usage": "Connecting sensors like BME680.",
                "references": ["https://www.nxp.com/docs/en/user-guide/UM10204.pdf"]
            },
            "UART": {
                "description": "Universal Asynchronous Receiver/Transmitter for serial communication.",
                "usage": "Serial communication with modules or sensors.",
                "references": ["https://www.ti.com/lit/an/slaa037/slaa037.pdf"]
            }
        },
        "languages": ["Python", "C", "C++", "Java", "Scratch"],
        "ide": ["VS Code", "Thonny", "Geany"],
        "image": "/static/images/raspberry pi 4.jpg",
        "firmware_development": {
            "toolchain": "Raspberry Pi OS (Linux-based), supports various distros",
            "debugging": "GDB, UART",
            "power_optimization": "Power management via config"
        },
        "additional_electronics": {
            "pinout": "40-pin header compatible with previous models",
            "voltage_regulator": "On-board",
            "adc_resolution": "No built-in ADC"
        }
    },

    # ===================== Jetson Nano =====================
    "jetson nano": {
        "type": "AI Single-Board Computer",
        "power_supply": "5V DC via Micro-USB or barrel jack (minimum 2A, recommended 4A)",
        "power_consumption": "5W - 10W",
        "gpio_logic_voltage": "3.3V",
        "gpio_max_current": "Up to 16 mA per pin",
        "cpu": "Quad-core ARM Cortex-A57 MPCore processor @ 1.43GHz",
        "ram": "4GB 64-bit LPDDR4",
        "flash": "16GB eMMC 5.1 or Micro-SD card",
        "gpio": "40 pin GPIO header, 3x UART, 2x SPI, 2x I2S, 4x I2C, GPIOs",
        "usage": "AI, machine learning, computer vision, robotics, embedded AI projects",
        "protocols_detail": {
            "Ethernet": {
                "description": "1x GbE.",
                "usage": "Wired networking.",
                "references": ["https://developer.nvidia.com/embedded/jetson-nano"]
            },
            "USB": {
                "description": "1x USB 3.0, 3x USB 2.0.",
                "usage": "Peripheral connectivity.",
                "references": ["https://developer.nvidia.com/embedded/jetson-nano"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for high-speed communication.",
                "usage": "Interfacing with sensors, displays, and memory devices.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            },
            "I2C": {
                "description": "Inter-Integrated Circuit for short-range communication.",
                "usage": "Connecting sensors like BME680.",
                "references": ["https://www.nxp.com/docs/en/user-guide/UM10204.pdf"]
            },
            "UART": {
                "description": "Universal Asynchronous Receiver/Transmitter for serial communication.",
                "usage": "Serial communication with modules or sensors.",
                "references": ["https://www.ti.com/lit/an/slaa037/slaa037.pdf"]
            }
        },
        "languages": ["Python", "C", "C++"],
        "ide": ["VS Code", "JetPack SDK tools"],
        "image": "/static/images/jetson_nano.jpg",
        "firmware_development": {
            "toolchain": "JetPack SDK, Ubuntu-based",
            "debugging": "CUDA, TensorRT",
            "power_optimization": "Power modes 5W/10W"
        },
        "additional_electronics": {
            "pinout": "260-pin SO-DIMM connector for module",
            "voltage_regulator": "On-board",
            "adc_resolution": "No built-in ADC"
        }
    },

    # ===================== SX1276 (LoRa Module) =====================
    "sx1276": {
        "type": "LoRa Transceiver Module",
        "power_supply": "1.8V to 3.7V (3.3V typical)",
        "power_consumption": "120 mA (transmit), 9.9 mA (receive), 0.2 µA (sleep)",
        "frequency_bands": ["433 MHz, 868 MHz, 915 MHz"],
        "interfaces": ["SPI"],
        "usage": "Long-range IoT, remote sensors, smart agriculture, asset tracking",
        "protocols_detail": {
            "LoRa": {
                "description": "Long-range, low-power wireless communication.",
                "usage": "Long-distance IoT, remote sensors.",
                "references": ["https://www.semtech.com/lora/what-is-lora"]
            },
            "SPI": {
                "description": "Serial Peripheral Interface for communication with microcontrollers.",
                "usage": "Interfacing with host microcontroller like ESP32 or STM32.",
                "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"]
            }
        },
        "image": "/static/images/sx1276.jpg",
        "additional_electronics": {
            "pinout": "MISO, MOSI, SCK, NSS for SPI; DIO0-DIO5 for interrupts.",
            "voltage_regulator": "Requires external 3.3V supply.",
            "modulation": "LoRa, FSK, GFSK, OOK."
        }
    }
}

# ===================== Capteurs =====================
CAPTEUR_INFO = {
    "bme680": {
        "type": "Environmental Sensor",
        "measurements": ["Temperature", "Humidity", "Pressure", "Air Quality (VOC)"],
        "power_supply": "1.71V to 3.6V",
        "interfaces": ["I2C", "SPI"],
        "power_consumption": "0.15 mA (normal mode), 0.1 µA (sleep mode)",
        "usage": "Weather stations, home automation, air quality monitoring",
        "signal_type": "Digital",
        "references": ["https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/"],
        "tutorials": [
            "BME680 with Arduino: https://randomnerdtutorials.com/bme680-sensor-arduino/",
            "BME680 with ESP32: https://randomnerdtutorials.com/esp32-bme680-sensor-arduino-ide/",
            "BME680 with STM32: https://github.com/Squieler/BME68x-STM32-HAL"
        ],
        "image": "/static/images/bme680.jpg"
    },
    "dht22": {
        "type": "Temperature and Humidity Sensor",
        "temperature_range": "-40°C to +80°C",
        "temperature_accuracy": "±0.5°C",
        "humidity_range": "0–100% RH",
        "humidity_accuracy": "±2–5% RH",
        "power_supply": "3.3V to 6V",
        "interface": "1-Wire (proprietary protocol)",
        "power_consumption": "0.5 mA (measuring), 0.1 mA (idle)",
        "usage": "Low-cost weather stations, home automation, IoT",
        "signal_type": "Digital",
        "references": ["https://www.adafruit.com/product/385"],
        "tutorials": [
            "DHT22 with Arduino: https://www.arduino.cc/en/Tutorial/LibraryExamples/DHTSensorLibrary",
            "DHT22 with ESP32: https://randomnerdtutorials.com/esp32-dht11-dht22-temperature-humidity-sensor-arduino-ide/",
            "DHT22 with STM32: https://controllerstech.com/temperature-measurement-using-dht22-in-stm32/"
        ],
        "image": "/static/images/dht22.jpg"
    },
    "ds18b20": {
        "type": "Digital Temperature Sensor",
        "temperature_range": "-55°C to +125°C",
        "temperature_accuracy": "±0.5°C (-10°C to +85°C)",
        "power_supply": "3V to 5.5V",
        "interface": "1-Wire",
        "power_consumption": "1 mA (active), 0.75 µA (sleep)",
        "usage": "Digital thermometers, temperature control, embedded projects",
        "signal_type": "Digital",
        "references": ["https://www.maximintegrated.com/en/products/sensors/DS18B20.html"],
        "tutorials": [
            "DS18B20 with Arduino: https://randomnerdtutorials.com/guide-for-ds18b20-temperature-sensor-with-arduino/",
            "DS18B20 with ESP32: https://randomnerdtutorials.com/esp32-ds18b20-temperature-sensor-arduino-ide/",
            "DS18B20 with STM32: https://controllerstech.com/ds18b20-with-stm32/"
        ],
        "image": "/static/images/ds18b20.jpg"
    },
    "mpu6050": {
        "type": "Accelerometer and Gyroscope Sensor",
        "measurements": ["3-axis acceleration", "3-axis gyroscope"],
        "acceleration_range": "±2g to ±16g",
        "gyro_range": "±250°/s to ±2000°/s",
        "power_supply": "3V to 5V",
        "interfaces": ["I2C"],
        "power_consumption": "3.9 mA (active), 10 µA (sleep)",
        "usage": "Motion detection, robotics, drones, gesture recognition in IoT",
        "signal_type": "Digital",
        "references": ["https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/"],
        "tutorials": [
            "MPU6050 with Arduino: https://randomnerdtutorials.com/arduino-mpu-6050-gyroscope-accelerometer-temperature/",
            "MPU6050 with ESP32: https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/",
            "MPU6050 with STM32: https://controllerstech.com/mpu6050-gyro-accelerometer-with-stm32/"
        ],
        "image": "/static/images/mpu6050.jpg"
    },
    "hc-sr04": {
        "type": "Ultrasonic Distance Sensor",
        "measurement_range": "2cm to 400cm",
        "accuracy": "±3mm",
        "power_supply": "5V",
        "interfaces": ["Digital (trigger/echo pins)"],
        "power_consumption": "15 mA (active)",
        "usage": "Distance measurement, obstacle avoidance in robotics, level sensing in IoT",
        "signal_type": "Digital",
        "references": ["https://www.sparkfun.com/products/15569"],
        "tutorials": [
            "HC-SR04 with Arduino: https://www.arduino.cc/en/Tutorial/LibraryExamples/UltrasonicSensor",
            "HC-SR04 with ESP32: https://randomnerdtutorials.com/esp32-hc-sr04-ultrasonic-arduino/",
            "HC-SR04 with STM32: https://controllerstech.com/hc-sr04-ultrasonic-with-stm32/"
        ],
        "image": "/static/images/hc-sr04.jpg"
    },
    "pir": {
        "type": "Passive Infrared Motion Sensor",
        "detection_range": "Up to 7m",
        "field_of_view": "110° x 70°",
        "power_supply": "3V to 5V",
        "interfaces": ["Digital output"],
        "power_consumption": "65 µA (idle)",
        "usage": "Motion detection for security, automation, energy management in IoT",
        "signal_type": "Digital",
        "references": ["https://www.adafruit.com/product/189"],
        "tutorials": [
            "PIR with Arduino: https://www.arduino.cc/en/Tutorial/LibraryExamples/PIRSensor",
            "PIR with ESP32: https://randomnerdtutorials.com/esp32-pir-motion-sensor-interrupts-timers/",
            "PIR with STM32: https://controllerstech.com/pir-sensor-with-stm32/"
        ],
        "image": "/static/images/pir.jpg"
    },
    "soil moisture": {
        "type": "Soil Moisture Sensor",
        "measurement": "Soil moisture level (analog/digital)",
        "power_supply": "3.3V to 5V",
        "interfaces": ["Analog/Digital"],
        "power_consumption": "5 mA (active)",
        "usage": "Smart agriculture, plant monitoring, irrigation systems in IoT",
        "signal_type": "Analog/Digital",
        "references": ["https://www.sparkfun.com/products/13322"],
        "tutorials": [
            "Soil Moisture with Arduino: https://www.arduino.cc/en/Tutorial/LibraryExamples/SoilMoistureSensor",
            "Soil Moisture with ESP32: https://randomnerdtutorials.com/esp32-soil-moisture-sensor/",
            "Soil Moisture with STM32: https://controllerstech.com/soil-moisture-sensor-with-stm32/"
        ],
        "image": "/static/images/soil_moisture.jpg"
    },
    "ldr": {
        "type": "Light Dependent Resistor (Photoresistor)",
        "measurement": "Light intensity (resistance changes with light)",
        "range": "1 lux to 100k lux",
        "power_supply": "3V to 5V (with voltage divider)",
        "interfaces": ["Analog"],
        "power_consumption": "Low (passive component)",
        "usage": "Light detection, automatic lighting, environmental monitoring in IoT",
        "signal_type": "Analog",
        "references": ["https://www.adafruit.com/product/161"],
        "tutorials": [
            "LDR with Arduino: https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial",
            "LDR with ESP32: https://randomnerdtutorials.com/esp32-ldr-photoresistor/",
            "LDR with STM32: https://controllerstech.com/ldr-with-stm32-adc/"
        ],
        "image": "/static/images/ldr.jpg"
    }
}

# ===================== Environnements =====================
ENVIRONNEMENTS = {
    "VS Code": {
        "description": "Lightweight and powerful IDE for Python, C, C++, MicroPython, and embedded development.",
        "usage": "Code editing, debugging, microcontroller extensions.",
        "image": "/static/images/vscode.jpg"
    },
    "Python": {
        "description": "Interpreted language for scripting, IoT, and data processing.",
        "usage": "Automation, MicroPython for embedded systems.",
        "image": "/static/images/python.jpg"
    },
    "C": {
        "description": "Low-level language for system and embedded programming.",
        "usage": "Firmware, microcontrollers, performance optimization.",
        
    },
    "C++": {
        "description": "Object-oriented extension of C for complex applications.",
        "usage": "Advanced embedded systems, desktop, gaming.",
        "image": "/static/images/cpp.jpg"
    },
    "Java": {
        "description": "Platform-independent object-oriented language.",
        "usage": "Web applications, Android, backend development.",
        "image": "/static/images/java.png"
    },
    "HTML": {
        "description": "Markup language for web page structure.",
        "usage": "Web page structure, front-end integration.",
        "image": "/static/images/html.jpg"
    },
    "CSS": {
        "description": "Styling language for web design.",
        "usage": "Responsive design, front-end animations.",
        
    },
    "Django": {
        "description": "Python framework for rapid and secure web development.",
        "usage": "Web applications, REST APIs, dynamic websites.",
        "image": "/static/images/django.jpg"
    }
}

# ===================== Réseaux IoT et Gateways =====================
RESEAUX_IOT ={
    "lora": {
        "type": "Long-Range Low-Power Wireless Communication",
        "description": "Proprietary radio technology for low-data-rate, long-range IoT applications.",
        "usage": "Remote sensors, smart agriculture, home automation, LPWAN networks.",
        "frequencies": ["868 MHz (EU)", "915 MHz (US)", "433 MHz (Asia)"],
        "protocols": ["LoRaWAN (optional for structured networks)"],
        "references": ["https://www.semtech.com/lora/what-is-lora"],
        "image": "/static/images/lora.jpg"
    },
    "lorawan": {
        "type": "LoRa-Based Network Protocol",
        "description": "Manages LoRa communications between nodes and gateways, with security and Class A/B/C support.",
        "usage": "LPWAN networks for IoT, remote sensors, smart cities.",
        "classes": ["Class A (low power, bi-directional)", "Class B (scheduled receive slots)", "Class C (continuous receive)"],
        "security": "AES-128 encryption for messages",
        "references": ["https://lora-alliance.org/about-lorawan/"],
        "image": "/static/images/lorawan.jpg"
    },
    "sigfox": {
        "type": "Proprietary LPWAN Network",
        "description": "Low-power transmission of small data packets over long distances.",
        "usage": "Low-data-rate IoT, asset tracking, remote sensors.",
        "frequencies": ["868 MHz (EU)", "902 MHz (US)"],
        "references": ["https://www.sigfox.com/en/what-sigfox"],
        
    },
    "nb-iot": {
        "type": "Cellular IoT Network",
        "description": "3GPP LPWAN technology using LTE bands for IoT applications.",
        "usage": "Connected sensors, smart meters, smart cities.",
        "advantages": ["Extended coverage", "Low power consumption", "LTE standard support"],
        "references": ["https://www.gsma.com/iot/narrowband-iot-nb-iot/"],
       
    },
    "spi": {
        "type": "Communication Protocol",
        "description": "Serial Peripheral Interface, a synchronous serial communication interface for short-distance, high-speed communication.",
        "usage": "Interfacing microcontrollers with sensors, displays, and memory devices like SD cards or LoRa modules.",
        "characteristics": ["Full-duplex", "Master-slave architecture", "4-wire (MISO, MOSI, SCK, SS)"],
        "references": ["https://www.analog.com/en/technical-articles/spi-interface.html"],
        
    },
    "i2c": {
        "type": "Communication Protocol",
        "description": "Inter-Integrated Circuit, a two-wire serial bus for short-range communication between devices.",
        "usage": "Connecting multiple sensors (e.g., BME680, MPU6050) to microcontrollers with minimal pins.",
        "characteristics": ["Half-duplex", "Master-slave", "SDA (data) and SCL (clock) lines", "Supports multiple devices on same bus"],
        "references": ["https://www.nxp.com/docs/en/user-guide/UM10204.pdf"],
        
    },
   "CAN": {
                "description": "Controller Area Network 2.0A/B for robust, real-time communication, support dual CAN.",
                "usage": "Automotive, industrial control, inter-device communication, error handling (TEC/REC counters).",
                "references": ["https://www.st.com/resource/en/application_note/an2606.pdf", "STM32 RM0090 Reference Manual Section 30"],
                "details_electroniques": "2 instances bxCAN, bitrate up to 1 Mbps, 3 TX mailboxes, 14 filters, loopback/silent modes, requires transceiver externe (ex: MCP2551 pour 5V bus, isolation galvanique recommandée avec optocouplers). Timings: propagation delay <250ns, bus length up to 40m @1Mbps. Pull-up on CAN_H/CAN_L si besoin.",
                "registres": {
                    "CAN_MCR": "Master Control Register - Bits: INRQ (init request), SLEEP (sleep mode), AWUM (auto wakeup), ABOM (auto bus-off management), TTCM (time triggered), DBF (debug freeze). Ex: CAN_MCR = (1 << 0) pour init mode.",
                    "CAN_MSR": "Master Status Register - Bits: INAK (init ack), SLAK (sleep ack), ERR1 (error interrupt), WKUI (wakeup), SLAKI (sleep interrupt).",
                    "CAN_TSR": "Transmit Status Register - Bits pour chaque mailbox: RQCP (request completed), TXOK (transmission OK), ALST (arbitration lost), TERR (transmission error).",
                    "CAN_RF0R/CAN_RF1R": "Receive FIFO Registers - FMP (messages pending), FULL, FOVR (overrun).",
                    "CAN_IER": "Interrupt Enable Register - TMEIE (TX mailbox empty), FMPIE (FIFO message pending), ERRIE (error).",
                    "CAN_ESR": "Error Status Register - EWGF (warning), EPVF (passive), BOFF (bus-off), LEC (last error code: stuff/bit/dom/ack/CRC).",
                    "CAN_BTR": "Bit Timing Register - BRP (prescaler 1-1024), TS1/TS2 (time segments), SJW (synchronization jump), LBKM (loopback), SILM (silent). Ex: pour 1Mbps avec 8MHz APB1: BRP=1, TS1=11, TS2=4, SJW=1.",
                    "CAN_TI0R/CAN_TDT0R/CAN_TDL0R/CAN_TDH0R": "TX Mailbox Registers - ID, RTR, IDE, DLC, data bytes.",
                    "CAN_FA1R/CAN_FMR": "Filter Registers - 14 banks, mask/identifier modes, scale 16/32 bits.",
                    "notes": "Configurer clock APB1 à 42MHz max pour CAN (diviseur /2), enable interruptions NVIC pour CAN1_TX_IRQn etc. Dépannage: vérifier transceiver, terminaison 120Ω aux extrémités du bus, oscillo pour signaux différentiels (~2.5V common mode)."
             }
           },     
    "dragino lg308": {
        "type": "LoRaWAN Gateway",
        "description": "8-channel indoor LoRaWAN gateway with WiFi and Ethernet connectivity.",
        "specifications": {
            "chipset": "Semtech SX1301 (LoRa concentrator)",
            "channels": "8 concurrent channels",
            "frequency_bands": ["868 MHz (EU), 915 MHz (US), 923 MHz (AS923), etc."],
            "power_supply": "12V DC or PoE (IEEE 802.3af)",
            "connectivity": ["WiFi 802.11 b/g/n", "Ethernet", "Optional 4G via USB dongle"],
            "protocols": ["LoRaWAN 1.0.2", "MQTT", "UDP", "TCP"],
            "processing": "400 MHz MIPS processor, 128 MB DDR RAM",
            "software": "OpenWrt-based firmware, supports packet forwarder",
            "usage": "Indoor IoT deployments, smart buildings, industrial monitoring."
        },
        "references": ["https://www.dragino.com/products/lorawan-gateway/item/140-lg308.html"],
        "image": "/static/images/dragino_lg308.jpg"
    },
    "flybox": {
        "type": "LoRaWAN Gateway",
        "description": "Compact LoRaWAN gateway with WiFi, 4G, and MQTT support for IoT applications.",
        "specifications": {
            "chipset": "Semtech SX1302 (LoRa concentrator)",
            "channels": "8 concurrent channels",
            "frequency_bands": ["868 MHz (EU), 915 MHz (US)"],
            "power_supply": "5V USB-C or battery backup",
            "connectivity": ["WiFi 802.11 b/g/n/ac", "4G LTE", "Ethernet"],
            "protocols": ["LoRaWAN 1.0.3", "MQTT", "HTTP/HTTPS"],
            "processing": "Quad-core ARM Cortex-A53, 1 GB RAM",
            "software": "Custom Linux-based firmware, supports TTN and private LoRaWAN servers",
            "usage": "Mobile IoT deployments, smart cities, temporary setups."
        },
        "references": ["https://www.orange.com/en/products-and-services/iot/flybox"],
        "image": "/static/images/flybox.jpg"
    }
}

# ===================== Cloud Platforms =====================
CLOUD_PLATFORMS = {
    "thingspeak": {
        "type": "IoT Analytics Platform",
        "description": "An IoT analytics platform service that allows you to aggregate, visualize, and analyze live data streams in the cloud. Supports MATLAB integration for advanced analysis.",
        "usage": "Sensor data collection, visualization, alerting, and integration with IoT devices like ESP32 or Arduino.",
        "features": ["Data aggregation", "Real-time visualization", "MATLAB analysis", "Plugins and apps"],
        "references": ["https://thingspeak.mathworks.com/"],
    },
    " things network": {
        "type": "LoRaWAN Public Network",
        "description": "A global, open LoRaWAN network that enables low-power devices to communicate over long ranges. Provides gateways, network servers, and application integration.",
        "usage": "Building LoRaWAN-based IoT applications, smart cities, asset tracking, and community-driven networks.",
        "features": ["Decentralized network", "Integration with gateways", "Console for management", "Support for end-to-end encryption"],
        "references": ["https://www.thethingsnetwork.org/"],
        "image": "/static/images/ttn.jpg"
    },
    "chirpstack": {
        "type": "Open-Source LoRaWAN Network Server",
        "description": "An open-source LoRaWAN network server stack for managing gateways, devices, and data forwarding. Supports private or public deployments.",
        "usage": "Setting up custom LoRaWAN networks, device management, live frame logging, and integration with external services like MQTT.",
        "features": ["Web interface for management", "Multi-tenant support", "API for integrations", "Gateway OS for embedded devices"],
        "references": ["https://www.chirpstack.io/"],
        
    },
    "aws iot core": {
        "type": "Managed IoT Cloud Service",
        "description": "A managed cloud platform that lets connected devices easily and securely interact with cloud applications and other devices. Supports billions of devices and trillions of messages.",
        "usage": "Secure device connectivity, data processing, integration with AWS services like Lambda, S3, and Machine Learning.",
        "features": ["Device gateway", "Rules engine", "Device shadows", "Over-the-air updates", "Security and authentication"],
        "references": ["https://aws.amazon.com/iot-core/"]
    
    },
    "azure iot hub": {
        "type": "Managed IoT Cloud Service",
        "description": "A cloud-hosted solution for connecting, monitoring, and managing IoT devices at scale. Integrates with other Azure services for analytics and AI.",
        "usage": "Device provisioning, telemetry ingestion, command and control, and edge computing.",
        "features": ["Bi-directional communication", "Device twins", "Message routing", "Integration with Azure ML and Power BI"],
        "references": ["https://azure.microsoft.com/en-us/products/iot-hub/"],
        
    },
    "google cloud iot": {
        "type": "Managed IoT Cloud Service",
        "description": "A fully managed service for securely connecting, managing, and ingesting data from globally dispersed devices. Integrates with Google Cloud's data analytics tools.",
        "usage": "Large-scale device management, real-time data processing, and AI/ML integration.",
        "features": ["Device registry", "Telemetry events", "State synchronization", "Integration with Pub/Sub and BigQuery"],
        "references": ["https://cloud.google.com/iot-core"],
        
    }
}

# ===================== Exemples de Code =====================
CODE_EXAMPLES = {
    "led stm32": {
        "language": "C (STM32 HAL)",
        "code": """
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Assuming LED on PA5
    HAL_Delay(1000);
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
""",
        "description": "Blinks an LED on STM32F407VG using HAL library. Assumes LED connected to PA5."
    },
    "led esp32": {
        "language": "C++ (Arduino IDE)",
        "code": """
#define LED_PIN 2  // Built-in LED on GPIO2

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
""",
        "description": "Blinks the built-in LED on ESP32 using Arduino framework."
    },
    "led arduino": {
        "language": "C++ (Arduino IDE)",
        "code": """
#define LED_PIN 13  // Built-in LED on pin 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
""",
        "description": "Blinks the built-in LED on Arduino Uno/Mega."
    },
    "dht22 arduino": {
        "language": "C++ (Arduino IDE)",
        "code": """
#include <DHT.h>

#define DHT_PIN 2
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\\tTemperature: ");
  Serial.print(t);
  Serial.println(" *C");
  delay(2000);
}
""",
        "description": "Reads temperature and humidity from DHT22 on Arduino."
    },
    "dht22 esp32": {
        "language": "C++ (Arduino IDE)",
        "code": """
#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));
  dht.begin();
}

void loop() {
  delay(2000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));
}
""",
        "description": "Reads temperature and humidity from DHT22 on ESP32 using Arduino framework."
    },
    "dht22 stm32": {
        "language": "C (STM32 HAL)",
        "code": """
#include "main.h"
#include "stdio.h"

#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_1

void delay(uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while ((__HAL_TIM_GET_COUNTER(&htim6)) < time);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_Start(void)
{
    Set_Pin_Output(DHT22_PORT, DHT22_PIN);
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET);
    HAL_Delay(1200);
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
    delay(20);
    Set_Pin_Input(DHT22_PORT, DHT22_PIN);
}

uint8_t DHT22_Check_Response(void)
{
    Set_Pin_Input(DHT22_PORT, DHT22_PIN);
    uint8_t Response = 0;
    delay(40);
    if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)))
    {
        delay(80);
        if ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) Response = 1;
        else Response = -1;
    }
    while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));
    return Response;
}

uint8_t DHT22_Read(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        while (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));
        delay(40);
        if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)))
        {
            i &= ~(1 << (7 - j));
        }
        else i |= (1 << (7 - j));
        while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));
    }
    return i;
}

void Display_Temp(float Temp)
{
    char str[20] = {0};
    lcd_put_cur(0, 0);
    sprintf(str, "TEMP:- %.2f ", Temp);
    lcd_send_string(str);
    lcd_send_data('C');
}

void Display_Rh(float Rh)
{
    char str[20] = {0};
    lcd_put_cur(1, 0);
    sprintf(str, "RH:- %.2f ", Rh);
    lcd_send_string(str);
    lcd_send_data('%');
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM6_Init();
    MX_I2C1_Init();
    lcd_init();

    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
    uint16_t sum, RH, TEMP;
    float Temperature = 0;
    float Humidity = 0;
    uint8_t Presence = 0;

    while (1)
    {
        DHT22_Start();
        Presence = DHT22_Check_Response();
        Rh_byte1 = DHT22_Read();
        Rh_byte2 = DHT22_Read();
        Temp_byte1 = DHT22_Read();
        Temp_byte2 = DHT22_Read();
        sum = DHT22_Read();

        if (sum == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2))
        {
            TEMP = ((Temp_byte1 << 8) | Temp_byte2);
            RH = ((Rh_byte1 << 8) | Rh_byte2);
            Temperature = (float) TEMP / 10.0;
            Humidity = (float) RH / 10.0;
            Display_Temp(Temperature);
            Display_Rh(Humidity);
        }
        HAL_Delay(1000);
    }
}
""",
        "description": "Reads temperature and humidity from DHT22 on STM32 using HAL library. Assumes TIM6 for microsecond delay and LCD for display."
    },
    "bme680 stm32": {
        "language": "C (STM32 HAL)",
        "code": """
#include <bme680/bme68x_necessary_functions.h>

struct bme68x_data data;
bme68x_start(&data, &hi2c1);

if (bme68x_single_measure(&data) == 0) {
    data.iaq_score = bme68x_iaq();
    HAL_Delay(2000);
}

data.temperature
data.pressure
data.humidity
data.gas_resistance
data.iaq_score
""",
        "description": "Example for reading data from BME680 on STM32 using HAL and BME68x library."
    }
}

# ===================== Connection Details =====================
CONNECTION_DETAILS = {
    "stm32f407vg avec bme680": {
        "board": "stm32f407vg",
        "sensor": "bme680",
        "interface": "I2C",
        "connection": {
            "description": "Connect BME680 to STM32F407VG using I2C interface.",
            "pins": {
                "BME680_SDA": "STM32_PB9 (I2C1_SDA)",
                "BME680_SCL": "STM32_PB8 (I2C1_SCL)",
                "BME680_VCC": "3.3V",
                "BME680_GND": "GND"
            },
            "notes": [
                "Enable I2C1 in STM32CubeMX and configure PB8/PB9 as I2C pins.",
                "Use 4.7kΩ pull-up resistors on SDA and SCL lines if not present on BME680 module.",
                "Ensure BME680 library (e.g., BME68x) is included in the project."
            ],
            "code_example": CODE_EXAMPLES["bme680 stm32"]
        }
    },
    "esp32 avec bme680": {
        "board": "esp32",
        "sensor": "bme680",
        "interface": "I2C",
        "connection": {
            "description": "Connect BME680 to ESP32 using I2C interface.",
            "pins": {
                "BME680_SDA": "ESP32_GPIO21 (SDA)",
                "BME680_SCL": "ESP32_GPIO22 (SCL)",
                "BME680_VCC": "3.3V",
                "BME680_GND": "GND"
            },
            "notes": [
                "Default I2C pins on ESP32 are GPIO21 (SDA) and GPIO22 (SCL).",
                "Use Adafruit BME680 library for Arduino framework.",
                "Ensure pull-up resistors (4.7kΩ) on SDA/SCL if not included on BME680 module."
            ],
        }

               
        }
    }


# ===================== Fonctions supplémentaires =====================
def generate_board_comparison():
    """Generate a comparative table for microcontrollers and single-board computers."""
    comparison = {
        "boards": [],
        "fields": [
            "type", "cpu", "ram", "flash", "power_supply", "power_consumption", "gpio", "protocols"
        ]
    }
    for key, board in CARTE_INFO.items():
        board_data = {
            "name": key,
            "type": board.get("type", ""),
            "cpu": board.get("cpu", ""),
            "ram": board.get("ram", ""),
            "flash": board.get("flash", ""),
            "power_supply": board.get("power_supply", ""),
            "power_consumption": board.get("power_consumption", ""),
            "gpio": board.get("gpio", ""),
            "protocols": ", ".join(board.get("protocols_detail", {}).keys())
        }
    ["boards"].append(board_data)
    return comparison

def generate_protocol_comparison():
    """Generate a comparative table for communication protocols."""
    protocols = ["lora", "lorawan", "sigfox", "nb-iot", "spi", "i2c", "can", "WiFi", "Bluetooth BLE", "UART"]
    comparison = {
        "protocols": [],
        "fields": [
            "type", "description", "usage", "range", "data_rate", "power_efficiency"
        ]
    }
    for protocol in protocols:
        protocol_data = RESEAUX_IOT.get(protocol, CARTE_INFO["esp32"]["protocols_detail"].get(protocol, {}))
        if not protocol_data:
            continue
        proto_info = {
            "name": protocol,
            "type": protocol_data.get("type", "Communication Protocol"),
            "description": protocol_data.get("description", ""),
            "usage": protocol_data.get("usage", ""),
            "range": protocol_data.get("range", "N/A"),
            "data_rate": protocol_data.get("data_rate", "N/A"),
            "power_efficiency": protocol_data.get("power_efficiency", "N/A")
        }
        # Add specific details for known protocols
        if protocol == "lora":
            proto_info.update({"range": "2-15 km", "data_rate": "0.3-50 kbps", "power_efficiency": "High"})
        elif protocol == "lorawan":
            proto_info.update({"range": "2-15 km", "data_rate": "0.3-50 kbps", "power_efficiency": "High"})
        elif protocol == "sigfox":
            proto_info.update({"range": "10-50 km", "data_rate": "100 bps", "power_efficiency": "Very High"})
        elif protocol == "nb-iot":
            proto_info.update({"range": "1-10 km", "data_rate": "20-250 kbps", "power_efficiency": "High"})
        elif protocol == "spi":
            proto_info.update({"range": "Short (cm-m)", "data_rate": "Up to 10 Mbps", "power_efficiency": "Moderate"})
        elif protocol == "i2c":
            proto_info.update({"range": "Short (cm-m)", "data_rate": "100-400 kbps", "power_efficiency": "Moderate"})
        elif protocol == "can":
            proto_info.update({"range": "Up to 40m", "data_rate": "Up to 1 Mbps", "power_efficiency": "Moderate"})
        elif protocol == "WiFi":
            proto_info.update({"range": "50-100m", "data_rate": "Up to 300 Mbps", "power_efficiency": "Low"})
        elif protocol == "Bluetooth BLE":
            proto_info.update({"range": "10-100m", "data_rate": "1-2 Mbps", "power_efficiency": "High"})
        elif protocol == "UART":
            proto_info.update({"range": "Short (m)", "data_rate": "Up to 115.2 kbps", "power_efficiency": "Moderate"})
        comparison["protocols"].append(proto_info)
    return comparison

# ===================== Vues =====================
def bot_view(request):
    """Renders the chatbot HTML page"""
    return render(request, "bot/chat.html")

def chat_view(request):
    """API that returns JSON info based on the question"""
    question = request.GET.get("question", "").strip().lower()

    if not question:
        return JsonResponse({"error": "No question provided"}, status=400)

    # 1️⃣ Handle greetings
    if question in ["hello", "salut", "bonjour", "hi"]:
        return JsonResponse({"response": "Hello, how can I assist you with embedded systems or IoT?"})

    # 2️⃣ Handle specific question: difference between lora and lorawan
    if "difference between lora and lorawan" in question:
        response = {
            "response": "LoRa is the physical layer modulation technology developed by Semtech for long-range, low-power wireless communication. LoRaWAN is a Media Access Control (MAC) layer protocol built on top of LoRa, defining the network architecture, device classes (A/B/C), security, and how devices communicate with gateways and network servers."
        }
        return JsonResponse(response, safe=True)

    # 3️⃣ Handle code examples
    code_key = None
    clean_question = question.replace("code", "").strip().lower()
    for key in CODE_EXAMPLES.keys():
        if all(word in clean_question for word in key.split()):
            code_key = key
            break
    if code_key:
        return JsonResponse({"code_example": CODE_EXAMPLES[code_key]}, safe=True)

    # 4️⃣ Handle comparative table requests
    if "compare boards" in question or "board comparison" in question:
        return JsonResponse(generate_board_comparison(), safe=True)

    if "compare protocols" in question or "protocol comparison" in question:
        return JsonResponse(generate_protocol_comparison(), safe=True)

    # 5️⃣ Handle connection details
    connection_key = None
    for key in CONNECTION_DETAILS.keys():
        if all(word in question for word in key.split("_")):
            connection_key = key
            break
    if connection_key:
        return JsonResponse({"connection_details": CONNECTION_DETAILS[connection_key]}, safe=True)

    # 6️⃣ Direct lookup (microcontrollers and modules)
    if question in CARTE_INFO:
        return JsonResponse(CARTE_INFO[question], safe=True)

    # 7️⃣ Direct lookup (sensors)
    if question in CAPTEUR_INFO:
        return JsonResponse(CAPTEUR_INFO[question], safe=True)

    # 8️⃣ Direct lookup (environments, IoT networks/gateways, or cloud platforms)
    for key, val in ENVIRONNEMENTS.items():
        if question == key.lower():
            return JsonResponse({key: val}, safe=True)
    for key, val in RESEAUX_IOT.items():
        if question == key.lower():
            return JsonResponse({key: val}, safe=True)
    for key, val in CLOUD_PLATFORMS.items():
        if question == key.lower():
            return JsonResponse({key: val}, safe=True)

    # 9️⃣ Search by language, IDE, or protocol
    results = {}
    for key, carte in CARTE_INFO.items():
        if isinstance(carte, dict):
            langages = [l.lower() for l in carte.get("languages", []) if isinstance(l, str)]
            ides = [i.lower() for i in carte.get("ide", []) if isinstance(i, str)]
            protocols = [p.lower() for p in carte.get("protocols_detail", {}).keys() if isinstance(p, str)]
            if question in langages or question in ides or question in protocols:
                results[key] = carte

    # 10️⃣ Search for gateway-specific queries (e.g., MQTT, WiFi, Bluetooth)
    for key, gateway in RESEAUX_IOT.items():
        if isinstance(gateway, dict) and "specifications" in gateway:
            protocols = [p.lower() for p in gateway.get("specifications", {}).get("protocols", [])]
            connectivity = [c.lower() for c in gateway.get("specifications", {}).get("connectivity", [])]
            if question in protocols or question in connectivity:
                results[key] = gateway

    # 11️⃣ Search for cloud platforms
    for key, cloud in CLOUD_PLATFORMS.items():
        if question in key.lower() or key.lower() in question:
            results[key] = cloud

    if results:
        return JsonResponse(results, safe=True)

    # 12️⃣ Handle specific embedded systems queries
    if "gateway" in question:
        gateway_results = {k: v for k, v in RESEAUX_IOT.items() if "gateway" in v.get("type", "").lower()}
        if gateway_results:
            return JsonResponse(gateway_results, safe=True)

    if "cloud" in question or "iot platform" in question:
        return JsonResponse(CLOUD_PLATFORMS, safe=True)

    # 13️⃣ Error if no match found
    return JsonResponse({"error": "Microcontroller, sensor, environment, gateway, cloud platform, code example, or connection not found"}, status=404)