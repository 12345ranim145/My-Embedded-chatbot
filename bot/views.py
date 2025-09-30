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
        },
        "languages": ["C", "C++", "Python (MicroPython)", "Arduino IDE"],
        "ide": ["VS Code", "Arduino IDE", "PlatformIO", "Espressif IDF"],
        "image": "/static/images/esp32.jpg",
        "firmware_development": {
            "toolchain": "ESP-IDF (Espressif IoT Development Framework), FreeRTOS support",
            "debugging": "JTAG debugging, UART logging",
            "power_optimization": "Deep sleep, ULP coprocessor for low-power tasks"
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
        },
        "languages": ["C", "C++", "Python (MicroPython)", "Arduino IDE"],
        "ide": ["VS Code", "Arduino IDE", "PlatformIO", "Espressif IDF"],
        "image": "/static/images/esp32_cam.jpg",
        "firmware_development": {
            "toolchain": "ESP-IDF, Arduino framework",
            "debugging": "UART-based debugging",
            "camera": "OV2640 (2MP), supports MJPEG streaming"
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
                "description": "Robust communication bus for automotive and industrial applications.",
                "usage": "Inter-microcontroller communication, vehicle networks.",
                "references": ["https://www.st.com/resource/en/application_note/an2606.pdf"]
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
            "UART/SPI/I2C": {
                "description": "Standard communication for sensors and actuators.",
                "usage": "Board interconnection, simple IoT.",
                "references": ["https://www.arduino.cc/en/Guide/ArduinoUno"]
            },
            "1-Wire": {
                "description": "Simple protocol for sensors like DS18B20.",
                "usage": "Temperature measurement, unique sensor addressing.",
                "references": ["https://www.maximintegrated.com/en/products/1-wire.html"]
            }
        },
        "languages": ["C", "C++", "Arduino IDE"],
        "ide": ["Arduino IDE", "VS Code + PlatformIO"],
        "image": "/static/images/arduino_uno.jpg",
        "firmware_development": {
            "toolchain": "AVR-GCC, Arduino framework",
            "debugging": "Serial monitor, limited debugging support",
            "power_optimization": "Sleep modes via AVR libraries"
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
            "UART/SPI/I2C": {
                "description": "Standard communication for sensors and actuators.",
                "usage": "Board interconnection, advanced IoT.",
                "references": ["https://www.arduino.cc/en/Guide/ArduinoMega2560"]
            }
        },
        "languages": ["C", "C++", "Arduino IDE"],
        "ide": ["Arduino IDE", "VS Code + PlatformIO"],
        "image": "/static/images/arduino_mega.jpg",
        "firmware_development": {
            "toolchain": "AVR-GCC, Arduino framework",
            "debugging": "Serial monitor, limited debugging support",
            "power_optimization": "Sleep modes via AVR libraries"
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
        "references": ["https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/"],
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
        "references": ["https://www.adafruit.com/product/385"],
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
        "references": ["https://www.maximintegrated.com/en/products/sensors/DS18B20.html"],
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
        "references": ["https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/"],
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
        "references": ["https://www.sparkfun.com/products/15569"],
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
        "references": ["https://www.adafruit.com/product/189"],
        "image": "/static/images/pir.jpg"
    },
    "soil moisture": {
        "type": "Soil Moisture Sensor",
        "measurement": "Soil moisture level (analog/digital)",
        "power_supply": "3.3V to 5V",
        "interfaces": ["Analog/Digital"],
        "power_consumption": "5 mA (active)",
        "usage": "Smart agriculture, plant monitoring, irrigation systems in IoT",
        "references": ["https://www.sparkfun.com/products/13322"],
        "image": "/static/images/soil_moisture.jpg"
    },
    "ldr": {
        "type": "Light Dependent Resistor (Photoresistor)",
        "measurement": "Light intensity (resistance changes with light)",
        "range": "1 lux to 100k lux",
        "power_supply": "3V to 5V (with voltage divider)",
        "interfaces": ["Analog"],
        "power_consumption": "Low (passive component)" ,
        "usage": "Light detection, automatic lighting, environmental monitoring in IoT",
        "references": ["https://www.adafruit.com/product/161"],
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
        "image": "/static/images/c.jpg"
    },
    "C++": {
        "description": "Object-oriented extension of C for complex applications.",
        "usage": "Advanced embedded systems, desktop, gaming.",
        "image": "/static/images/cpp.jpg"
    },
    "Java": {
        "description": "Platform-independent object-oriented language.",
        "usage": "Web applications, Android, backend development.",
        "image": "/static/images/java.jpg"
    },
    "HTML": {
        "description": "Markup language for web page structure.",
        "usage": "Web page structure, front-end integration.",
        "image": "/static/images/html.jpg"
    },
    "CSS": {
        "description": "Styling language for web design.",
        "usage": "Responsive design, front-end animations.",
        "image": "/static/images/css.jpg"
    },
    "Django": {
        "description": "Python framework for rapid and secure web development.",
        "usage": "Web applications, REST APIs, dynamic websites.",
        "image": "/static/images/django.jpg"
    }
}

# ===================== Réseaux IoT et Gateways =====================
RESEAUX_IOT = {
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
        "image": "/static/images/sigfox.jpg"
    },
    "nb-iot": {
        "type": "Cellular IoT Network",
        "description": "3GPP LPWAN technology using LTE bands for IoT applications.",
        "usage": "Connected sensors, smart meters, smart cities.",
        "advantages": ["Extended coverage", "Low power consumption", "LTE standard support"],
        "references": ["https://www.gsma.com/iot/narrowband-iot-nb-iot/"],
        "image": "/static/images/nb-iot.jpg"
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
        "type": "LoRaWAN Gateway (Orange Flybox)",
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
        "image": "/static/images/thingspeak.jpg"
    },
    "the things network": {
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
        "image": "/static/images/chirpstack.jpg"
    },
    "aws iot core": {
        "type": "Managed IoT Cloud Service",
        "description": "A managed cloud platform that lets connected devices easily and securely interact with cloud applications and other devices. Supports billions of devices and trillions of messages.",
        "usage": "Secure device connectivity, data processing, integration with AWS services like Lambda, S3, and Machine Learning.",
        "features": ["Device gateway", "Rules engine", "Device shadows", "Over-the-air updates", "Security and authentication"],
        "references": ["https://aws.amazon.com/iot-core/"],
        "image": "/static/images/aws_iot.jpg"
    },
    "azure iot hub": {
        "type": "Managed IoT Cloud Service",
        "description": "A cloud-hosted solution for connecting, monitoring, and managing IoT devices at scale. Integrates with other Azure services for analytics and AI.",
        "usage": "Device provisioning, telemetry ingestion, command and control, and edge computing.",
        "features": ["Bi-directional communication", "Device twins", "Message routing", "Integration with Azure ML and Power BI"],
        "references": ["https://azure.microsoft.com/en-us/products/iot-hub/"],
        "image": "/static/images/azure_iot.jpg"
    },
    "google cloud iot": {
        "type": "Managed IoT Cloud Service",
        "description": "A fully managed service for securely connecting, managing, and ingesting data from globally dispersed devices. Integrates with Google Cloud's data analytics tools.",
        "usage": "Large-scale device management, real-time data processing, and AI/ML integration.",
        "features": ["Device registry", "Telemetry events", "State synchronization", "Integration with Pub/Sub and BigQuery"],
        "references": ["https://cloud.google.com/iot-core"],
        "image": "/static/images/google_iot.jpg"
    }
}

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

    # 3️⃣ Direct lookup (microcontrollers)
    if question in CARTE_INFO:
        return JsonResponse(CARTE_INFO[question], safe=True)

    # 4️⃣ Direct lookup (sensors)
    if question in CAPTEUR_INFO:
        return JsonResponse(CAPTEUR_INFO[question], safe=True)

    # 5️⃣ Direct lookup (environments, IoT networks/gateways, or cloud platforms)
    for key, val in ENVIRONNEMENTS.items():
        if question == key.lower():
            return JsonResponse({key: val}, safe=True)
    for key, val in RESEAUX_IOT.items():
        if question == key.lower():
            return JsonResponse({key: val}, safe=True)
    for key, val in CLOUD_PLATFORMS.items():
        if question == key.lower():
            return JsonResponse({key: val}, safe=True)

    # 6️⃣ Search by language, IDE, or protocol
    results = {}
    for key, carte in CARTE_INFO.items():
        if isinstance(carte, dict):
            langages = [l.lower() for l in carte.get("languages", []) if isinstance(l, str)]
            ides = [i.lower() for i in carte.get("ide", []) if isinstance(i, str)]
            protocols = [p.lower() for p in carte.get("protocols_detail", {}).keys() if isinstance(p, str)]
            if question in langages or question in ides or question in protocols:
                results[key] = carte

    # 7️⃣ Search for gateway-specific queries (e.g., MQTT, WiFi, Bluetooth)
    for key, gateway in RESEAUX_IOT.items():
        if isinstance(gateway, dict) and "specifications" in gateway:
            protocols = [p.lower() for p in gateway.get("specifications", {}).get("protocols", [])]
            connectivity = [c.lower() for c in gateway.get("specifications", {}).get("connectivity", [])]
            if question in protocols or question in connectivity:
                results[key] = gateway

    # 8️⃣ Search for cloud platforms (e.g., if question contains cloud service names or 'cloud')
    for key, cloud in CLOUD_PLATFORMS.items():
        if question in key.lower() or key.lower() in question:
            results[key] = cloud

    if results:
        return JsonResponse(results, safe=True)

    # 9️⃣ Handle specific embedded systems queries
    if "gateway" in question:
        gateway_results = {k: v for k, v in RESEAUX_IOT.items() if "gateway" in v.get("type", "").lower()}
        if gateway_results:
            return JsonResponse(gateway_results, safe=True)

    if "cloud" in question or "iot platform" in question:
        return JsonResponse(CLOUD_PLATFORMS, safe=True)

    # 10️⃣ Error if no match found
    return JsonResponse({"error": "Microcontroller, sensor, environment, gateway, or cloud platform not found"}, status=404)