from django.shortcuts import render
from django.http import JsonResponse
import json

# ===================== Base de données =====================
CARTE_INFO = {
    # ===================== ESP32 =====================
    "esp32": {
        "type": "Microcontrôleur",
        "alimentation": "2.7V à 3.6V (3.3V typique)",
        "consommation": "80 mA @ 240 MHz",
        "tension_logique_gpio": "3.3V",
        "courant_max_gpio": "12 mA par pin",
        "cpu": "Tensilica Xtensa dual-core, jusqu'à 240 MHz",
        "ram": "520 KB SRAM interne",
        "flash": "16 MB max externe (SPI)",
        "gpio": "34 pins digitales, ADC, DAC, PWM, I2C, SPI, UART, WiFi, Bluetooth",
        "usage": "IoT, domotique, projets connectés, capteurs embarqués, robotique légère",
        "protocoles_detail": {
            "WiFi": {
                "description": "Connexion réseau local et internet pour échanges de données.",
                "usage": "IoT, serveur web embarqué, mise à jour OTA.",
                "references": ["https://www.espressif.com/en/products/socs/esp32"]
            },
            "Bluetooth BLE": {
                "description": "Communication courte portée avec faible consommation.",
                "usage": "Capteurs sans fil, wearable, échanges point-à-point.",
                "references": ["https://www.bluetooth.com/specifications/"]
            },
            "MQTT": {
                "description": "Protocole léger publish/subscribe pour IoT.",
                "usage": "Communication entre capteurs et serveur ou cloud.",
                "references": ["https://mqtt.org/"]
            },
            "HTTP/HTTPS": {
                "description": "Protocole client-serveur standard pour requêtes web.",
                "usage": "API, échanges de données sécurisés.",
                "references": ["https://developer.mozilla.org/en-US/docs/Web/HTTP"]
            },
            "LoRa": {
                "description": "Communication longue portée basse consommation via modules externes (ex: SX1276).",
                "usage": "Réseaux LPWAN, capteurs distants, agriculture connectée.",
                "references": ["https://www.semtech.com/lora/what-is-lora"]
            }
        },
        "langages": ["C", "C++", "Python (MicroPython)", "Arduino IDE"],
        "ide": ["VS Code", "Arduino IDE", "PlatformIO", "Espressif IDF"],
        "image": "/static/images/esp32.jpg"
    },

    # ===================== ESP32-CAM =====================
    "esp32 cam": {
        "type": "Microcontrôleur + Caméra",
        "alimentation": "5V via USB ou Vin 5V",
        "consommation": "actif 160 mA caméra active",
        "cpu": "Tensilica Xtensa dual-core, jusqu'à 240 MHz",
        "ram": "520 KB SRAM interne",
        "flash": "4-16 MB",
        "gpio": "GPIO pour caméra et capteurs",
        "usage": "Caméra IoT, surveillance, streaming vidéo",
        "protocoles_detail": {
            "WiFi": {
                "description": "Connexion réseau local et internet.",
                "usage": "Streaming vidéo, serveur web",
                "references": ["https://www.espressif.com/en/products/socs/esp32"]
            },
            "HTTP": {
                "description": "Serveur web et requêtes API",
                "usage": "Streaming vidéo et contrôle caméra",
                "references": ["https://developer.mozilla.org/en-US/docs/Web/HTTP"]
            }
        },
        "langages": ["C", "C++", "Python (MicroPython)", "Arduino IDE"],
        "ide": ["VS Code", "Arduino IDE", "PlatformIO", "Espressif IDF"],
        "image": "/static/images/esp32_cam.jpg"
    },

    # ===================== STM32F407VG =====================
    "stm32f407vg": {
        "type": "Microcontrôleur ARM Cortex-M4",
        "alimentation": "1.8V à 3.6V (3.3V typique)",
        "cpu": "32-bit ARM Cortex-M4, 168 MHz",
        "ram": "192 KB SRAM",
        "flash": "1 MB",
        "gpio": "16 GPIO ports, ADC, DAC, PWM, UART, SPI, I2C, CAN",
        "usage": "Projets embarqués avancés, robotique, systèmes industriels",
        "protocoles_detail": {
            "CAN": {
                "description": "Bus de communication robuste utilisé en automobile et industrie.",
                "usage": "Communication entre microcontrôleurs.",
                "references": ["https://www.st.com/resource/en/application_note/an2606.pdf"]
            }
        },
        "langages": ["C", "C++", "Python via MicroPython", "HAL/LL STM32"],
        "ide": ["STM32CubeIDE", "VS Code", "Keil uVision", "CooCox CoIDE"],
        "frameworks": ["HAL", "FreeRTOS", "mbed OS"],
        "image": "/static/images/stm32f407vg.jpg",
        "tutoriels": [
            "STM32F4 Getting Started : https://deepbluembedded.com/stm32f4-discovery-board-tutorials/",
            "STM32 CAN Tutorial : https://controllerstech.com/stm32-can-tutorial/"
        ]
    },

    # ===================== Arduino Uno =====================
    "arduino uno": {
        "type": "Microcontrôleur AVR",
        "alimentation": "5V via USB ou Vin 7-12V",
        "consommation": {"actif": "50 mA", "sleep": "0.5 mA"},
        "cpu": "ATmega328P, 16 MHz",
        "ram": "2 KB SRAM",
        "flash": "32 KB",
        "gpio": "14 digital, 6 analogique, PWM, UART, SPI, I2C",
        "usage": "Prototypage, IoT simple, robots éducatifs",
        "protocoles_detail": {
            "UART/SPI/I2C": {
                "description": "Communication classique pour capteurs/actionneurs.",
                "usage": "Interconnexion cartes, IoT simple.",
                "references": ["https://www.arduino.cc/en/Guide/ArduinoUno"]
            },
            "1-Wire": {
                "description": "Protocole simple pour capteurs comme DS18B20.",
                "usage": "Mesure température, capteurs uniques.",
                "references": ["https://www.maximintegrated.com/en/products/1-wire.html"]
            }
        },
        "langages": ["C", "C++", "Arduino IDE"],
        "ide": ["Arduino IDE", "VS Code + PlatformIO"],
        "image": "/static/images/arduino_uno.jpg"
    },

    # ===================== Arduino Mega =====================
    "arduino mega": {
        "type": "Microcontrôleur AVR",
        "alimentation": "5V via USB ou Vin 7-12V",
        "consommation": {"actif": "70 mA", "sleep": "0.5 mA"},
        "cpu": "ATmega2560, 16 MHz",
        "ram": "8 KB SRAM",
        "flash": "256 KB",
        "gpio": "54 digital, 16 analogique, PWM, UART, SPI, I2C",
        "usage": "Projets complexes, robots, automation",
        "protocoles_detail": {
            "UART/SPI/I2C": {
                "description": "Communication classique pour capteurs/actionneurs.",
                "usage": "Interconnexion cartes, IoT avancé.",
                "references": ["https://www.arduino.cc/en/Guide/ArduinoMega2560"]
            }
        },
        "langages": ["C", "C++", "Arduino IDE"],
        "ide": ["Arduino IDE", "VS Code + PlatformIO"],
        "image": "/static/images/arduino_mega.jpg"
    },
}

# ===================== Capteurs =====================
CAPTEUR_INFO = {
    "bme680": {
        "type": "Capteur environnemental",
        "mesures": ["Température", "Humidité", "Pression", "Qualité de l'air (COV)"],
        "alimentation": "3.3V à 5V",
        "interfaces": ["I2C", "SPI"],
        "consommation": "0.15 mA (mode normal)",
        "usage": "Stations météo, domotique, suivi de la qualité de l’air",
        "references": ["https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors-bme680/"],
        "image": "/static/images/bme680.jpg"
    },
    "dht22": {
        "type": "Capteur de température et humidité",
        "plage_température": "-40°C à +80°C",
        "précision_température": "±0.5°C",
        "plage_humidité": "0–100% RH",
        "précision_humidité": "±2–5% RH",
        "alimentation": "3.3V à 6V",
        "interface": "1 fil (protocole propriétaire)",
        "usage": "Stations météo low-cost, domotique, IoT",
        "references": ["https://www.adafruit.com/product/385"],
        "image": "/static/images/dht22.jpg"
    },
    "ds18b20": {
        "type": "Capteur de température numérique",
        "plage_température": "-55°C à +125°C",
        "précision_température": "±0.5°C (entre -10°C et +85°C)",
        "alimentation": "3V à 5.5V",
        "interface": "1-Wire",
        "usage": "Thermomètres numériques, contrôle de température, projets embarqués",
        "references": ["https://www.maximintegrated.com/en/products/sensors/DS18B20.html"],
        "image": "/static/images/ds18b20.jpg"
    }
}

# ===================== Environnements =====================
ENVIRONNEMENTS = {
    "VS Code": {
        "description": "IDE léger et puissant pour Python, C, C++, MicroPython, embarqué.",
        "usage": "Édition de code, debug, extensions microcontrôleurs.",
        "image": "/static/images/vscode.jpg"
    },
    "Python": {
        "description": "Langage interprété simple pour scripting, IoT, data.",
        "usage": "Automatisation, MicroPython embarqué.",
        "image": "/static/images/python.jpg"
    },
    "C": {
        "description": "Langage bas niveau pour système et embarqué.",
        "usage": "Firmware, microcontrôleurs, optimisation.",
        "image": "/static/images/c.jpg"
    },
    "C++": {
        "description": "Extension orientée objet du C.",
        "usage": "Systèmes embarqués avancés, desktop, jeux.",
        "image": "/static/images/cpp.jpg"
    },
    "Java": {
        "description": "Langage orienté objet multiplateforme.",
        "usage": "Applications web, Android, backend.",
        "image": "/static/images/java.jpg"
    },
    "HTML": {
        "description": "Langage de balisage pour pages web.",
        "usage": "Structure de pages, intégration front-end.",
        "image": "/static/images/html.jpg"
    },
    "CSS": {
        "description": "Langage de style pour mise en page web.",
        "usage": "Design responsive, animations front-end.",
        "image": "/static/images/css.jpg"
    },
    "Django": {
        "description": "Framework Python pour développement web rapide et sécurisé.",
        "usage": "Applications web, APIs REST, sites dynamiques.",
        "image": "/static/images/django.jpg"
    },
}

RESEAUX_IOT = {
    "lora": {
        "type": "Communication sans fil longue portée basse consommation",
        "description": "Technologie radio propriétaire pour IoT, faible débit et longue portée.",
        "usage": "Capteurs distants, agriculture connectée, domotique, réseaux LPWAN.",
        "frequences": ["868 MHz (EU)", "915 MHz (US)", "433 MHz (ASIA)"],
        "protocoles": ["LoRaWAN optionnel pour réseau structuré"],
        "references": ["https://www.semtech.com/lora/what-is-lora"],
        "image": "/static/images/lora.jpg"
    },
    "lorawan": {
        "type": "Protocole réseau basé sur LoRa",
        "description": "Gestion des communications LoRa entre nœuds et gateway, avec sécurité et classes A/B/C.",
        "usage": "Réseaux LPWAN pour IoT, capteurs distants, smart city.",
        "classes": ["A", "B", "C"],
        "security": "AES-128 pour chiffrement des messages",
        "references": ["https://lora-alliance.org/about-lorawan/"],
        "image": "/static/images/lorawan.jpg"
    },
    "sigfox": {
        "type": "Réseau LPWAN propriétaire",
        "description": "Transmission de petites quantités de données à très faible consommation.",
        "usage": "IoT bas débit, suivi d'objets, capteurs distants.",
        "frequences": ["868 MHz (EU)", "902 MHz (US)"],
        "references": ["https://www.sigfox.com/en/what-sigfox"],
        "image": "/static/images/sigfox.jpg"
    },
    "nb-iot": {
        "type": "Réseau cellulaire IoT",
        "description": "Technologie 3GPP LPWAN utilisant les bandes LTE pour l'IoT.",
        "usage": "Capteurs connectés, smart meters, villes intelligentes.",
        "advantages": ["Couverture étendue", "Faible consommation", "Support standard LTE"],
        "references": ["https://www.gsma.com/iot/narrowband-iot-nb-iot/"],
        "image": "/static/images/nb-iot.jpg"
    }
}

# ===================== Vues =====================
def bot_view(request):
    """Affiche la page HTML du chatbot"""
    return render(request, "bot/chat.html")

def chat_view(request):
    """API qui retourne les infos JSON selon la question"""
    question = request.GET.get("question", "").strip().lower()

    if not question:
        return JsonResponse({"error": "Aucune question fournie"}, status=400)
    if question in ["hello", "salut", "bonjour", "hi"]:
        return JsonResponse({"response": "Hello Ranim, comment je peux t'aider ?"})
    # 1️⃣ Recherche directe (carte)
    if question in CARTE_INFO:
        return JsonResponse(CARTE_INFO[question], safe=True)

    # 2️⃣ Recherche directe (capteur)
    if question in CAPTEUR_INFO:
        return JsonResponse(CAPTEUR_INFO[question], safe=True)

    # 3️⃣ Recherche directe (environnement)
    for key, val in ENVIRONNEMENTS.items():
        if question == key.lower():
            return JsonResponse({key: val}, safe=True)
    for key, val in RESEAUX_IOT.items():
        if question == key.lower():
            return JsonResponse({key: val}, safe=True)
    # 4️⃣ Recherche par langages et IDE
    results = {}
    for key, carte in CARTE_INFO.items():
        if isinstance(carte, dict):
            langages = [l.lower() for l in carte.get("langages", []) if isinstance(l, str)]
            ides = [i.lower() for i in carte.get("ide", []) if isinstance(i, str)]
            if question in langages or question in ides:
                results[key] = carte

    if results:
        return JsonResponse(results, safe=True)

    # 5️⃣ Erreur sinon
    return JsonResponse({"error": "Carte, capteur ou environnement non trouvé"}, status=404)