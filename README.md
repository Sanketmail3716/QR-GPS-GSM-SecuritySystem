# 🔐 QR-GPS-GSM Security System

This project is a **smart security and access control system** using **ESP32-CAM**. It integrates **QR code scanning**, **GPS tracking**, **GSM alerts**, and a **buzzer** for unauthorized access detection. Ideal for use in car security, smart gates, or restricted area access.

## 🚀 Features

- 📷 **QR Code scanning** for access control (via camera)
- 📡 **GPS module** for real-time location tracking
- 📲 **GSM (SIM800L)** for sending SMS alerts to the owner
- 🔔 **Buzzer alert** for unauthorized attempts
- 💡 **Flash LED** and switch-based alert trigger
- 🕒 Sends timestamped location in every alert

---

## 📁 Project Structure & File Description

| File Name                        | Description                                                                 |
|----------------------------------|-----------------------------------------------------------------------------|
| **QR_code_GPS_GSM_Message.ino** | Main Arduino sketch that integrates camera, GSM, GPS, QR scanner, and buzzer |
| **decode.c**                    | Part of the **quirc** QR code decoding library (decoding logic)             |
| **identify.c**                  | Handles QR code format identification (used by quirc)                       |
| **quirc.c**                     | Core logic for QR detection and processing                                  |
| **quirc.h**                     | Header file for QR code functions                                           |
| **quirc_internal.h**            | Internal definitions for the quirc library                                  |
| **version_db.c**                | QR version database for identifying QR code types and sizes                 |

---

## 🧰 Hardware Used

- ✅ ESP32-CAM (AI-Thinker)
- ✅ SIM800L GSM module
- ✅ GPS Module (e.g., NEO-6M)
- ✅ Buzzer & Push-button
- ✅ LEDs for feedback

---

## ⚙️ Setup Instructions

1. Connect modules as per pin mappings in `QR_code_GPS_GSM_Message.ino`.
2. Flash the code using Arduino IDE.
3. Power ESP32-CAM with 5V (use external power for GSM module).
4. Use a SIM with SMS plan and GPS with open sky view.
5. Scan a valid QR code (e.g., "Sanket") for access.
6. Use button to trigger manual alert request.

---

## 📬 SMS Examples

- On access granted:  
  ✅ `"Access granted, QR code scanned"`  
  📍 Location with timestamp sent via GPS

- On access denied:  
  ❌ `"Car theft alert! Unauthorized QR code scan"`  
  📍 Location with timestamp

---

## 📷 QR Code Processing

QR codes are captured by ESP32-CAM and decoded using the **quirc** library. If the scanned value matches a predefined string (e.g., "Sanket"), access is granted.

---

## 🛡️ Author

**Sanket Mali**  
🔗 [GitHub Profile](https://github.com/Sanketmail3716)

---

Let me know if you'd like to add wiring diagrams or images!
