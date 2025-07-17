# Automated Robotic System for Classification and Sorting Using Machine Vision

This project focused on integrating **computer vision**, **robotics**, and **embedded systems** to automate the sorting of products in a production environment. Inspired by real industrial challenges observed during an internship at FanMilk PLC, the system demonstrates how **AI-powered sorting** can optimize and modernize manufacturing lines.

## 🔍 Project Overview

The system detects and classifies products (*Milo* and *Chivita*) moving on a conveyor using a YOLOv11 object detection model, then sorts them with a 2-DOF robotic arm controlled by an ESP32 microcontroller.

### 🎯 Key Features

- **YOLOv11-based Object Detection**
- **Custom-built Conveyor System**
- **2-DOF Robotic Arm for Sorting**
- **ESP32 Microcontroller Integration**
- **Real-time Classification and Action**
- **Handcrafted Conveyor Frame (Woodwork)**

---

## ⚙️ How It Works

1. Products are placed on a **conveyor belt** (built from scratch with woodwork).
2. A **camera** captures an image as each product reaches the end.
3. The image is passed to a **YOLOv11 model** trained to classify between Milo and Chivita.
4. Classification results are sent to the **ESP32**, which then:
   - Activates the robotic arm.
   - Moves the product to its designated location based on class.

---

## 🛠️ Tech Stack & Tools

| Component           | Description                                 |
|---------------------|---------------------------------------------|
| **Python**          | YOLOv11 object detection and image handling |
| **ESP32**           | Microcontroller to control robotic actions  |
| **Servo Motors**    | Control arm movement and sorting            |
| **2-DOF Robotic Arm** | Handles sorting task based on classification |
| **OpenCV**          | Image capture and preprocessing             |
| **SolidWorks**      | Design of robotic arm parts                 |
| **Arduino IDE**     | Programming ESP32 microcontroller           |

---




