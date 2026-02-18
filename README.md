# Sound Localization using I2S and DMA

This project implements a real-time sound localization system using an array of **3 MEMS microphones** and an **STM32F446RE**. It uses Time Difference of Arrival and Cross-Correlation to calculate the angle of incoming sound.

## How it Works

The system captures audio from 3 microphones simultaneously. When a sound is detected, it calculates the time delay between the microphones to determine direction.

* **Hardware Sync:** Uses a single I2S Master Clock to drive all 3 microphones, ensuring sample-perfect synchronization between the "Left" (Mic 1) and "Right" (Mic 2) channels.
* **DMA Capture:** Utilizes Circular DMA buffers to capture 24-bit audio @ 44.1kHz without blocking the CPU.
* **DSP Processing:**
    * **Noise Gate:** Ignores background noise until a volume threshold is breached.
    * **Cross-Correlation:** `R[k] = sum(Mic1[i] * Mic2[i+k])` finds the exact sample lag (`k`) where the signals align best.
    * **Triangulation:** Converts the sample lag into a physical angle using the speed of sound (343m/s).

## Pinout Configuration

The system is configured for the **Nucleo-F446RE**.

| Signal | STM32 Pin | Function | Connection |
| :--- | :--- | :--- | :--- |
| **SCK** | **PB10** | Master Clock | Connect to SCK on **ALL 3 Mics** + **PC10** |
| **WS** | **PB12** | Word Select | Connect to WS on **ALL 3 Mics** + **PA4** |
| **Data 1** | **PC1** | I2S2 SD | Connect to SD of **Mic 1 & 2** |
| **Data 2** | **PC12** | I2S3 SD | Connect to SD of **Mic 3** |

> **Note:** I2S2 is the Master, and I2S3 is the Slave. The SCK/WS lines must be physically bridged between the two peripherals.

## How to Run

1.  Clone this repository.
2.  Open the folder in **STM32CubeIDE**.
3.  Open `SoundLoc.ioc` if you need to modify the pinout.
4.  **Build** and **Flash** to a Nucleo-F446RE.
5.  Open a Serial Monitor at **115200 baud** to see the angle output.
<img width="834" height="728" alt="Screenshot 2026-02-17 222334" src="https://github.com/user-attachments/assets/ea2fbecb-7873-402c-9b4d-8c4d5d5b60ee" />

