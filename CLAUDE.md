# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Smart scale for continuously monitoring the weight of an LPG (GLP) gas cylinder using **ESP32 + HX711 + 4x 50kg load cells** (half-bridge). The system estimates remaining gas by compensating for mechanical, thermal, and electronic drift over time.

## Hardware Configuration

- **MCU:** ESP32
- **ADC:** HX711 (powered at 3.3V) — DT on GPIO19, SCK on GPIO18
- **Load cells:** 4x 50kg, wired in a Wheatstone bridge via junction board
- Wheatstone bridge wiring is documented in `Memoria_Projeto_Balanca_GLP.md`

## Firmware Architecture (V3 target)

Each version is delivered as a **single `.ino` file**. Changes are always made on top of the latest complete version — never isolated snippets.

**State machine:** STARTUP -> ESTABILIZACAO -> SEM_PESO -> CAPTURA_DE_PESO -> MONITORAMENTO

**Core model:** `RAW = Offset + Peso` — the traditional fixed-tare approach was abandoned in favor of continuously estimating the offset to compensate for long-term drift.

**Key variables:** `rawHX`, `offsetEstimado`, `pesoEstimado`, `pesoFiltrado`

**Signal processing features (evolved through V0-V2):**
- Moving average and standard deviation filtering
- Confidence index (multiple stable windows required before proceeding)
- Automatic tare after stabilization
- Zero tracking

## Build & Upload

This is an Arduino/ESP32 project. Build and upload using:
```bash
# Arduino CLI (if installed)
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-* .
```
Or use the Arduino IDE with ESP32 board support installed.

**Required library:** HX711 (e.g., `bogde/HX711`)

## Roadmap (from project memory)

1. Implement V3 complete firmware
2. Calibrate with known weight (514g reference weight available)
3. Convert RAW to kg
4. Convert kg to remaining GLP
5. OLED display
6. Wi-Fi connectivity
7. Consumption history
8. Autonomy estimation

## Language

Project documentation and variable names are in **Brazilian Portuguese**. Follow this convention.
