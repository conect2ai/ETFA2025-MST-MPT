&nbsp;
&nbsp;
<p align="center">
  <img width="800" src="./figures/conecta_logo.png" />
</p> 

&nbsp;

# MST and MPT: Lightweight Incremental Algorithms for Multivariate Anomaly Detection and Correction on TinyML Devices

### ✍🏾Authors: [Morsinaldo Medeiros](https://github.com/Morsinaldo), [Marianne Diniz](https://github.com/MarianneDiniz), [Ivanovitch Silva](https://github.com/ivanovitchm), [Massimiliano Gaffurini](https://scholar.google.com.br/citations?user=s6ZkzYYAAAAJ&hl=en&authuser=1&oi=ao), [Dennis Brandão](https://scholar.google.com.br/citations?user=OxSKwvEAAAAJ&hl=en&authuser=1), [Paolo Ferrari](https://scholar.google.com.br/citations?user=-BIQbXMAAAAJ&hl=en&authuser=1)

## 1. Abstract/Overview

The Internet of Things (IoT) generates massive multivariate time series data that requires real-time anomaly detection and correction to ensure reliable monitoring and control. However, conventional methods relying on offline training and batch processing are impractical for embedded IoT devices due to their severe memory, processing, and energy constraints. To address this gap, we propose two lightweight incremental learning algorithms: Multivariate Sequential TEDA with RLS Correction (MST) and Multivariate Parallel TEDA with RLS (MPT), both derived from the TEDARLS framework, which enable on-device detection and correction of multivariate anomalies within the constraints of TinyML resources. Both algorithms operate without prior training, continuously updating internal statistics in real-time to identify outliers and apply immediate corrections. In experiments using real vehicular sensor data on a TinyML microcontroller, MST achieved approximately 99% precision and 100% recall with inference times around 500μs, while preserving the original data characteristics. MPT, although more sensitive to multivariate patterns, introduced greater signal distortions and required processing times exceeding 4,200μs. Overall, MST demonstrated superior stability and is more suitable for real-time anomaly correction in resource-constrained IoT environments. By enabling lightweight, accurate, and autonomous anomaly detection and correction directly at the edge, this approach addresses a critical gap in embedded analytics for IoT systems.

![Python](https://img.shields.io/badge/Python-3.11%2B-blue)
![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange)
![Edge%20AI](https://img.shields.io/badge/Edge%20AI-Ready-green)

This repository contains the source code and experimental materials associated with the MST (Multivariate Sequential TEDA-RLS) and MPT (Multivariate Parallel TEDA-RLS) algorithms, developed for online anomaly detection and correction in multivariate vehicular data streams.

Both algorithms are designed to operate in real time, making them suitable for edge computing scenarios involving vehicular telemetry, such as OBD-II data acquisition.

## Repository Structure 📂

- **`./Freematics/`** — Source code from the Freematics project used for the embedded experiments (ESP32, OBD-II communication, etc.).
- **`./src/`** — Source code developed for MST and MPT.
  - **`./src/cpp/`** — C++ implementations for embedded systems.
- **`./data/`** — Datasets used for experiments, including preprocessed vehicular data.
- **`./figures/`** — Figures generated for analysis and publication.
- **`.git/`** — Version control metadata (Git).

## Datasets 📈

All experimental datasets are provided in `.csv` format within the `data/` folder.  
Each file contains the following variables:

| Variable | Description |
|:--------:|:-----------|
| `speed` | Vehicle speed (km/h) |
| `rpm` | Engine revolutions per minute (rpm) |
| `tp` | Throttle position (%) |
| `load` | Engine load (%) |
| `timing` | Ignition timing advance (°) |

The datasets are organized as follows:
- **Original data** — Vehicular signals without injected anomalies.
- **Corrected data** — Data after processing with MST or MPT.

## Getting Started 🚀

### 1️⃣ Cloning the Repository
```bash
git clone https://github.com/conect2ai/ETFA2025-MST-MPT.git
cd ETFA2025-MST-MPT
```

### 2️⃣ Installing Dependencies
It is recommended to use Python 3.11+ with a virtual environment (venv or conda):

```bash
pip install -r requirements.txt
```

### 3️⃣ Running the Notebooks
Open the `.ipynb` files using Jupyter Notebook or JupyterLab and execute the cells according to the instructions provided in each section.

## Executing on Freematics One+

### 4️⃣ Opening the Project
Open the project folder `./Freematics/firmware_v5/telelogger` on PlatformIO, as illustrated in the figure below.

<p align="center">
  <img width="800" src="./figures/platformio.png" />
</p>

### 5️⃣ Connecting the Device
Connect the Freematics One+ to your computer and turn it on using the Freematics Emulator or inside the vehicle.

### 6️⃣ Compiling, Uploading, and Monitoring
Compile, upload, and monitor the serial (steps 1, 2, and 3, respectively, in the figure below).

<p align="center">
  <img width="800" src="./figures/upload.png" />
</p>

---

## :page_facing_up: License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

--- 

# 🌎 About Conect2AI

The [**Conect2AI**](http://conect2ai.dca.ufrn.br) research group is composed of undergraduate and graduate students from the Federal University of Rio Grande do Norte (UFRN) and aims to apply Artificial Intelligence (AI) and Machine Learning in emerging areas. Our expertise includes Embedded Intelligence and IoT, optimizing resource management and energy efficiency, contributing to sustainable cities. In the energy transition and mobility, we apply AI to optimize energy use in connected vehicles and promote more sustainable mobility.
