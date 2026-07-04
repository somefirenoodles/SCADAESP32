$ErrorActionPreference = "Stop"

Write-Host "Opening ESP-IDF menuconfig..."
idf.py menuconfig
