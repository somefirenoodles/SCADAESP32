$ErrorActionPreference = "Stop"

Write-Host "Building ESP-IDF project..."
idf.py build
