$ErrorActionPreference = "Stop"

Write-Host "Cleaning ESP-IDF build output..."
idf.py fullclean
