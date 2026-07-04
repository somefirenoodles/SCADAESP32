param(
    [string]$Port = "COM17"
)

$ErrorActionPreference = "Stop"

Write-Host "Flashing and monitoring on $Port..."
idf.py -p $Port flash monitor
