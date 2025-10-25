# Param([Parameter(Mandatory)][string]$adbConnection)

Set-Location -Path "$HOME\AppData\Local\Android\Sdk\platform-tools"

Write-Host "Restarting ADB connection..."
./adb kill-server
./adb start-server
./adb connect 192.168.43.1:5555
Write-Host "$(./adb devices)"
Write-Host "ADB connection restarted."