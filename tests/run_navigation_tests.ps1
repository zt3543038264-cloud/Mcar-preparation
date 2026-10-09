param([string]$Gcc = 'C:\Program Files (x86)\cpeditor\mingw64\bin\gcc.exe')

$ErrorActionPreference = 'Stop'
$testProject = Split-Path $PSScriptRoot -Parent
$testOutput = Join-Path (Split-Path $testProject -Parent) 'tmp\motor-check'
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
Push-Location $testProject
try {
    function Invoke-NavigationHostTest([string]$name, [string[]]$sources, [string[]]$compilerFlags = @()) {
        $testExecutable = Join-Path $testOutput ($name + '.exe')
        & $Gcc -std=c99 -Wall -Wextra -Werror -I tests/motor_stubs -I project/code `
            -I libraries/zf_driver -I libraries/zf_device @compilerFlags @sources -lm -o $testExecutable
        if ($LASTEXITCODE -ne 0) { throw "$name compilation failed" }
        & $testExecutable
        if ($LASTEXITCODE -ne 0) { throw "$name failed" }
    }
    Invoke-NavigationHostTest 'imu_navigation_mahony_test' @(
        'tests/imu_navigation_publish_test.c', 'project/code/imu.c', 'project/code/attitude.c')
    Invoke-NavigationHostTest 'imu_navigation_madgwick_test' @(
        'tests/imu_navigation_publish_test.c', 'project/code/imu.c', 'project/code/attitude.c') @('-DAHRS_METHOD=2')
    Invoke-NavigationHostTest 'wifi_disabled_test' @(
        'tests/wifi_disabled_test.c', 'project/code/wifispi.c') @('-DIMU_WIFI_ENABLED=0')
    Invoke-NavigationHostTest 'vofa_packet_test' @(
        'tests/vofa_packet_test.c', 'project/code/wifispi.c') @('-DIMU_WIFI_ENABLED=0', '-Wl,--wrap=wifispi_send_floats')
    Invoke-NavigationHostTest 'wifi_telemetry_test' @(
        'tests/wifi_telemetry_test.c', 'project/code/wifispi.c',
        'project/code/pid.c', 'project/code/PID_config.c')
    Invoke-NavigationHostTest 'navigation_fusion_test' @(
        'tests/navigation_fusion_test.c', 'project/code/navigation_fusion.c')
    Invoke-NavigationHostTest 'position_control_test' @(
        'tests/position_control_test.c', 'project/code/position_control.c')
    Invoke-NavigationHostTest 'motor_mapping_test' @(
        'tests/motor_mapping_test.c', 'project/code/Motor.c', 'project/code/PID.c',
        'project/code/PID_config.c', 'project/code/app_control.c',
        'project/code/app_navigation.c', 'project/code/navigation_fusion.c',
        'project/code/position_control.c')
    Invoke-NavigationHostTest 'menu_display_test' @(
        'tests/menu_display_test.c', 'project/code/Mymenu.c', 'project/code/menu.c',
        'project/code/PID_config.c')
} finally {
    Pop-Location
}
