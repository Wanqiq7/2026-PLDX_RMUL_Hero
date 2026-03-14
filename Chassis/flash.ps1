$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$preset = if ($args.Count -ge 1) { $args[0] } else { "Debug" }
$buildJobs = if ($env:BUILD_JOBS) { $env:BUILD_JOBS } else { "24" }
$openocdCfg = if ($env:OPENOCD_CFG) { $env:OPENOCD_CFG } else { "openocd_dap.cfg" }
$elfPath = Join-Path $scriptDir ("build\" + $preset + "\basic_framework.elf")

if ($preset -notin @("Debug", "Release")) {
    throw "Unsupported preset '$preset'. Use Debug or Release."
}

Write-Host "--- Configuring project with CMake preset: $preset ---"
cmake --preset $preset -S $scriptDir

Write-Host "--- Building project with Ninja ($buildJobs jobs) ---"
cmake --build --preset $preset --parallel $buildJobs

if (-not (Test-Path $elfPath)) {
    throw "ELF not found at $elfPath"
}

Write-Host "--- Flashing firmware ($elfPath) ---"
openocd -f $openocdCfg -c "program $elfPath verify reset exit"

Write-Host "--- Done! ---"
