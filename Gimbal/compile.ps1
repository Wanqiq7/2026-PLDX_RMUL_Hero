$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$preset = if ($args.Count -ge 1) { $args[0] } else { "Debug" }
$buildJobs = if ($env:BUILD_JOBS) { $env:BUILD_JOBS } else { "24" }

if ($preset -notin @("Debug", "Release")) {
    throw "Unsupported preset '$preset'. Use Debug or Release."
}

Write-Host "--- Configuring project with CMake preset: $preset ---"
cmake --preset $preset -S $scriptDir

Write-Host "--- Building project with Ninja ($buildJobs jobs) ---"
cmake --build --preset $preset --parallel $buildJobs

Write-Host "--- Done! ---"
