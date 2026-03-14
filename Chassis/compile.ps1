$ErrorActionPreference = "Stop"

function Invoke-NativeOrThrow {
    param(
        [Parameter(Mandatory = $true)]
        [string[]] $Command
    )

    & $Command[0] $Command[1..($Command.Length - 1)]
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $($Command -join ' ')"
    }
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$preset = if ($args.Count -ge 1) { $args[0] } else { "Debug" }
$buildJobs = if ($env:BUILD_JOBS) { $env:BUILD_JOBS } else { "24" }

if ($preset -notin @("Debug", "Release")) {
    throw "Unsupported preset '$preset'. Use Debug or Release."
}

Write-Host "--- Configuring project with CMake preset: $preset ---"
Push-Location $scriptDir
try {
    Invoke-NativeOrThrow @("cmake", "--preset", $preset, "-S", $scriptDir)

    Write-Host "--- Building project with Ninja ($buildJobs jobs) ---"
    Invoke-NativeOrThrow @("cmake", "--build", "--preset", $preset, "--parallel", $buildJobs)

    Write-Host "--- Done! ---"
}
finally {
    Pop-Location
}
