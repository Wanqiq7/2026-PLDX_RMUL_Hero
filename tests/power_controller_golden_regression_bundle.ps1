$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$commands = @(
    '.\tests\power_predict_equivalence_regression.ps1',
    '.\tests\energy_loop_equivalence_regression.ps1',
    '.\tests\rls_equivalence_regression.ps1',
    '.\tests\limited_output_equivalence_regression.ps1',
    '.\tests\power_predict_golden_regression.ps1',
    '.\tests\energy_loop_golden_regression.ps1',
    '.\tests\rls_golden_regression.ps1',
    '.\tests\limited_output_golden_regression.ps1'
)

Push-Location $repoRoot
try {
    foreach ($command in $commands) {
        Write-Host ">>> Running $command"
        & powershell.exe -ExecutionPolicy Bypass -File $command
        if ($LASTEXITCODE -ne 0) {
            throw "Regression bundle failed: $command"
        }
    }

    Write-Output 'PASS: power controller golden regression bundle'
}
finally {
    Pop-Location
}
