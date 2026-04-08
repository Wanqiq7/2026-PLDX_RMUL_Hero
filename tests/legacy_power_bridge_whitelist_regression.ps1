$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$appRoots = @(
    (Join-Path $repoRoot 'Gimbal\application'),
    (Join-Path $repoRoot 'Chassis\application')
)

$violations = @()

foreach ($root in $appRoots) {
    $files = Get-ChildItem $root -Recurse -File -Filter *.c
    foreach ($file in $files) {
        $matches = Select-String -Path $file.FullName -Pattern 'LegacyPowerBridge'
        foreach ($match in $matches) {
            $violations += "$($file.FullName):$($match.LineNumber): $($match.Line.Trim())"
        }
    }
}

if ($violations.Count -gt 0) {
    $message = "Unexpected LegacyPowerBridge usage in application layer:`n" + ($violations -join "`n")
    throw $message
}

Write-Output 'PASS: LegacyPowerBridge whitelist regression checks'
