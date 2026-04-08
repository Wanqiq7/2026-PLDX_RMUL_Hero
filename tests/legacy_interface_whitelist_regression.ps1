$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$allowedFiles = @(
    (Join-Path $repoRoot 'Gimbal\application\sysid\sysid_task.c'),
    (Join-Path $repoRoot 'Gimbal\application\chassis\chassis.c')
)

$patterns = @(
    'DJIMotorSetRef\(',
    'DJIMotorSetRawRef\(',
    'DMMotorSetRef\(',
    'DMMotorSetMITTargets\(',
    'DMMotorSetMITTargetByProfile\(',
    'DMMotorSetPVT\('
)

$appRoots = @(
    (Join-Path $repoRoot 'Gimbal\application'),
    (Join-Path $repoRoot 'Chassis\application')
)

$violations = @()

foreach ($root in $appRoots) {
    $files = Get-ChildItem $root -Recurse -File -Filter *.c
    foreach ($file in $files) {
        foreach ($pattern in $patterns) {
            $matches = Select-String -Path $file.FullName -Pattern $pattern
            foreach ($match in $matches) {
                if ($allowedFiles -notcontains $file.FullName) {
                    $violations += "$($file.FullName):$($match.LineNumber): $($match.Line.Trim())"
                }
            }
        }
    }
}

if ($violations.Count -gt 0) {
    $message = "Unexpected legacy interface usage outside whitelist:`n" + ($violations -join "`n")
    throw $message
}

Write-Output 'PASS: legacy interface whitelist regression checks'
