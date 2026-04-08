$ErrorActionPreference = 'Stop'

function Get-IncludeEntries {
    param(
        [string]$RootPath
    )

    $pattern = '^\s*#include\s+"([^"]+)"'
    $files = Get-ChildItem -Path $RootPath -Recurse -File -Include *.c, *.h

    foreach ($file in $files) {
        $lineNumber = 0
        foreach ($line in Get-Content $file.FullName) {
            $lineNumber++
            $match = [regex]::Match($line, $pattern)
            if ($match.Success) {
                [PSCustomObject]@{
                    RelativePath = $file.FullName.Substring($RootPath.Length + 1).Replace('\', '/')
                    LineNumber   = $lineNumber
                    IncludePath  = $match.Groups[1].Value.Replace('\', '/')
                }
            }
        }
    }
}

function Assert-NoIncludeViolation {
    param(
        [pscustomobject]$Entry,
        [string]$Message
    )

    throw "$Message ($($Entry.RelativePath):$($Entry.LineNumber) -> $($Entry.IncludePath))"
}

function Get-AlgorithmHeaderBaseNames {
    param(
        [string[]]$AlgorithmRoots
    )

    $headerNameSet = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)

    foreach ($algorithmRoot in $AlgorithmRoots) {
        if (-not (Test-Path -LiteralPath $algorithmRoot)) {
            continue
        }

        Get-ChildItem -Path $algorithmRoot -Recurse -File -Filter *.h | ForEach-Object {
            [void]$headerNameSet.Add($_.Name)
        }
    }

    return @($headerNameSet)
}

function Get-LegacyTopLevelAlgorithmHeadersFromLayoutRegression {
    param(
        [string]$RepoRoot
    )

    $layoutRegressionPath = Join-Path $RepoRoot 'tests\algorithm_layer_layout_regression.ps1'
    $layoutRegressionText = Get-Content $layoutRegressionPath -Raw
    $legacyTopLevelMatch = [regex]::Match(
        $layoutRegressionText,
        '\$legacyTopLevel\s*=\s*@\((?<body>[\s\S]*?)\n\)',
        [System.Text.RegularExpressions.RegexOptions]::Multiline
    )

    if (-not $legacyTopLevelMatch.Success) {
        throw 'Missing $legacyTopLevel definition in algorithm_layer_layout_regression.ps1.'
    }

    return @(
        [regex]::Matches($legacyTopLevelMatch.Groups['body'].Value, "'([^']+\.h)'") |
        ForEach-Object { $_.Groups[1].Value }
    )
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$entries = Get-IncludeEntries -RootPath $repoRoot

$currentAlgorithmHeaderBaseNames = Get-AlgorithmHeaderBaseNames -AlgorithmRoots @(
    (Join-Path $repoRoot 'Chassis\modules\algorithm'),
    (Join-Path $repoRoot 'Gimbal\modules\algorithm')
)
$legacyTopLevelAlgorithmHeaders = Get-LegacyTopLevelAlgorithmHeadersFromLayoutRegression -RepoRoot $repoRoot
$algorithmHeaderNameSet = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)

foreach ($headerName in ($currentAlgorithmHeaderBaseNames + $legacyTopLevelAlgorithmHeaders)) {
    [void]$algorithmHeaderNameSet.Add($headerName)
}

$algorithmHeaderBaseNames = @($algorithmHeaderNameSet)

foreach ($requiredLegacyHeader in @(
        'controller.h',
        'QuaternionEKF.h',
        'kalman_filter.h',
        'crc8.h',
        'crc16.h',
        'user_lib.h',
        'arm_math_compat.h'
    )) {
    if ($algorithmHeaderBaseNames -notcontains $requiredLegacyHeader) {
        throw "Algorithm header coverage is incomplete: missing $requiredLegacyHeader in the application denylist."
    }
}

$publicHeaders = @(
    'Chassis/modules/power_controller/power_controller.h',
    'Chassis/modules/imu/ins_task.h',
    'Gimbal/modules/imu/ins_task.h'
)

foreach ($entry in $entries) {
    if ($entry.RelativePath -match '/modules/algorithm/utils/' -and $entry.IncludePath -match '^(controllers|estimation)/') {
        Assert-NoIncludeViolation $entry 'Include graph violation: utils layer must not include controllers/ or estimation/.'
    }

    if ($entry.RelativePath -match '/modules/algorithm/estimation/' -and $entry.IncludePath -match '^controllers/') {
        Assert-NoIncludeViolation $entry 'Include graph violation: estimation layer must not include controllers/.'
    }

    $includeBaseName = [System.IO.Path]::GetFileName($entry.IncludePath)
    $includeHasHierarchy = $entry.IncludePath.Contains('/')

    if ($entry.RelativePath -match '/application/' -and -not $includeHasHierarchy -and $algorithmHeaderBaseNames -contains $includeBaseName) {
        Assert-NoIncludeViolation $entry 'Include graph violation: application layer must not include algorithm headers via legacy top-level names.'
    }

    if ($entry.RelativePath -in $publicHeaders -and ($entry.IncludePath -match '^estimation/' -or $entry.IncludePath -eq 'BMI088driver.h')) {
        Assert-NoIncludeViolation $entry 'Include graph violation: designated public headers must not include internal estimation or driver headers.'
    }
}

Write-Output 'PASS: include graph regression checks'
