$ErrorActionPreference = 'Stop'

function Assert-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if (-not [regex]::IsMatch($Text, $Pattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)) {
        throw $Message
    }
}

function Assert-NoPattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ([regex]::IsMatch($Text, $Pattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)) {
        throw $Message
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$hardGateScript = Join-Path $repoRoot 'tests\debug_build_hard_gate.ps1'
$scriptText = Get-Content $hardGateScript -Raw

Assert-Pattern $scriptText 'function\s+Prune-HardGateArtifacts' 'debug_build_hard_gate is missing artifact-retention helper.'
Assert-Pattern $scriptText 'Prune-HardGateArtifacts\s+-RootPath\s+\$hardGateRoot' 'debug_build_hard_gate is not pruning retained hard-gate artifacts.'
Assert-NoPattern $scriptText 'Remove-StaleHardGateArtifacts\s+-RootPath\s+\$hardGateRoot' 'debug_build_hard_gate still deletes all stale artifacts before each run.'

Write-Output 'PASS: debug build hard gate retention regression checks'
