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

Assert-Pattern $scriptText 'function\s+Stop-HardGateBuildProcesses' 'debug_build_hard_gate is missing stale process cleanup helper.'
Assert-Pattern $scriptText 'Get-CimInstance\s+Win32_Process' 'debug_build_hard_gate is not inspecting stale build processes.'
Assert-Pattern $scriptText 'try\s*\{[\s\S]*finally\s*\{' 'debug_build_hard_gate is missing try/finally cleanup.'
Assert-Pattern $scriptText 'Stop-HardGateBuildProcesses\s+-RepoRoot\s+\$repoRoot' 'debug_build_hard_gate is not cleaning stale hard-gate processes before build.'
Assert-Pattern $scriptText 'Prune-HardGateArtifacts\s+-RootPath\s+\$hardGateRoot' 'debug_build_hard_gate is not pruning retained hard-gate artifacts.'
Assert-NoPattern $scriptText 'cmd\.exe\s*/c' 'debug_build_hard_gate still shells out through cmd.exe and may orphan child processes.'

Write-Output 'PASS: debug build hard gate isolation regression checks'
