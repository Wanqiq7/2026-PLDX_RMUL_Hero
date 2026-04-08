$ErrorActionPreference = "Stop"

function Assert-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if (-not [regex]::IsMatch($Text, $Pattern, [System.Text.RegularExpressions.RegexOptions]::Multiline)) {
        throw $Message
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$gitignorePath = Join-Path $repoRoot ".gitignore"
$hardGatePath = Join-Path $repoRoot "tests\\debug_build_hard_gate.ps1"

$gitignoreText = [System.IO.File]::ReadAllText($gitignorePath, [System.Text.Encoding]::GetEncoding(936))
$hardGateText = Get-Content $hardGatePath -Raw

if (-not $gitignoreText.Contains("**/.hard-gate-build/")) {
    throw ".gitignore must ignore the .hard-gate-build runtime artifact root."
}
if (-not $hardGateText.Contains("Hard gate")) {
    throw "debug_build_hard_gate.ps1 must document the runtime artifact retention policy."
}

if (-not $hardGateText.Contains('$HardGateKeepLatestOnStart = 4')) {
    throw "debug_build_hard_gate.ps1 must freeze the startup retention count."
}

if (-not $hardGateText.Contains('$HardGateKeepLatestAfterRun = 5')) {
    throw "debug_build_hard_gate.ps1 must freeze the post-run retention count."
}

if (-not $hardGateText.Contains('Prune-HardGateArtifacts -RootPath $hardGateRoot -KeepLatest $HardGateKeepLatestOnStart')) {
    throw "Hard gate script must use the named startup retention constant."
}

if (-not $hardGateText.Contains('Prune-HardGateArtifacts -RootPath $hardGateRoot -KeepLatest $HardGateKeepLatestAfterRun')) {
    throw "Hard gate script must use the named post-run retention constant."
}

Write-Output "PASS: hard gate artifact policy regression checks"
