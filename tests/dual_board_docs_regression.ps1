$ErrorActionPreference = 'Stop'

function Assert-Regex {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if (-not [regex]::IsMatch($Text, $Pattern, [System.Text.RegularExpressions.RegexOptions]::Multiline)) {
        throw $Message
    }
}

function Assert-NoRegex {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ([regex]::IsMatch($Text, $Pattern, [System.Text.RegularExpressions.RegexOptions]::Multiline)) {
        throw $Message
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$gimbalAppDocPath = Join-Path $repoRoot 'Gimbal\application\application.md'
$chassisAppDocPath = Join-Path $repoRoot 'Chassis\application\application.md'

$gimbalAppDocText = Get-Content $gimbalAppDocPath -Raw
$chassisAppDocText = Get-Content $chassisAppDocPath -Raw

Assert-NoRegex $gimbalAppDocText 'ONE_BOARD' 'Gimbal application doc must not advertise the removed ONE_BOARD mode.'
Assert-NoRegex $chassisAppDocText 'ONE_BOARD' 'Chassis application doc must not advertise the removed ONE_BOARD mode.'

Assert-Regex $gimbalAppDocText '固定.*GIMBAL_BOARD|双板专用|严格双板' 'Gimbal application doc must state that this worktree is fixed to the gimbal-board role.'
Assert-Regex $chassisAppDocText '固定.*CHASSIS_BOARD|双板专用|严格双板' 'Chassis application doc must state that this worktree is fixed to the chassis-board role.'

Write-Output 'PASS: dual-board docs regression checks'
