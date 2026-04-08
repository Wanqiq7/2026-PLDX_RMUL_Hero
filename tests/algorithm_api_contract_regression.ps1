$ErrorActionPreference = 'Stop'

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
$chassisAlgDocPath = Join-Path $repoRoot 'Chassis\modules\algorithm\algorithm.md'
$gimbalAlgDocPath = Join-Path $repoRoot 'Gimbal\modules\algorithm\algorithm.md'
$sharedContractPath = Join-Path $repoRoot 'docs\superpowers\specs\2026-04-07-algorithm-layer-api-contract.md'

$chassisAlgDocText = Get-Content $chassisAlgDocPath -Raw
$gimbalAlgDocText = Get-Content $gimbalAlgDocPath -Raw
$sharedContractText = Get-Content $sharedContractPath -Raw

foreach ($text in @($chassisAlgDocText, $gimbalAlgDocText, $sharedContractText)) {
    Assert-Pattern $text 'controllers' 'Algorithm contract docs must mention the controllers layer.'
    Assert-Pattern $text 'estimation' 'Algorithm contract docs must mention the estimation layer.'
    Assert-Pattern $text 'utils' 'Algorithm contract docs must mention the utils layer.'
    Assert-Pattern $text 'controllers\s*->\s*estimation\s*->\s*utils' 'Algorithm contract docs must freeze the dependency direction.'
    Assert-Pattern $text '叶子目录.*public header|只能包含.*叶子目录' 'Algorithm contract docs must document the public-header entry rule.'
}

Assert-Pattern $gimbalAlgDocText 'Ref Manager / Arbiter' 'Gimbal algorithm doc must preserve the Ref Manager wording.'
Assert-Pattern $gimbalAlgDocText 'Adapter / Protocol' 'Gimbal algorithm doc must document that adapter/protocol stays outside the algorithm layer.'
Assert-Pattern $chassisAlgDocText 'rls_estimator\.h' 'Chassis algorithm doc must list the RLS estimator public entry.'
Assert-Pattern $sharedContractText '禁止事项' 'Shared algorithm contract doc must include explicit prohibited patterns.'

Write-Output 'PASS: algorithm api contract regression checks'
