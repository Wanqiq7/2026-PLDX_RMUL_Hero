$ErrorActionPreference = 'Stop'

function Assert-Pattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if (-not (Select-String -Path $Path -Pattern $Pattern -Quiet)) {
        throw $Message
    }
}

function Assert-NoPattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if (Select-String -Path $Path -Pattern $Pattern -Quiet) {
        throw $Message
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$docs = @(
    (Join-Path $repoRoot 'Chassis\modules\motor\DJImotor\dji_motor.md'),
    (Join-Path $repoRoot 'Gimbal\modules\motor\DJImotor\dji_motor.md')
)

foreach ($doc in $docs) {
    Assert-Pattern $doc '### \u5f53\u524d\u4e3b\u7ebf' "Missing current-mainline section: $doc"
    Assert-Pattern $doc 'DJIMotorCalculateEffort\(\)' "Missing CalculateEffort mainline wording: $doc"
    Assert-Pattern $doc 'DJIMotorSetEffort\(\)' "Missing SetEffort mainline wording: $doc"
    Assert-Pattern $doc '### \u5386\u53f2\u517c\u5bb9\u8def\u5f84' "Missing legacy-compat section: $doc"
    Assert-NoPattern $doc '\u8981\u63a7\u5236\u4e00\u4e2aDJI\u7535\u673a\uff0c\u6211\u4eec\u63d0\u4f9b\u4e862\u4e2a\u63a5\u53e3' "Legacy 'two-interface' wording still appears in the main body: $doc"
    Assert-Pattern $doc '\u5386\u53f2\u517c\u5bb9\u63a5\u53e3' "SetRef is not demoted to compatibility wording: $doc"
    Assert-NoPattern $doc '\*\*`pid_ref`\u662f\u63a7\u5236\u7684\u8bbe\u5b9a\u503c\uff0capp\u5c42\u7684\u5e94\u7528\u60f3\u8981\u66f4\u6539\u7535\u673a\u7684\u8f93\u51fa\uff0c\u5c31\u8981\u8c03\u7528`DJIMotorSetRef\(\)`\u66f4\u6539\u6b64\u503c\u3002\*\*' "pid_ref/SetRef still described as the current app-level entry: $doc"
    Assert-NoPattern $doc '- `DJIMotorSetRef\(\)`\u662f\u8bbe\u5b9a\u7535\u673a\u8f93\u51fa\u7684\u63a5\u53e3' "SetRef still described as the default output interface: $doc"
}

Write-Output 'PASS: DJI motor doc mainline regression checks'
