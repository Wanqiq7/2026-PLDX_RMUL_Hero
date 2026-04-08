$ErrorActionPreference = 'Stop'

function Assert-True {
    param(
        [bool]$Condition,
        [string]$Message
    )

    if (-not $Condition) {
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

$gimbalPrunedModules = @(
    'Gimbal\modules\bluetooth',
    'Gimbal\modules\encoder',
    'Gimbal\modules\ist8310',
    'Gimbal\modules\oled',
    'Gimbal\modules\standard_cmd',
    'Gimbal\modules\super_cap',
    'Gimbal\modules\TFminiPlus',
    'Gimbal\modules\unicomm'
)

$chassisPrunedModules = @(
    'Chassis\modules\bluetooth',
    'Chassis\modules\encoder',
    'Chassis\modules\ist8310',
    'Chassis\modules\oled',
    'Chassis\modules\remote',
    'Chassis\modules\standard_cmd',
    'Chassis\modules\TFminiPlus',
    'Chassis\modules\unicomm',
    'Chassis\modules\master_machine'
)

foreach ($relativePath in ($gimbalPrunedModules + $chassisPrunedModules)) {
    $fullPath = Join-Path $repoRoot $relativePath
    Assert-True (-not (Test-Path $fullPath)) "$relativePath must be removed once unused-module pruning is complete."
}

$gimbalMakefileText = Get-Content (Join-Path $repoRoot 'Gimbal\Makefile') -Raw
$chassisMakefileText = Get-Content (Join-Path $repoRoot 'Chassis\Makefile') -Raw

Assert-NoRegex $gimbalMakefileText 'modules/bluetooth/HC05\.c' 'Gimbal Makefile must not compile bluetooth after pruning.'
Assert-NoRegex $gimbalMakefileText 'modules/oled/oled\.c' 'Gimbal Makefile must not compile oled after pruning.'
Assert-NoRegex $gimbalMakefileText 'modules/ist8310/ist8310\.c' 'Gimbal Makefile must not compile ist8310 after pruning.'
Assert-NoRegex $gimbalMakefileText 'modules/standard_cmd/std_cmd\.c' 'Gimbal Makefile must not compile standard_cmd after pruning.'
Assert-NoRegex $gimbalMakefileText 'modules/super_cap/super_cap\.c' 'Gimbal Makefile must not compile super_cap after pruning.'
Assert-NoRegex $gimbalMakefileText 'modules/TFminiPlus/tfminiplus\.c' 'Gimbal Makefile must not compile TFminiPlus after pruning.'
Assert-NoRegex $gimbalMakefileText 'modules/unicomm/unicomm\.c' 'Gimbal Makefile must not compile unicomm after pruning.'
Assert-NoRegex $gimbalMakefileText '-Imodules/bluetooth' 'Gimbal Makefile must not include bluetooth headers after pruning.'
Assert-NoRegex $gimbalMakefileText '-Imodules/ist8310' 'Gimbal Makefile must not include ist8310 headers after pruning.'
Assert-NoRegex $gimbalMakefileText '-Imodules/oled' 'Gimbal Makefile must not include oled headers after pruning.'
Assert-NoRegex $gimbalMakefileText '-Imodules/standard_cmd' 'Gimbal Makefile must not include standard_cmd headers after pruning.'
Assert-NoRegex $gimbalMakefileText '-Imodules/super_cap' 'Gimbal Makefile must not include super_cap headers after pruning.'
Assert-NoRegex $gimbalMakefileText '-Imodules/TFminiPlus' 'Gimbal Makefile must not include TFminiPlus headers after pruning.'
Assert-NoRegex $gimbalMakefileText '-Imodules/unicomm' 'Gimbal Makefile must not include unicomm headers after pruning.'

Assert-NoRegex $chassisMakefileText 'modules/bluetooth/HC05\.c' 'Chassis Makefile must not compile bluetooth after pruning.'
Assert-NoRegex $chassisMakefileText 'modules/ist8310/ist8310\.c' 'Chassis Makefile must not compile ist8310 after pruning.'
Assert-NoRegex $chassisMakefileText 'modules/oled/oled\.c' 'Chassis Makefile must not compile oled after pruning.'
Assert-NoRegex $chassisMakefileText 'modules/remote/remote_control\.c' 'Chassis Makefile must not compile remote after pruning.'
Assert-NoRegex $chassisMakefileText 'modules/standard_cmd/std_cmd\.c' 'Chassis Makefile must not compile standard_cmd after pruning.'
Assert-NoRegex $chassisMakefileText 'modules/TFminiPlus/tfminiplus\.c' 'Chassis Makefile must not compile TFminiPlus after pruning.'
Assert-NoRegex $chassisMakefileText 'modules/unicomm/unicomm\.c' 'Chassis Makefile must not compile unicomm after pruning.'
Assert-NoRegex $chassisMakefileText 'modules/master_machine/master_process\.c' 'Chassis Makefile must not compile master_machine after pruning.'
Assert-NoRegex $chassisMakefileText 'modules/master_machine/seasky_protocol\.c' 'Chassis Makefile must not compile master_machine protocol after pruning.'
Assert-NoRegex $chassisMakefileText '-Imodules/bluetooth' 'Chassis Makefile must not include bluetooth headers after pruning.'
Assert-NoRegex $chassisMakefileText '-Imodules/ist8310' 'Chassis Makefile must not include ist8310 headers after pruning.'
Assert-NoRegex $chassisMakefileText '-Imodules/oled' 'Chassis Makefile must not include oled headers after pruning.'
Assert-NoRegex $chassisMakefileText '-Imodules/remote' 'Chassis Makefile must not include remote headers after pruning.'
Assert-NoRegex $chassisMakefileText '-Imodules/standard_cmd' 'Chassis Makefile must not include standard_cmd headers after pruning.'
Assert-NoRegex $chassisMakefileText '-Imodules/TFminiPlus' 'Chassis Makefile must not include TFminiPlus headers after pruning.'
Assert-NoRegex $chassisMakefileText '-Imodules/unicomm' 'Chassis Makefile must not include unicomm headers after pruning.'
Assert-NoRegex $chassisMakefileText '-Imodules/master_machine' 'Chassis Makefile must not include master_machine headers after pruning.'

Write-Output 'PASS: unused module prune regression checks'
