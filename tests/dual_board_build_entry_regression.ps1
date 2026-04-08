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
$gimbalCompilePath = Join-Path $repoRoot 'Gimbal\compile.ps1'
$gimbalMakefilePath = Join-Path $repoRoot 'Gimbal\Makefile'
$chassisMakefilePath = Join-Path $repoRoot 'Chassis\Makefile'
$chassisRobotDefPath = Join-Path $repoRoot 'Chassis\application\robot_def.h'
$chassisCmakePath = Join-Path $repoRoot 'Chassis\CMakeLists.txt'

$gimbalCompileText = Get-Content $gimbalCompilePath -Raw
$gimbalMakefileText = Get-Content $gimbalMakefilePath -Raw
$chassisMakefileText = Get-Content $chassisMakefilePath -Raw
$chassisRobotDefText = Get-Content $chassisRobotDefPath -Raw
$chassisCmakeText = Get-Content $chassisCmakePath -Raw

Assert-Regex $gimbalCompileText 'function\s+Invoke-NativeOrThrow' 'Gimbal compile.ps1 must validate native command exit codes.'
Assert-Regex $gimbalCompileText 'Push-Location\s+\$scriptDir' 'Gimbal compile.ps1 must switch to the project directory before invoking presets.'
Assert-Regex $gimbalCompileText 'Pop-Location' 'Gimbal compile.ps1 must restore the previous working directory.'
Assert-NoRegex $gimbalCompileText '^[ \t]*cmake --preset' 'Gimbal compile.ps1 must not call cmake directly without exit-code checks.'
Assert-NoRegex $gimbalCompileText '^[ \t]*cmake --build' 'Gimbal compile.ps1 must not build directly without exit-code checks.'

Assert-NoRegex $gimbalMakefileText 'application/chassis/chassis\.c' 'Gimbal Makefile must not reference deleted chassis execution sources.'
Assert-NoRegex $gimbalMakefileText '-Iapplication/chassis' 'Gimbal Makefile must not include deleted chassis application headers.'

Assert-NoRegex $chassisMakefileText 'application/cmd/robot_cmd\.c' 'Chassis Makefile must not reference deleted cmd execution sources.'
Assert-NoRegex $chassisMakefileText 'application/gimbal/gimbal\.c' 'Chassis Makefile must not reference deleted gimbal execution sources.'
Assert-NoRegex $chassisMakefileText 'application/shoot/shoot\.c' 'Chassis Makefile must not reference deleted shoot execution sources.'
Assert-NoRegex $chassisMakefileText 'modules/master_machine/master_process\.c' 'Chassis Makefile must not keep the removed master_machine runtime source.'
Assert-NoRegex $chassisMakefileText 'modules/master_machine/seasky_protocol\.c' 'Chassis Makefile must not keep the removed master_machine protocol source.'
Assert-NoRegex $chassisMakefileText '-Iapplication/cmd' 'Chassis Makefile must not include deleted cmd headers.'
Assert-NoRegex $chassisMakefileText '-Iapplication/gimbal' 'Chassis Makefile must not include deleted gimbal headers.'
Assert-NoRegex $chassisMakefileText '-Iapplication/shoot' 'Chassis Makefile must not include deleted shoot headers.'
Assert-NoRegex $chassisMakefileText '-Imodules/master_machine' 'Chassis Makefile must not include master_machine headers after ownership cleanup.'

Assert-NoRegex $chassisRobotDefText '#include\s+"master_process\.h"' 'Chassis robot_def.h must not depend on master_process.h after removing vision-side duties.'
Assert-Regex $chassisRobotDefText 'typedef\s+enum\s*\{\s*BULLET_SPEED_NONE' 'Chassis robot_def.h must own the Bullet_Speed_e enum locally.'

Assert-Regex $chassisCmakeText 'modules/master_machine/\.\*\\\\\.c\$' 'Chassis CMakeLists.txt must explicitly exclude master_machine sources from the board build.'

Write-Output 'PASS: dual-board build entry regression checks'
