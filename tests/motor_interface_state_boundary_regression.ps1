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
$gimbalDji = Join-Path $repoRoot 'Gimbal\modules\motor\DJImotor\dji_motor.c'
$chassisDji = Join-Path $repoRoot 'Chassis\modules\motor\DJImotor\dji_motor.c'
$gimbalDm = Join-Path $repoRoot 'Gimbal\modules\motor\DMmotor\dmmotor.c'
$gimbalDmHeader = Join-Path $repoRoot 'Gimbal\modules\motor\DMmotor\dmmotor.h'
$gimbalDmDoc = Join-Path $repoRoot 'Gimbal\modules\motor\DMmotor\dmmotor.md'
$gimbalDjiText = Get-Content $gimbalDji -Raw
$chassisDjiText = Get-Content $chassisDji -Raw
$gimbalDmText = Get-Content $gimbalDm -Raw
$gimbalDmHeaderText = Get-Content $gimbalDmHeader -Raw
$gimbalDmDocText = Get-Content $gimbalDmDoc -Raw

Assert-Pattern $gimbalDjiText 'void DJIMotorSetRef[\s\S]*?memset\(&motor->motor_controller\.ref_effort, 0,' 'Gimbal DJI SetRef is not clearing direct effort state.'
Assert-Pattern $gimbalDjiText 'void DJIMotorSetEffort[\s\S]*?motor->motor_controller\.pid_ref = 0\.0f;' 'Gimbal DJI SetEffort is not clearing compatibility/bypass carrier.'
Assert-Pattern $gimbalDjiText 'void DJIMotorSetRawRef[\s\S]*?memset\(&motor->motor_controller\.ref_effort, 0,' 'Gimbal DJI SetRawRef is not clearing direct effort state.'

Assert-Pattern $chassisDjiText 'void DJIMotorSetRef[\s\S]*?memset\(&motor->motor_controller\.ref_effort, 0,' 'Chassis DJI SetRef is not clearing direct effort state.'
Assert-Pattern $chassisDjiText 'void DJIMotorSetEffort[\s\S]*?motor->motor_controller\.pid_ref = 0\.0f;' 'Chassis DJI SetEffort is not clearing compatibility carrier.'

Assert-NoPattern $gimbalDmText 'motor->use_pvt_command_frame = \(mode == DM_MODE_PVT\);' 'DM drive mode switch should not arm PVT command path by itself.'
Assert-Pattern $gimbalDmText 'void DMMotorSetRef[\s\S]*?motor->use_pvt_command_frame = 0;[\s\S]*?motor->use_mit_velocity_only = 0;[\s\S]*?motor->use_cascade_pid_path = 1;[\s\S]*?motor->use_mit_full_command = 0;' 'DM SetRef is not enforcing cascade compatibility state.'
Assert-Pattern $gimbalDmText 'void DMMotorSetEffort[\s\S]*?motor->use_pvt_command_frame = 0;[\s\S]*?motor->use_mit_velocity_only = 0;[\s\S]*?motor->use_cascade_pid_path = 0;[\s\S]*?motor->use_mit_full_command = 0;' 'DM SetEffort is not enforcing mainline-exclusive state.'
Assert-Pattern $gimbalDmText 'void DMMotorSetMITTargets[\s\S]*?motor->use_pvt_command_frame = 0;[\s\S]*?motor->use_mit_velocity_only = 0;[\s\S]*?motor->use_cascade_pid_path = 0;[\s\S]*?motor->use_mit_full_command = 1;' 'DM MIT full-command path is not enforcing extension-exclusive state.'
Assert-Pattern $gimbalDmText 'void DMMotorSetPVT[\s\S]*?motor->use_pvt_command_frame = 1;[\s\S]*?motor->use_cascade_pid_path = 0;[\s\S]*?motor->use_mit_full_command = 0;[\s\S]*?motor->use_mit_velocity_only = 0;' 'DM PVT path is not enforcing extension-exclusive state.'

Assert-Pattern $gimbalDmHeaderText 'MIT full-command 兼容接口' 'DM header is missing MIT full-command compatibility label.'
Assert-Pattern $gimbalDmHeaderText 'PVT 扩展接口' 'DM header is missing PVT extension label.'
Assert-Pattern $gimbalDmDocText 'DMMotorSetRef\(\).*兼容接口' 'DM doc is missing SetRef compatibility wording.'
Assert-Pattern $gimbalDmDocText 'DMMotorSetPVT\(\).*扩展能力入口' 'DM doc is missing PVT extension wording.'

Write-Output 'PASS: motor interface state boundary regression checks'
