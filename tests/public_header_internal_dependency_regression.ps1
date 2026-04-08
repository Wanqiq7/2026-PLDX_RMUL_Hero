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

$powerHeaderText = Get-Content (Join-Path $repoRoot 'Chassis\modules\power_controller\power_controller.h') -Raw
$powerSourceText = Get-Content (Join-Path $repoRoot 'Chassis\modules\power_controller\power_controller.c') -Raw
$chassisInsHeaderText = Get-Content (Join-Path $repoRoot 'Chassis\modules\imu\ins_task.h') -Raw
$chassisInsSourceText = Get-Content (Join-Path $repoRoot 'Chassis\modules\imu\ins_task.c') -Raw
$gimbalInsHeaderText = Get-Content (Join-Path $repoRoot 'Gimbal\modules\imu\ins_task.h') -Raw
$gimbalInsSourceText = Get-Content (Join-Path $repoRoot 'Gimbal\modules\imu\ins_task.c') -Raw

Assert-NoRegex $powerHeaderText '#include\s+"estimation/identification/rls_estimator\.h"' 'FAIL: power_controller.h still exposes internal estimation dependency'
Assert-Regex $powerHeaderText 'typedef\s+struct\s+PowerControllerInstance\s+PowerControllerInstance;' 'power_controller.h must preserve the opaque PowerControllerInstance forward declaration.'
Assert-Regex $powerHeaderText 'typedef\s+struct\s*\{\s*[\s\S]*?\}\s*PowerControllerConfig_t;' 'power_controller.h must continue exposing PowerControllerConfig_t.'
Assert-Regex $powerHeaderText 'typedef\s+struct\s*\{\s*[\s\S]*?\}\s*PowerControllerStatus_t;' 'power_controller.h must continue exposing PowerControllerStatus_t.'
Assert-Regex $powerHeaderText 'PowerControllerRegister' 'power_controller.h must preserve the public register API.'
Assert-Regex $powerSourceText '#include\s+"estimation/identification/rls_estimator\.h"' 'power_controller.c must include rls_estimator.h after the public header is de-internalized.'

Assert-NoRegex $chassisInsHeaderText '#include\s+"estimation/attitude/quaternion_ekf\.h"' 'FAIL: ins_task.h still exposes internal estimation dependency'
Assert-NoRegex $chassisInsHeaderText '#include\s+"BMI088driver\.h"' 'FAIL: ins_task.h still exposes internal BMI088 driver dependency'
Assert-NoRegex $chassisInsHeaderText '#define\s+X\s+0' 'Chassis ins_task.h must not leak generic X/Y/Z axis macros into public consumers.'
Assert-NoRegex $chassisInsHeaderText '#define\s+Y\s+1' 'Chassis ins_task.h must not leak generic X/Y/Z axis macros into public consumers.'
Assert-NoRegex $chassisInsHeaderText '#define\s+Z\s+2' 'Chassis ins_task.h must not leak generic X/Y/Z axis macros into public consumers.'
Assert-Regex $chassisInsHeaderText 'typedef\s+struct\s*\{\s*[\s\S]*?\}\s*attitude_t;' 'Chassis ins_task.h must preserve the public attitude_t payload.'
Assert-Regex $chassisInsHeaderText 'attitude_t\s*\*\s*INS_Init\s*\(' 'Chassis ins_task.h must preserve the INS_Init public API.'
Assert-Regex $chassisInsHeaderText 'void\s+INS_Task\s*\(' 'Chassis ins_task.h must preserve the INS_Task public API.'
Assert-Regex $chassisInsSourceText '#include\s+"estimation/attitude/quaternion_ekf\.h"' 'Chassis ins_task.c must include quaternion_ekf.h after the public header is de-internalized.'
Assert-Regex $chassisInsSourceText '#include\s+"BMI088driver\.h"' 'Chassis ins_task.c must include BMI088driver.h after the public header is de-internalized.'

Assert-NoRegex $gimbalInsHeaderText '#include\s+"estimation/attitude/quaternion_ekf\.h"' 'FAIL: gimbal ins_task.h still exposes internal estimation dependency'
Assert-NoRegex $gimbalInsHeaderText '#include\s+"BMI088driver\.h"' 'FAIL: gimbal ins_task.h still exposes internal BMI088 driver dependency'
Assert-NoRegex $gimbalInsHeaderText '#define\s+X\s+0' 'Gimbal ins_task.h must not leak generic X/Y/Z axis macros into public consumers.'
Assert-NoRegex $gimbalInsHeaderText '#define\s+Y\s+1' 'Gimbal ins_task.h must not leak generic X/Y/Z axis macros into public consumers.'
Assert-NoRegex $gimbalInsHeaderText '#define\s+Z\s+2' 'Gimbal ins_task.h must not leak generic X/Y/Z axis macros into public consumers.'
Assert-Regex $gimbalInsHeaderText 'typedef\s+struct\s*\{\s*[\s\S]*?\}\s*attitude_t;' 'Gimbal ins_task.h must preserve the public attitude_t payload.'
Assert-Regex $gimbalInsHeaderText 'attitude_t\s*\*\s*INS_Init\s*\(' 'Gimbal ins_task.h must preserve the INS_Init public API.'
Assert-Regex $gimbalInsHeaderText 'void\s+INS_Task\s*\(' 'Gimbal ins_task.h must preserve the INS_Task public API.'
Assert-Regex $gimbalInsSourceText '#include\s+"estimation/attitude/quaternion_ekf\.h"' 'Gimbal ins_task.c must include quaternion_ekf.h after the public header is de-internalized.'
Assert-Regex $gimbalInsSourceText '#include\s+"BMI088driver\.h"' 'Gimbal ins_task.c must include BMI088driver.h after the public header is de-internalized.'

Write-Output 'PASS: public header internal dependency regression checks'
