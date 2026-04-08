$ErrorActionPreference = 'Stop'

function Assert-Exists {
    param(
        [string]$Path,
        [string]$Message
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        throw $Message
    }
}

function Assert-NotExists {
    param(
        [string]$Path,
        [string]$Message
    )

    if (Test-Path -LiteralPath $Path) {
        throw $Message
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$gimbalAlg = Join-Path $repoRoot 'Gimbal\modules\algorithm'
$chassisAlg = Join-Path $repoRoot 'Chassis\modules\algorithm'

$expectedGimbal = @(
    'controllers\pid\pid_controller.h',
    'controllers\pid\pid_controller.c',
    'controllers\lqr\lqr_controller.h',
    'controllers\lqr\lqr_controller.c',
    'controllers\smc\smc_controller.h',
    'controllers\smc\smc_controller.c',
    'controllers\domain\shoot_effort_controller.h',
    'controllers\domain\shoot_effort_controller.c',
    'controllers\domain\vision_control.h',
    'controllers\domain\vision_control.c',
    'controllers\reference\gimbal_ref_manager.h',
    'controllers\reference\gimbal_ref_manager.c',
    'controllers\models\heat_gate_model.h',
    'controllers\models\heat_gate_model.c',
    'estimation\kalman\kalman_filter.h',
    'estimation\kalman\kalman_filter.c',
    'estimation\attitude\quaternion_ekf.h',
    'estimation\attitude\quaternion_ekf.c',
    'utils\checksum\crc8.h',
    'utils\checksum\crc8.c',
    'utils\checksum\crc16.h',
    'utils\checksum\crc16.c',
    'utils\math\user_lib.h',
    'utils\math\user_lib.c',
    'utils\math\arm_math_compat.h'
)

$expectedChassis = @(
    'controllers\pid\pid_controller.h',
    'controllers\pid\pid_controller.c',
    'controllers\lqr\lqr_controller.h',
    'controllers\lqr\lqr_controller.c',
    'controllers\domain\chassis_force_control.h',
    'controllers\domain\chassis_force_control.c',
    'controllers\domain\shoot_effort_controller.h',
    'controllers\domain\shoot_effort_controller.c',
    'estimation\kalman\kalman_filter.h',
    'estimation\kalman\kalman_filter.c',
    'estimation\attitude\quaternion_ekf.h',
    'estimation\attitude\quaternion_ekf.c',
    'estimation\identification\rls_estimator.h',
    'estimation\identification\rls_estimator.c',
    'utils\checksum\crc8.h',
    'utils\checksum\crc8.c',
    'utils\checksum\crc16.h',
    'utils\checksum\crc16.c',
    'utils\math\user_lib.h',
    'utils\math\user_lib.c',
    'utils\math\arm_math_compat.h'
)

$legacyTopLevel = @(
    'controller.h',
    'controller.c',
    'QuaternionEKF.h',
    'QuaternionEKF.c',
    'SMC_Controller.h',
    'SMC_Controller.c',
    'kalman_filter.h',
    'kalman_filter.c',
    'crc8.h',
    'crc8.c',
    'crc16.h',
    'crc16.c',
    'user_lib.h',
    'user_lib.c',
    'arm_math_compat.h',
    'gimbal_ref_manager.h',
    'gimbal_ref_manager.c',
    'vision_control.h',
    'vision_control.c',
    'shoot_effort_controller.h',
    'shoot_effort_controller.c',
    'heat_gate_model.h',
    'heat_gate_model.c',
    'chassis_force_control.h',
    'chassis_force_control.c'
)

foreach ($rel in $expectedGimbal) {
    Assert-Exists (Join-Path $gimbalAlg $rel) "Missing expected Gimbal algorithm file: $rel"
}

foreach ($rel in $expectedChassis) {
    Assert-Exists (Join-Path $chassisAlg $rel) "Missing expected Chassis algorithm file: $rel"
}

foreach ($legacy in $legacyTopLevel) {
    Assert-NotExists (Join-Path $gimbalAlg $legacy) "Legacy top-level Gimbal algorithm file must be removed: $legacy"
    Assert-NotExists (Join-Path $chassisAlg $legacy) "Legacy top-level Chassis algorithm file must be removed: $legacy"
}

Write-Output 'PASS: algorithm layer layout regression checks'
