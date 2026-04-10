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

function Assert-NotPattern {
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
$chassisRobotDef = Join-Path $repoRoot 'Chassis\application\robot_def.h'
$gimbalRobotDef = Join-Path $repoRoot 'Gimbal\application\robot_def.h'
$chassisTask = Join-Path $repoRoot 'Chassis\application\chassis\chassis.c'
$chassisCan = Join-Path $repoRoot 'Chassis\modules\can_comm\chassis_can_link.c'
$robotCmd = Join-Path $repoRoot 'Gimbal\application\cmd\robot_cmd.c'

Assert-Pattern `
    -Path $chassisRobotDef `
    -Pattern 'uint8_t chassis_safety_status;' `
    -Message 'Chassis robot_def is missing compact chassis_safety_status in chassis feedback contract.'

Assert-Pattern `
    -Path $chassisRobotDef `
    -Pattern 'CHASSIS_SAFETY_STATUS_DEAD = 0x01U' `
    -Message 'Chassis robot_def is missing CHASSIS_SAFETY_STATUS_DEAD flag.'

Assert-Pattern `
    -Path $chassisRobotDef `
    -Pattern 'CHASSIS_SAFETY_STATUS_READY = 0x02U' `
    -Message 'Chassis robot_def is missing CHASSIS_SAFETY_STATUS_READY flag.'

Assert-NotPattern `
    -Path $chassisRobotDef `
    -Pattern 'uint8_t wheel_online_mask;' `
    -Message 'Chassis robot_def still exposes wheel_online_mask across the board boundary.'

Assert-Pattern `
    -Path $gimbalRobotDef `
    -Pattern 'uint8_t chassis_safety_status;' `
    -Message 'Gimbal robot_def is missing compact chassis_safety_status in mirrored chassis feedback contract.'

Assert-Pattern `
    -Path $gimbalRobotDef `
    -Pattern 'CHASSIS_SAFETY_STATUS_DEAD = 0x01U' `
    -Message 'Gimbal robot_def is missing CHASSIS_SAFETY_STATUS_DEAD flag.'

Assert-Pattern `
    -Path $gimbalRobotDef `
    -Pattern 'CHASSIS_SAFETY_STATUS_READY = 0x02U' `
    -Message 'Gimbal robot_def is missing CHASSIS_SAFETY_STATUS_READY flag.'

Assert-NotPattern `
    -Path $gimbalRobotDef `
    -Pattern 'uint8_t wheel_online_mask;' `
    -Message 'Gimbal robot_def still exposes wheel_online_mask across the board boundary.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern '#define CHASSIS_ALL_WHEELS_STABLE_MS' `
    -Message 'Chassis task is missing stable recovery debounce constant.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'ChassisSafetyInput_s' `
    -Message 'Chassis task is missing compact ChassisSafetyInput_s file-local input contract.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'ChassisSafetyResult_s' `
    -Message 'Chassis task is missing compact ChassisSafetyResult_s file-local result contract.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'uint32_t now_ms;' `
    -Message 'ChassisSafetyInput_s is missing now_ms.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'uint8_t comm_online;' `
    -Message 'ChassisSafetyInput_s is missing comm_online.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'uint8_t wheel_online_mask;' `
    -Message 'ChassisSafetyInput_s is missing wheel_online_mask.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'uint8_t should_hold;' `
    -Message 'ChassisSafetyResult_s is missing should_hold.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'uint8_t chassis_safety_status;' `
    -Message 'ChassisSafetyResult_s is missing chassis_safety_status.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'static void BuildChassisSafetyInput\(ChassisSafetyInput_s \*input,' `
    -Message 'Chassis task is missing BuildChassisSafetyInput helper declaration.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'static void StepChassisSafetyGate\(const ChassisSafetyInput_s \*input,' `
    -Message 'Chassis task is missing StepChassisSafetyGate helper declaration.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'static uint8_t ShouldHoldChassisControl\(' `
    -Message 'Chassis task is missing ShouldHoldChassisControl helper declaration.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'static void ApplyChassisSafetyFeedback\(' `
    -Message 'Chassis task is missing ApplyChassisSafetyFeedback helper declaration.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'static void LogChassisSafetyTransition\(' `
    -Message 'Chassis task is missing LogChassisSafetyTransition helper declaration.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'GameRobotState\.current_HP == 0U' `
    -Message 'Chassis local death detection does not gate on current_HP == 0.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'power_management_chassis_output == 0U' `
    -Message 'Chassis local death detection does not gate on power_management_chassis_output == 0.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'DaemonIsOnline\(motor_rf->daemon\)' `
    -Message 'Chassis runtime gate does not check RF motor online state.'

Assert-Pattern `
    -Path $chassisTask `
    -Pattern 'ShouldHoldChassisControl\(&chassis_cmd_recv, &safety_result\)' `
    -Message 'Chassis control path is not gated through ShouldHoldChassisControl.'

Assert-NotPattern `
    -Path $chassisTask `
    -Pattern 'UpdateChassisRuntimeGate' `
    -Message 'Chassis task still uses the old monolithic UpdateChassisRuntimeGate helper.'

Assert-Pattern `
    -Path $chassisCan `
    -Pattern 'chassis_feed_fast_send\.chassis_safety_status = feedback_data->chassis_safety_status;' `
    -Message 'Chassis CAN fast feedback packet is missing compact chassis_safety_status.'

Assert-NotPattern `
    -Path $chassisCan `
    -Pattern 'wheel_online_mask|chassis_inhibit_reason' `
    -Message 'Chassis CAN fast feedback packet still sends verbose wheel/reason fields.'

Assert-Pattern `
    -Path $robotCmd `
    -Pattern 'chassis_fetch_data\.chassis_safety_status = chassis_feed_fast_recv\.chassis_safety_status;' `
    -Message 'Robot CMD is not reading chassis_safety_status from chassis CAN feedback.'

Assert-Pattern `
    -Path $robotCmd `
    -Pattern 'CHASSIS_SAFETY_STATUS_DEAD' `
    -Message 'Robot safety state does not use CHASSIS_SAFETY_STATUS_DEAD.'

Assert-Pattern `
    -Path $robotCmd `
    -Pattern 'CHASSIS_SAFETY_STATUS_READY' `
    -Message 'Robot safety state does not use CHASSIS_SAFETY_STATUS_READY.'

Assert-NotPattern `
    -Path $robotCmd `
    -Pattern 'wheel_online_mask|chassis_inhibit_reason' `
    -Message 'Robot CMD still depends on verbose wheel/reason fields from CAN feedback.'

Write-Output 'PASS: chassis death recovery safety regression checks'
