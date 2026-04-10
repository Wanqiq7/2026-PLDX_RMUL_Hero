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
$chassisCan = Join-Path $repoRoot 'Chassis\modules\can_comm\chassis_can_link.c'
$robotCmd = Join-Path $repoRoot 'Gimbal\application\cmd\robot_cmd.c'
$chassisDoc = Join-Path $repoRoot 'Chassis\application\chassis\chassis.md'

Assert-Pattern `
    -Path $chassisRobotDef `
    -Pattern 'CHASSIS_SAFETY_STATUS_DEAD = 0x01U' `
    -Message 'Chassis robot_def is missing CHASSIS_SAFETY_STATUS_DEAD.'

Assert-Pattern `
    -Path $chassisRobotDef `
    -Pattern 'CHASSIS_SAFETY_STATUS_READY = 0x02U' `
    -Message 'Chassis robot_def is missing CHASSIS_SAFETY_STATUS_READY.'

Assert-Pattern `
    -Path $chassisRobotDef `
    -Pattern 'uint8_t chassis_safety_status;' `
    -Message 'Chassis robot_def is missing compact chassis_safety_status field.'

Assert-Pattern `
    -Path $gimbalRobotDef `
    -Pattern 'uint8_t chassis_safety_status;' `
    -Message 'Gimbal robot_def is missing compact chassis_safety_status field.'

Assert-NotPattern `
    -Path $chassisRobotDef `
    -Pattern 'uint8_t wheel_online_mask;|uint8_t chassis_inhibit_reason;|uint8_t robot_dead;|uint8_t chassis_ready;' `
    -Message 'Chassis robot_def still exposes verbose safety state across the dual-board boundary.'

Assert-NotPattern `
    -Path $gimbalRobotDef `
    -Pattern 'uint8_t wheel_online_mask;|uint8_t chassis_inhibit_reason;|uint8_t robot_dead;|uint8_t chassis_ready;' `
    -Message 'Gimbal robot_def still exposes verbose safety state across the dual-board boundary.'

Assert-Pattern `
    -Path $chassisCan `
    -Pattern 'chassis_feed_fast_send\.chassis_safety_status = feedback_data->chassis_safety_status;' `
    -Message 'Chassis CAN fast packet is not publishing compact chassis_safety_status.'

Assert-NotPattern `
    -Path $chassisCan `
    -Pattern 'wheel_online_mask|chassis_inhibit_reason|robot_dead|chassis_ready' `
    -Message 'Chassis CAN fast packet still sends verbose safety fields.'

Assert-Pattern `
    -Path $robotCmd `
    -Pattern 'chassis_fetch_data\.chassis_safety_status = chassis_feed_fast_recv\.chassis_safety_status;' `
    -Message 'Robot CMD is not consuming compact chassis_safety_status.'

Assert-NotPattern `
    -Path $robotCmd `
    -Pattern 'chassis_fetch_data\.(wheel_online_mask|chassis_inhibit_reason|robot_dead|chassis_ready)' `
    -Message 'Robot CMD still depends on verbose safety fields from dual-board CAN.'

Assert-Pattern `
    -Path $chassisDoc `
    -Pattern 'chassis_safety_status' `
    -Message 'Chassis doc is missing compact chassis_safety_status documentation.'

Write-Output 'PASS: chassis safety status protocol boundary regression checks'
