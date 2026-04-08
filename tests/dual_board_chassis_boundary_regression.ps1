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
$chassisRoot = Join-Path $repoRoot 'Chassis'
$robotDefPath = Join-Path $chassisRoot 'application\robot_def.h'
$robotPath = Join-Path $chassisRoot 'application\robot.c'
$robotTaskPath = Join-Path $chassisRoot 'application\robot_task.h'
$chassisTaskPath = Join-Path $chassisRoot 'application\chassis\chassis.c'
$insTaskPath = Join-Path $chassisRoot 'modules\imu\ins_task.c'
$cmakePath = Join-Path $chassisRoot 'CMakeLists.txt'
$legacyCmdPath = Join-Path $chassisRoot 'application\cmd\robot_cmd.c'
$legacyGimbalPath = Join-Path $chassisRoot 'application\gimbal\gimbal.c'
$legacyShootPath = Join-Path $chassisRoot 'application\shoot\shoot.c'
$canLinkPath = Join-Path $chassisRoot 'modules\can_comm\chassis_can_link.c'

$robotDefText = Get-Content $robotDefPath -Raw
$robotText = Get-Content $robotPath -Raw
$robotTaskText = Get-Content $robotTaskPath -Raw
$chassisTaskText = Get-Content $chassisTaskPath -Raw
$insTaskText = Get-Content $insTaskPath -Raw
$cmakeText = Get-Content $cmakePath -Raw

# 底盘板只能保留固定板型宏，不能继续携带单板或云台板运行角色。
Assert-Regex $robotDefText '^[ \t]*#define[ \t]+CHASSIS_BOARD\b' 'Chassis robot_def.h must keep a single active CHASSIS_BOARD definition.'
Assert-NoRegex $robotDefText '^[ \t]*#define[ \t]+ONE_BOARD\b' 'Chassis robot_def.h must not keep an active ONE_BOARD definition.'
Assert-NoRegex $robotDefText '^[ \t]*#define[ \t]+GIMBAL_BOARD\b' 'Chassis robot_def.h must not keep an active GIMBAL_BOARD definition.'

# 底盘板入口不应再包含 RobotCMD / 云台 / 发射执行主线。
Assert-NoRegex $robotText 'RobotCMDInit\s*\(' 'Chassis robot.c must not initialize RobotCMD.'
Assert-NoRegex $robotText 'RobotCMDTask\s*\(' 'Chassis robot.c must not schedule RobotCMD.'
Assert-NoRegex $robotText 'GimbalInit\s*\(' 'Chassis robot.c must not initialize gimbal execution.'
Assert-NoRegex $robotText 'GimbalTask\s*\(' 'Chassis robot.c must not schedule gimbal execution.'
Assert-NoRegex $robotText 'ShootInit\s*\(' 'Chassis robot.c must not initialize shoot execution.'
Assert-NoRegex $robotText 'ShootTask\s*\(' 'Chassis robot.c must not schedule shoot execution.'
Assert-NoRegex $robotText 'defined\s*\(\s*ONE_BOARD\s*\)' 'Chassis robot.c must not retain ONE_BOARD runtime branching.'
Assert-NoRegex $robotTaskText 'VisionSend\s*\(' 'Chassis robot_task.h must not keep chassis-to-vision transmission hooks.'
Assert-NoRegex $insTaskText 'VisionSetAltitude\s*\(' 'Chassis INS task must not keep vision attitude publishing.'

# 底盘板主线只能通过 CAN 接收控制并回传状态，不能保留单板 Pub/Sub 兼容路径。
Assert-NoRegex $chassisTaskText 'RegisterSubscriber\("chassis_cmd"' 'Chassis task must not register local chassis_cmd subscriber.'
Assert-NoRegex $chassisTaskText 'RegisterPublisher\("chassis_feed"' 'Chassis task must not register local chassis_feed publisher.'
Assert-NoRegex $chassisTaskText 'SubGetMessage\s*\(\s*chassis_sub' 'Chassis task must not read local chassis_sub commands.'
Assert-NoRegex $chassisTaskText 'PubPushMessage\s*\(\s*chassis_pub' 'Chassis task must not publish local chassis feedback.'
Assert-NoRegex $chassisTaskText 'defined\s*\(\s*ONE_BOARD\s*\)' 'Chassis task must not retain ONE_BOARD compatibility branches.'
Assert-Regex $chassisTaskText 'ChassisCanLinkUpdateCommand\s*\(' 'Chassis task must keep CAN command ingestion.'
Assert-Regex $chassisTaskText 'ChassisCanLinkSendFeedbackIfDue\s*\(' 'Chassis task must keep CAN feedback transmission.'
Assert-True (Test-Path $canLinkPath) 'Chassis CAN link helper must remain in place.'

# 底盘板不应再携带对侧执行目录。
Assert-True (-not (Test-Path $legacyCmdPath)) 'Chassis application/cmd legacy execution path must be removed.'
Assert-True (-not (Test-Path $legacyGimbalPath)) 'Chassis application/gimbal legacy execution path must be removed.'
Assert-True (-not (Test-Path $legacyShootPath)) 'Chassis application/shoot legacy execution path must be removed.'

# 构建面必须显式列出应用源文件，而不是递归吞掉整个 application 目录。
Assert-NoRegex $cmakeText '"application/\*\.c"' 'Chassis CMakeLists.txt must not glob all application sources recursively.'
Assert-Regex $cmakeText 'application/chassis/chassis\.c' 'Chassis CMakeLists.txt must explicitly keep chassis.c.'
Assert-Regex $cmakeText 'application/sysid/sysid_task\.c' 'Chassis CMakeLists.txt must explicitly keep sysid_task.c.'
Assert-Regex $cmakeText 'application/robot\.c' 'Chassis CMakeLists.txt must explicitly keep robot.c.'

Write-Output 'PASS: dual-board chassis boundary regression checks'
