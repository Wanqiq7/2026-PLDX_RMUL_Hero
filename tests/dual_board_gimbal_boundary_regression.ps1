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
$gimbalRoot = Join-Path $repoRoot 'Gimbal'
$robotDefPath = Join-Path $gimbalRoot 'application\robot_def.h'
$robotPath = Join-Path $gimbalRoot 'application\robot.c'
$robotCmdPath = Join-Path $gimbalRoot 'application\cmd\robot_cmd.c'
$cmakePath = Join-Path $gimbalRoot 'CMakeLists.txt'
$legacyChassisPath = Join-Path $gimbalRoot 'application\chassis\chassis.c'

$robotDefText = Get-Content $robotDefPath -Raw
$robotText = Get-Content $robotPath -Raw
$robotCmdText = Get-Content $robotCmdPath -Raw
$cmakeText = Get-Content $cmakePath -Raw

# 云台板只能保留固定板型宏，不能继续携带单板或底盘板运行角色。
Assert-Regex $robotDefText '^[ \t]*#define[ \t]+GIMBAL_BOARD\b' 'Gimbal robot_def.h must keep a single active GIMBAL_BOARD definition.'
Assert-NoRegex $robotDefText '^[ \t]*#define[ \t]+ONE_BOARD\b' 'Gimbal robot_def.h must not keep an active ONE_BOARD definition.'
Assert-NoRegex $robotDefText '^[ \t]*#define[ \t]+CHASSIS_BOARD\b' 'Gimbal robot_def.h must not keep an active CHASSIS_BOARD definition.'

# 云台板入口不应再包含底盘执行主线。
Assert-NoRegex $robotText 'ChassisInit\s*\(' 'Gimbal robot.c must not initialize chassis execution.'
Assert-NoRegex $robotText 'ChassisTask\s*\(' 'Gimbal robot.c must not schedule chassis execution.'
Assert-NoRegex $robotText 'defined\s*\(\s*ONE_BOARD\s*\)' 'Gimbal robot.c must not retain ONE_BOARD runtime branching.'

# 云台板控制链只能通过 CAN 与底盘交互，不能保留本板 Pub/Sub 兼容路径。
Assert-NoRegex $robotCmdText '\bchassis_cmd_pub\b' 'Gimbal RobotCMD must not keep chassis_cmd_pub local pubsub state.'
Assert-NoRegex $robotCmdText '\bchassis_feed_sub\b' 'Gimbal RobotCMD must not keep chassis_feed_sub local pubsub state.'
Assert-NoRegex $robotCmdText 'defined\s*\(\s*ONE_BOARD\s*\)' 'Gimbal RobotCMD must not retain ONE_BOARD chassis compatibility branches.'
Assert-Regex $robotCmdText 'UpdateChassisFetchDataFromCan\s*\(' 'Gimbal RobotCMD must keep CAN-based chassis feedback ingestion.'
Assert-Regex $robotCmdText 'SendChassisCommandCanIfDue\s*\(' 'Gimbal RobotCMD must keep CAN-based chassis command sending.'

# 云台板不应再携带底盘执行目录。
Assert-True (-not (Test-Path $legacyChassisPath)) 'Gimbal application/chassis legacy execution path must be removed.'

# 构建面必须显式列出应用源文件，而不是递归吞掉整个 application 目录。
Assert-NoRegex $cmakeText '"application/\*\.c"' 'Gimbal CMakeLists.txt must not glob all application sources recursively.'
Assert-Regex $cmakeText 'application/cmd/robot_cmd\.c' 'Gimbal CMakeLists.txt must explicitly keep robot_cmd.c.'
Assert-Regex $cmakeText 'application/gimbal/gimbal\.c' 'Gimbal CMakeLists.txt must explicitly keep gimbal.c.'
Assert-Regex $cmakeText 'application/shoot/shoot\.c' 'Gimbal CMakeLists.txt must explicitly keep shoot.c.'
Assert-Regex $cmakeText 'application/vision/vision\.c' 'Gimbal CMakeLists.txt must explicitly keep vision.c.'
Assert-Regex $cmakeText 'application/sysid/sysid_task\.c' 'Gimbal CMakeLists.txt must explicitly keep sysid_task.c.'

Write-Output 'PASS: dual-board gimbal boundary regression checks'
