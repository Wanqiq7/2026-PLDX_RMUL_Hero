$ErrorActionPreference = "Stop"

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
$chassisDocPath = Join-Path $repoRoot "Chassis\\application\\application.md"
$gimbalDocPath = Join-Path $repoRoot "Gimbal\\application\\application.md"

$chassisDocText = Get-Content $chassisDocPath -Raw
$gimbalDocText = Get-Content $gimbalDocPath -Raw

Assert-Regex $chassisDocText "CHASSIS_BOARD" "Chassis application doc must declare the fixed chassis-board runtime role."
Assert-Regex $chassisDocText "(?s)RobotInit[\s\S]*ChassisInit[\s\S]*OSTaskInit" "Chassis application doc must describe the actual RobotInit -> ChassisInit runtime chain."
Assert-Regex $chassisDocText "(?s)RobotTask[\s\S]*ChassisTask" "Chassis application doc must describe the actual RobotTask -> ChassisTask runtime chain."
Assert-Regex $chassisDocText "CAN" "Chassis application doc must describe CAN summary input and state feedback output."
Assert-NoRegex $chassisDocText "robot_cmd.*publish|message_center.*cross-board|gimbal/chassis/shoot" "Chassis application doc must not keep the old single-board robot_cmd pub/sub narrative."

Assert-Regex $gimbalDocText "GIMBAL_BOARD" "Gimbal application doc must declare the fixed gimbal-board runtime role."
Assert-Regex $gimbalDocText "(?s)VisionAppTask[\s\S]*RobotCMDTask[\s\S]*GimbalTask[\s\S]*ShootTask" "Gimbal application doc must describe the current Vision -> RobotCMD -> Gimbal -> Shoot task order."
Assert-Regex $gimbalDocText "CAN" "Gimbal application doc must describe the cross-board CAN summary contract."
Assert-Regex $gimbalDocText "Ref Manager / Arbiter" "Gimbal application doc must keep the Ref Manager wording for the current mainline model."
Assert-NoRegex $gimbalDocText "message_center.*cross-board" "Gimbal application doc must not describe cross-board traffic as local pubsub."

Write-Output "PASS: application runtime model regression checks"
