Set-StrictMode -Version Latest

$script:PowerControllerConstants = [pscustomobject]@{
    PowerPdKp                       = 50.0
    PowerPdKd                       = 0.20
    PowerPdDFilterAlpha             = 0.3
    ErrorPowerDistributionThreshold = 20.0
    PropPowerDistributionThreshold  = 15.0
    RefereeFullBuff                 = 60.0
    RefereeBaseBuff                 = 50.0
    CapFullBuff                     = 230.0
    CapBaseBuff                     = 30.0
    MaxCapPowerOut                  = 300.0
    CapRefereeBothGgCoe             = 0.85
    MotorDisconnectTimeout          = 1000
    MinPowerConfigured              = 30.0
    MaxRobotLevel                   = 10
    HeroChassisPowerLimit           = @(100, 100, 100, 100, 100, 100, 100, 100, 100, 100)
    InfantryChassisPowerLimit       = @(45, 50, 55, 60, 65, 70, 75, 80, 90, 100)
    SentryChassisPowerLimit         = 100
    PowerErrorNone                  = 0
    PowerErrorMotorDisconnect       = 0x01
    PowerErrorRefereeDisconnect     = 0x02
    PowerErrorCapDisconnect         = 0x04
}

function Get-PowerControllerRegressionPaths {
    param([Parameter(Mandatory = $true)][string]$ScriptPath)

    $repoRoot = Split-Path -Parent (Split-Path -Parent $ScriptPath)
    [pscustomobject]@{
        RepoRoot      = $repoRoot
        HarnessHeader = Join-Path $repoRoot 'tests\harnesses\power_controller_sample_harness.h'
        HarnessSource = Join-Path $repoRoot 'tests\harnesses\power_controller_sample_harness.c'
        FixtureReadme = Join-Path $repoRoot 'tests\fixtures\power_controller\README.md'
        PowerSource   = Join-Path $repoRoot 'Chassis\modules\power_controller\power_controller.c'
        PowerHeader   = Join-Path $repoRoot 'Chassis\modules\power_controller\power_controller.h'
    }
}

function Assert-PowerControllerRegressionAssets {
    param([Parameter(Mandatory = $true)][pscustomobject]$Paths)

    foreach ($path in @($Paths.HarnessHeader, $Paths.HarnessSource, $Paths.FixtureReadme, $Paths.PowerSource, $Paths.PowerHeader)) {
        if (-not (Test-Path $path)) {
            throw "Missing required regression asset: $path"
        }
    }

    if (-not (Select-String -Path $Paths.HarnessHeader -Pattern 'PowerControllerRunSample' -Quiet)) {
        throw 'Sample harness header is missing PowerControllerRunSample declaration.'
    }

    if (-not (Select-String -Path $Paths.HarnessHeader -Pattern 'PowerControllerRunSampleSequence' -Quiet)) {
        throw 'Sample harness header is missing PowerControllerRunSampleSequence declaration.'
    }

    if (-not (Select-String -Path $Paths.PowerHeader -Pattern 'PowerGetStatus' -Quiet)) {
        throw 'power_controller.h no longer exposes PowerGetStatus().'
    }
}

function Get-PowerControllerFixtureDocument {
    param([Parameter(Mandatory = $true)][string]$FixturePath)

    if (-not (Test-Path $FixturePath)) {
        throw "Missing fixture: $FixturePath"
    }

    Get-Content -Raw -Encoding UTF8 $FixturePath | ConvertFrom-Json
}

function Test-ObjectProperty {
    param([object]$Object, [string]$Name)

    if ($null -eq $Object) {
        return $false
    }

    $null -ne $Object.PSObject.Properties[$Name]
}

function Get-ObjectValueOrDefault {
    param([object]$Object, [string]$Name, $DefaultValue)

    if (Test-ObjectProperty -Object $Object -Name $Name) {
        return $Object.$Name
    }

    $DefaultValue
}

function Get-PowerControllerDefaultConfig {
    [pscustomobject]@{
        k1_init         = 0.22
        k2_init         = 1.2
        k3              = 5.1
        rls_lambda      = 0.98
        torque_constant = 1.0
        current_scale   = 0.001
        robot_division  = 'infantry'
    }
}

function Merge-PowerControllerConfig {
    param([object]$Override)

    $config = Get-PowerControllerDefaultConfig
    if ($null -eq $Override) {
        return $config
    }

    foreach ($name in @('k1_init', 'k2_init', 'k3', 'rls_lambda', 'torque_constant', 'current_scale', 'robot_division')) {
        if ($null -ne $Override.$name) {
            $config.$name = $Override.$name
        }
    }

    $config
}

function New-PowerControllerState {
    param([Parameter(Mandatory = $true)][pscustomobject]$Config)

    [pscustomobject]@{
        config             = $Config
        rls                = [pscustomobject]@{
            lambda        = [double]$Config.rls_lambda
            delta         = 1e-5
            trans_matrix  = @(1e-5, 0.0, 0.0, 1e-5)
            gain_vector   = @(0.0, 0.0)
            params_vector = @([double]$Config.k1_init, [double]$Config.k2_init)
            update_cnt    = 0
        }
        k1                 = [double]$Config.k1_init
        k2                 = [double]$Config.k2_init
        k3                 = [double]$Config.k3
        referee            = [pscustomobject]@{
            limit_w       = 80.0
            buffer_energy = 60.0
            power_w       = 0.0
            online        = 0
            robot_level   = 1
        }
        cap                = [pscustomobject]@{
            energy_percent = 0
            online         = 0
        }
        motor              = [pscustomobject]@{
            speeds         = @(0.0, 0.0, 0.0, 0.0)
            torques        = @(0.0, 0.0, 0.0, 0.0)
            online         = @(1, 1, 1, 1)
            disconnect_cnt = @(0, 0, 0, 0)
        }
        pd_last_error_full = 0.0
        pd_last_error_base = 0.0
        pd_d_error_full    = 0.0
        pd_d_error_base    = 0.0
        limit              = [pscustomobject]@{
            max_power   = 100.0
            power_upper = 80.0
            power_lower = 15.0
        }
        error_flags        = 0
        last_robot_level   = 1
        status             = [pscustomobject]@{
            k1               = [double]$Config.k1_init
            k2               = [double]$Config.k2_init
            allowed_power_w  = 100.0
            upper_limit_w    = 0.0
            lower_limit_w    = 0.0
            ref_limit_w      = 0.0
            hard_limit_w     = 0.0
            est_power_w      = 0.0
            measured_power_w = 0.0
            cmd_power_sum_w  = 0.0
            mech_power_w     = 0.0
            loss_power_w     = 0.0
            buffer_feedback  = 0.0
            cap_energy_est   = 0.0
            cap_online       = 0
            referee_online   = 0
            error_flags      = 0
            rls_enabled      = 1
            robot_level      = 1
        }
    }
}

function Invoke-LowPassFilterFloat {
    param([double]$NewValue, [double]$K, [ref]$LastValue)

    $output = $K * $NewValue + (1.0 - $K) * $LastValue.Value
    $LastValue.Value = $output
    $output
}

function Invoke-FloatConstrain {
    param([double]$Value, [double]$MinValue, [double]$MaxValue)

    if ($Value -lt $MinValue) { return $MinValue }
    if ($Value -gt $MaxValue) { return $MaxValue }
    $Value
}

function Get-PowerLimitByLevel {
    param([string]$RobotDivision, [int]$Level)

    $level = [Math]::Min([Math]::Max($Level, 1), $script:PowerControllerConstants.MaxRobotLevel)
    switch ($RobotDivision) {
        'hero'   { return [double]$script:PowerControllerConstants.HeroChassisPowerLimit[$level - 1] }
        'sentry' { return [double]$script:PowerControllerConstants.SentryChassisPowerLimit }
        default  { return [double]$script:PowerControllerConstants.InfantryChassisPowerLimit[$level - 1] }
    }
}

function Update-PowerErrorFlags {
    param([Parameter(Mandatory = $true)][pscustomobject]$State)

    $errorFlags = $script:PowerControllerConstants.PowerErrorNone
    if (-not $State.cap.online) { $errorFlags = $errorFlags -bor $script:PowerControllerConstants.PowerErrorCapDisconnect }
    if (-not $State.referee.online) { $errorFlags = $errorFlags -bor $script:PowerControllerConstants.PowerErrorRefereeDisconnect }

    foreach ($online in $State.motor.online) {
        if (-not $online) {
            $errorFlags = $errorFlags -bor $script:PowerControllerConstants.PowerErrorMotorDisconnect
            break
        }
    }

    $State.error_flags = $errorFlags
    $State.status.error_flags = $errorFlags
}

function Update-PowerStatistics {
    param([Parameter(Mandatory = $true)][pscustomobject]$State)

    $sumAbsSpeed = 0.0
    $sumTorqueSq = 0.0
    $mechPower = 0.0

    for ($i = 0; $i -lt 4; $i++) {
        if ($State.motor.disconnect_cnt[$i] -ge $script:PowerControllerConstants.MotorDisconnectTimeout) { continue }
        $speed = [double]$State.motor.speeds[$i]
        $torque = [double]$State.motor.torques[$i]
        $sumAbsSpeed += [Math]::Abs($speed)
        $sumTorqueSq += $torque * $torque
        $mechPower += $torque * $speed
    }

    $State.status.mech_power_w = $mechPower
    $State.status.est_power_w = $mechPower + $State.k1 * $sumAbsSpeed + $State.k2 * $sumTorqueSq + $State.k3
    $State.status.measured_power_w = [double]$State.referee.power_w
    $State.status.loss_power_w = $State.status.measured_power_w - $mechPower

    if ($State.cap.online) {
        $State.status.cap_energy_est = [double]$State.cap.energy_percent
    } else {
        $bufferRatio = [double]$State.referee.buffer_energy / $script:PowerControllerConstants.RefereeFullBuff
        $State.status.cap_energy_est = Invoke-FloatConstrain ($bufferRatio * 255.0) 0.0 255.0
    }
}

function Invoke-EnergyLoopControlMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State)

    Update-PowerErrorFlags -State $State

    $refLimitW = [double]$State.referee.limit_w
    $hardLimitW = $refLimitW
    $bufferFeedback = [double]$State.referee.buffer_energy
    $fullBufferTarget = $script:PowerControllerConstants.RefereeFullBuff
    $baseBufferTarget = $script:PowerControllerConstants.RefereeBaseBuff

    if ($State.error_flags -band $script:PowerControllerConstants.PowerErrorRefereeDisconnect) {
        $refLimitW = Get-PowerLimitByLevel -RobotDivision $State.config.robot_division -Level $State.last_robot_level
    } else {
        if ($State.referee.robot_level -ge 1 -and $State.referee.robot_level -le $script:PowerControllerConstants.MaxRobotLevel) {
            $State.last_robot_level = [int]$State.referee.robot_level
        }
    }

    if ($State.cap.online) {
        $hardLimitW = $refLimitW + $script:PowerControllerConstants.MaxCapPowerOut
    }

    if (($State.error_flags -band $script:PowerControllerConstants.PowerErrorRefereeDisconnect) -and $State.cap.online) {
        $bufferFeedback = [double]$State.cap.energy_percent
        $fullBufferTarget = $script:PowerControllerConstants.CapFullBuff
        $baseBufferTarget = $script:PowerControllerConstants.CapBaseBuff
    }

    $bufferFeedback = [Math]::Max($bufferFeedback, 0.0)
    $errorFull = [Math]::Sqrt($fullBufferTarget) - [Math]::Sqrt($bufferFeedback)
    $errorBase = [Math]::Sqrt($baseBufferTarget) - [Math]::Sqrt($bufferFeedback)

    $dErrorFull = $errorFull - $State.pd_last_error_full
    $dErrorBase = $errorBase - $State.pd_last_error_base
    $pdLastFull = [ref]$State.pd_d_error_full
    $pdLastBase = [ref]$State.pd_d_error_base
    $State.pd_d_error_full = Invoke-LowPassFilterFloat -NewValue $dErrorFull -K $script:PowerControllerConstants.PowerPdDFilterAlpha -LastValue $pdLastFull
    $State.pd_d_error_base = Invoke-LowPassFilterFloat -NewValue $dErrorBase -K $script:PowerControllerConstants.PowerPdDFilterAlpha -LastValue $pdLastBase
    $pdOutputFull = $script:PowerControllerConstants.PowerPdKp * $errorFull + $script:PowerControllerConstants.PowerPdKd * $State.pd_d_error_full
    $pdOutputBase = $script:PowerControllerConstants.PowerPdKp * $errorBase + $script:PowerControllerConstants.PowerPdKd * $State.pd_d_error_base

    $State.pd_last_error_full = $errorFull
    $State.pd_last_error_base = $errorBase

    $allowedPowerW = $hardLimitW
    $State.limit.power_upper = Invoke-FloatConstrain ($refLimitW - $pdOutputFull) $script:PowerControllerConstants.MinPowerConfigured $hardLimitW
    $State.limit.power_lower = Invoke-FloatConstrain ($refLimitW - $pdOutputBase) $script:PowerControllerConstants.MinPowerConfigured $hardLimitW

    if ($State.limit.power_lower -gt $State.limit.power_upper) {
        $State.limit.power_lower = $State.limit.power_upper
    }

    $capOffline = [bool]($State.error_flags -band $script:PowerControllerConstants.PowerErrorCapDisconnect)
    $refOffline = [bool]($State.error_flags -band $script:PowerControllerConstants.PowerErrorRefereeDisconnect)
    if ($capOffline -and $refOffline) {
        $allowedPowerW = $refLimitW * $script:PowerControllerConstants.CapRefereeBothGgCoe
        $allowedPowerW = Invoke-FloatConstrain $allowedPowerW $script:PowerControllerConstants.MinPowerConfigured $hardLimitW
        $State.limit.power_upper = $allowedPowerW
        $State.limit.power_lower = $allowedPowerW
        $State.pd_last_error_full = 0.0
        $State.pd_last_error_base = 0.0
        $State.pd_d_error_full = 0.0
        $State.pd_d_error_base = 0.0
    } else {
        $allowedPowerW = Invoke-FloatConstrain $allowedPowerW $State.limit.power_lower $State.limit.power_upper
    }

    $allowedPowerW = Invoke-FloatConstrain $allowedPowerW $script:PowerControllerConstants.MinPowerConfigured $hardLimitW
    $State.limit.max_power = $allowedPowerW
    $State.status.allowed_power_w = $allowedPowerW
    $State.status.upper_limit_w = $State.limit.power_upper
    $State.status.lower_limit_w = $State.limit.power_lower
    $State.status.ref_limit_w = $refLimitW
    $State.status.hard_limit_w = $hardLimitW
    $State.status.buffer_feedback = $bufferFeedback
    $State.status.cap_online = [int]$State.cap.online
    $State.status.referee_online = [int]$State.referee.online
    $State.status.robot_level = [int]$State.last_robot_level
}

function Invoke-PowerRlsUpdateMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State)

    if (-not $State.status.rls_enabled) { return $false }
    $feedbackPowerW = [double]$State.referee.power_w
    if ([Math]::Abs($feedbackPowerW) -lt 5.0) { return $false }

    $sampleVector0 = 0.0
    $sampleVector1 = 0.0
    $mechPowerW = 0.0
    for ($i = 0; $i -lt 4; $i++) {
        $speed = [double]$State.motor.speeds[$i]
        $torque = [double]$State.motor.torques[$i]
        $sampleVector0 += [Math]::Abs($speed)
        $sampleVector1 += $torque * $torque
        $mechPowerW += $torque * $speed
    }

    $powerLoss = $feedbackPowerW - $mechPowerW - $State.k3
    $p0 = $State.rls.trans_matrix[0] * $sampleVector0 + $State.rls.trans_matrix[1] * $sampleVector1
    $p1 = $State.rls.trans_matrix[2] * $sampleVector0 + $State.rls.trans_matrix[3] * $sampleVector1
    $phiTPPhi = $sampleVector0 * $p0 + $sampleVector1 * $p1
    $denominator = $State.rls.lambda + $phiTPPhi
    if ([Math]::Abs($denominator) -lt 1e-10) { return $false }

    $k0 = $p0 / $denominator
    $k1 = $p1 / $denominator
    $State.rls.gain_vector[0] = $k0
    $State.rls.gain_vector[1] = $k1

    $phiTTheta = $sampleVector0 * $State.rls.params_vector[0] + $sampleVector1 * $State.rls.params_vector[1]
    $error = $powerLoss - $phiTTheta

    $State.rls.params_vector[0] += $k0 * $error
    $State.rls.params_vector[1] += $k1 * $error
    if ($State.rls.params_vector[0] -lt 1e-5) { $State.rls.params_vector[0] = 1e-5 }
    if ($State.rls.params_vector[1] -lt 1e-5) { $State.rls.params_vector[1] = 1e-5 }

    $kPhi00 = $k0 * $sampleVector0
    $kPhi01 = $k0 * $sampleVector1
    $kPhi10 = $k1 * $sampleVector0
    $kPhi11 = $k1 * $sampleVector1
    $temp0 = $State.rls.trans_matrix[0] - $kPhi00 * $State.rls.trans_matrix[0] - $kPhi01 * $State.rls.trans_matrix[2]
    $temp1 = $State.rls.trans_matrix[1] - $kPhi00 * $State.rls.trans_matrix[1] - $kPhi01 * $State.rls.trans_matrix[3]
    $temp2 = $State.rls.trans_matrix[2] - $kPhi10 * $State.rls.trans_matrix[0] - $kPhi11 * $State.rls.trans_matrix[2]
    $temp3 = $State.rls.trans_matrix[3] - $kPhi10 * $State.rls.trans_matrix[1] - $kPhi11 * $State.rls.trans_matrix[3]
    $State.rls.trans_matrix[0] = $temp0 / $State.rls.lambda
    $State.rls.trans_matrix[1] = $temp1 / $State.rls.lambda
    $State.rls.trans_matrix[2] = $temp2 / $State.rls.lambda
    $State.rls.trans_matrix[3] = $temp3 / $State.rls.lambda
    $State.rls.update_cnt += 1

    $State.k1 = [Math]::Max([double]$State.rls.params_vector[0], 1e-5)
    $State.k2 = [Math]::Max([double]$State.rls.params_vector[1], 1e-5)
    $State.status.k1 = $State.k1
    $State.status.k2 = $State.k2
    $true
}

function Invoke-PredictPowerMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [double]$Torque, [double]$Speed)
    $Torque * $Speed + $State.k1 * [Math]::Abs($Speed) + $State.k2 * $Torque * $Torque + $State.k3 / 4.0
}

function Invoke-SolveMaxTorqueMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [double]$Speed, [double]$PowerAllocated, [double]$CurrentTorque)

    $a = $State.k2
    $b = $Speed
    $c = $State.k1 * [Math]::Abs($Speed) + $State.k3 / 4.0 - $PowerAllocated
    $delta = $b * $b - 4.0 * $a * $c
    if ($delta -le 0.0) { return -$b / (2.0 * $a) }

    $sqrtDelta = [Math]::Sqrt($delta)
    $torquePos = (-$b + $sqrtDelta) / (2.0 * $a)
    $torqueNeg = (-$b - $sqrtDelta) / (2.0 * $a)
    if ($CurrentTorque -ge 0.0) { return $torquePos }
    $torqueNeg
}

function Invoke-PowerControllerTaskMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State)

    Invoke-EnergyLoopControlMirror -State $State
    $updated = Invoke-PowerRlsUpdateMirror -State $State
    Update-PowerStatistics -State $State
    $updated
}

function Invoke-PowerGetLimitedOutputMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [Parameter(Mandatory = $true)][object[]]$MotorObjects)

    $maxPower = [double]$State.limit.max_power
    $cmdPower = @(0.0, 0.0, 0.0, 0.0)
    $output = @(0.0, 0.0, 0.0, 0.0)
    $sumCmdPower = 0.0
    $sumPositivePower = 0.0

    for ($i = 0; $i -lt 4; $i++) {
        if ($State.motor.disconnect_cnt[$i] -ge $script:PowerControllerConstants.MotorDisconnectTimeout) {
            continue
        }

        $pidOutput = [double]$MotorObjects[$i].pid_output
        $torque = $pidOutput * [double]$State.config.current_scale * [double]$State.config.torque_constant
        $speed = [double]$MotorObjects[$i].current_av
        $cmdPower[$i] = Invoke-PredictPowerMirror -State $State -Torque $torque -Speed $speed
        $sumCmdPower += $cmdPower[$i]

        if ($cmdPower[$i] -gt 0.0) {
            $sumPositivePower += $cmdPower[$i]
        } else {
            $maxPower += -$cmdPower[$i]
        }
    }

    $State.status.cmd_power_sum_w = $sumCmdPower
    if ($sumPositivePower -le $maxPower) {
        for ($i = 0; $i -lt 4; $i++) { $output[$i] = [double]$MotorObjects[$i].pid_output }
        return [pscustomobject]@{
            limited_output        = $output
            cmd_power_sum_w       = $sumCmdPower
            effective_max_power_w = $maxPower
            distribution_mode     = 'passthrough'
        }
    }

    $speedError = @(0.0, 0.0, 0.0, 0.0)
    $sumError = 0.0
    for ($i = 0; $i -lt 4; $i++) {
        $speedError[$i] = [Math]::Abs([double]$MotorObjects[$i].target_av - [double]$MotorObjects[$i].current_av)
        $sumError += $speedError[$i]
    }

    $errorConfidence = 0.0
    $distributionMode = 'proportional'
    if ($sumError -gt $script:PowerControllerConstants.ErrorPowerDistributionThreshold) {
        $errorConfidence = 1.0
        $distributionMode = 'error'
    } elseif ($sumError -gt $script:PowerControllerConstants.PropPowerDistributionThreshold) {
        $errorConfidence = ($sumError - $script:PowerControllerConstants.PropPowerDistributionThreshold) / ($script:PowerControllerConstants.ErrorPowerDistributionThreshold - $script:PowerControllerConstants.PropPowerDistributionThreshold)
        $distributionMode = 'mixed'
    }

    for ($i = 0; $i -lt 4; $i++) {
        if ($cmdPower[$i] -le 0.0) {
            $output[$i] = [double]$MotorObjects[$i].pid_output
            continue
        }

        $weightError = if ($sumError -gt 1e-6) { $speedError[$i] / $sumError } else { 0.25 }
        $weightProp = if ($sumPositivePower -gt 1e-6) { $cmdPower[$i] / $sumPositivePower } else { 0.25 }
        $weight = $errorConfidence * $weightError + (1.0 - $errorConfidence) * $weightProp
        $powerAllocated = $weight * $maxPower
        $currentTorque = [double]$MotorObjects[$i].pid_output * [double]$State.config.current_scale * [double]$State.config.torque_constant
        $maxTorque = Invoke-SolveMaxTorqueMirror -State $State -Speed ([double]$MotorObjects[$i].current_av) -PowerAllocated $powerAllocated -CurrentTorque $currentTorque

        if ([Math]::Abs($currentTorque) -gt 1e-6) {
            $torqueScale = Invoke-FloatConstrain ($maxTorque / $currentTorque) 0.0 1.0
            $output[$i] = [double]$MotorObjects[$i].pid_output * $torqueScale
        } else {
            $output[$i] = [double]$MotorObjects[$i].pid_output
        }
    }

    [pscustomobject]@{
        limited_output        = $output
        cmd_power_sum_w       = $sumCmdPower
        effective_max_power_w = $maxPower
        distribution_mode     = $distributionMode
    }
}

function Invoke-PowerUpdateRefereeDataMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [double]$LimitW, [double]$BufferEnergy, [double]$PowerW)
    $State.referee.limit_w = $LimitW
    $State.referee.buffer_energy = $BufferEnergy
    $State.referee.power_w = $PowerW
}

function Invoke-PowerUpdateCapDataMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [int]$CapEnergy, [int]$CapOnline)
    $State.cap.energy_percent = $CapEnergy
    $State.cap.online = $CapOnline
}

function Invoke-PowerUpdateMotorFeedbackMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [double[]]$MotorSpeeds, [double[]]$MotorTorques)
    for ($i = 0; $i -lt 4; $i++) {
        $State.motor.speeds[$i] = $MotorSpeeds[$i]
        $State.motor.torques[$i] = $MotorTorques[$i]
    }
}

function Invoke-PowerUpdateRefereeOnlineMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [int]$Online, [int]$RobotLevel)
    $State.referee.online = $Online
    if ($RobotLevel -ge 1 -and $RobotLevel -le $script:PowerControllerConstants.MaxRobotLevel) {
        $State.referee.robot_level = $RobotLevel
    }
}

function Invoke-PowerUpdateMotorOnlineMirror {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [int]$MotorIndex, [int]$Online)

    if ($MotorIndex -lt 0 -or $MotorIndex -ge 4) { return }
    $State.motor.online[$MotorIndex] = $Online
    if ($Online) {
        $State.motor.disconnect_cnt[$MotorIndex] = 0
    } elseif ($State.motor.disconnect_cnt[$MotorIndex] -lt $script:PowerControllerConstants.MotorDisconnectTimeout) {
        $State.motor.disconnect_cnt[$MotorIndex] += 1
    }
}

function Apply-PowerControllerStep {
    param([Parameter(Mandatory = $true)][pscustomobject]$State, [Parameter(Mandatory = $true)][object]$Step)

    if ((Test-ObjectProperty $Step 'referee_online') -or (Test-ObjectProperty $Step 'robot_level')) {
        $online = if (Test-ObjectProperty $Step 'referee_online') { [int]$Step.referee_online } else { [int]$State.referee.online }
        $robotLevel = if (Test-ObjectProperty $Step 'robot_level') { [int]$Step.robot_level } else { [int]$State.referee.robot_level }
        Invoke-PowerUpdateRefereeOnlineMirror -State $State -Online $online -RobotLevel $robotLevel
    }

    if ((Test-ObjectProperty $Step 'ref_limit_w') -or (Test-ObjectProperty $Step 'buffer_energy') -or (Test-ObjectProperty $Step 'referee_power_w')) {
        $limitW = if (Test-ObjectProperty $Step 'ref_limit_w') { [double]$Step.ref_limit_w } else { [double]$State.referee.limit_w }
        $bufferEnergy = if (Test-ObjectProperty $Step 'buffer_energy') { [double]$Step.buffer_energy } else { [double]$State.referee.buffer_energy }
        $powerW = if (Test-ObjectProperty $Step 'referee_power_w') { [double]$Step.referee_power_w } else { [double]$State.referee.power_w }
        Invoke-PowerUpdateRefereeDataMirror -State $State -LimitW $limitW -BufferEnergy $bufferEnergy -PowerW $powerW
    }

    if ((Test-ObjectProperty $Step 'cap_energy_percent') -or (Test-ObjectProperty $Step 'cap_online')) {
        $capEnergy = if (Test-ObjectProperty $Step 'cap_energy_percent') { [int]$Step.cap_energy_percent } else { [int]$State.cap.energy_percent }
        $capOnline = if (Test-ObjectProperty $Step 'cap_online') { [int]$Step.cap_online } else { [int]$State.cap.online }
        Invoke-PowerUpdateCapDataMirror -State $State -CapEnergy $capEnergy -CapOnline $capOnline
    }

    if ((Test-ObjectProperty $Step 'motor_speeds') -or (Test-ObjectProperty $Step 'motor_torques')) {
        $motorSpeeds = if (Test-ObjectProperty $Step 'motor_speeds') { @($Step.motor_speeds | ForEach-Object { [double]$_ }) } else { @($State.motor.speeds) }
        $motorTorques = if (Test-ObjectProperty $Step 'motor_torques') { @($Step.motor_torques | ForEach-Object { [double]$_ }) } else { @($State.motor.torques) }
        Invoke-PowerUpdateMotorFeedbackMirror -State $State -MotorSpeeds $motorSpeeds -MotorTorques $motorTorques
    }

    if ((Test-ObjectProperty $Step 'motor_online') -and $null -ne $Step.motor_online) {
        for ($i = 0; $i -lt [Math]::Min(4, $Step.motor_online.Count); $i++) {
            Invoke-PowerUpdateMotorOnlineMirror -State $State -MotorIndex $i -Online ([int]$Step.motor_online[$i])
        }
    }

    if ((Test-ObjectProperty $Step 'motor_disconnect_cycles') -and $null -ne $Step.motor_disconnect_cycles) {
        for ($i = 0; $i -lt [Math]::Min(4, $Step.motor_disconnect_cycles.Count); $i++) {
            $cycles = [int]$Step.motor_disconnect_cycles[$i]
            for ($cycle = 0; $cycle -lt $cycles; $cycle++) {
                Invoke-PowerUpdateMotorOnlineMirror -State $State -MotorIndex $i -Online 0
            }
        }
    }

    if (Test-ObjectProperty $Step 'rls_enabled') { $State.status.rls_enabled = [int]$Step.rls_enabled }

    $iterations = if (Test-ObjectProperty $Step 'run_task_iterations') { [int]$Step.run_task_iterations } else { 0 }
    $updated = $false
    for ($iteration = 0; $iteration -lt $iterations; $iteration++) {
        $updated = (Invoke-PowerControllerTaskMirror -State $State) -or $updated
    }

    $updated
}

function Invoke-PredictPowerGoldenSample {
    param([Parameter(Mandatory = $true)][object]$Sample)

    $state = New-PowerControllerState -Config (Merge-PowerControllerConfig -Override (Get-ObjectValueOrDefault $Sample.input 'config' $null))
    foreach ($step in @(Get-ObjectValueOrDefault $Sample 'pre_steps' @())) {
        [void](Apply-PowerControllerStep -State $state -Step $step)
    }

    [pscustomobject]@{
        predicted_power_w = Invoke-PredictPowerMirror -State $state -Torque ([double]$Sample.input.torque_nm) -Speed ([double]$Sample.input.speed_rad_s)
    }
}

function Invoke-EnergyLoopGoldenSample {
    param([Parameter(Mandatory = $true)][object]$Sample)

    $state = New-PowerControllerState -Config (Merge-PowerControllerConfig -Override (Get-ObjectValueOrDefault $Sample.input 'config' $null))
    foreach ($step in @(Get-ObjectValueOrDefault $Sample 'pre_steps' @())) {
        [void](Apply-PowerControllerStep -State $state -Step $step)
    }

    $inputStep = [pscustomobject]@{
        referee_online      = $Sample.input.referee_online
        robot_level         = $Sample.input.robot_level
        ref_limit_w         = $Sample.input.ref_limit_w
        buffer_energy       = $Sample.input.buffer_energy
        referee_power_w     = (Get-ObjectValueOrDefault $Sample.input 'referee_power_w' 0.0)
        cap_energy_percent  = $Sample.input.cap_energy_percent
        cap_online          = $Sample.input.cap_online
        motor_speeds        = (Get-ObjectValueOrDefault $Sample.input 'motor_speeds' @(0.0, 0.0, 0.0, 0.0))
        motor_torques       = (Get-ObjectValueOrDefault $Sample.input 'motor_torques' @(0.0, 0.0, 0.0, 0.0))
        motor_online        = (Get-ObjectValueOrDefault $Sample.input 'motor_online' @(1, 1, 1, 1))
        run_task_iterations = (Get-ObjectValueOrDefault $Sample.input 'run_task_iterations' 1)
    }
    [void](Apply-PowerControllerStep -State $state -Step $inputStep)

    [pscustomobject]@{
        allowed_power_w = $state.status.allowed_power_w
        upper_limit_w   = $state.status.upper_limit_w
        lower_limit_w   = $state.status.lower_limit_w
        ref_limit_w     = $state.status.ref_limit_w
        hard_limit_w    = $state.status.hard_limit_w
        buffer_feedback = $state.status.buffer_feedback
        error_flags     = $state.status.error_flags
        robot_level     = $state.status.robot_level
    }
}

function Invoke-RlsGoldenSample {
    param([Parameter(Mandatory = $true)][object]$Sample)

    $sampleInput = Get-ObjectValueOrDefault $Sample 'input' ([pscustomobject]@{})
    $state = New-PowerControllerState -Config (Merge-PowerControllerConfig -Override (Get-ObjectValueOrDefault $sampleInput 'config' $null))
    $updated = $false

    foreach ($step in @(Get-ObjectValueOrDefault $Sample 'pre_steps' @())) {
        $updated = (Apply-PowerControllerStep -State $state -Step $step) -or $updated
    }

    if (Test-ObjectProperty $Sample 'steps') {
        foreach ($step in $Sample.steps) {
            $updated = (Apply-PowerControllerStep -State $state -Step $step) -or $updated
        }
    } else {
        $inputStep = [pscustomobject]@{
            referee_online      = (Get-ObjectValueOrDefault $sampleInput 'referee_online' 1)
            robot_level         = (Get-ObjectValueOrDefault $sampleInput 'robot_level' 1)
            ref_limit_w         = (Get-ObjectValueOrDefault $sampleInput 'ref_limit_w' 100.0)
            buffer_energy       = (Get-ObjectValueOrDefault $sampleInput 'buffer_energy' 60.0)
            referee_power_w     = $sampleInput.feedback_power_w
            cap_energy_percent  = (Get-ObjectValueOrDefault $sampleInput 'cap_energy_percent' 0)
            cap_online          = (Get-ObjectValueOrDefault $sampleInput 'cap_online' 0)
            motor_speeds        = $sampleInput.motor_speeds
            motor_torques       = $sampleInput.motor_torques
            run_task_iterations = (Get-ObjectValueOrDefault $sampleInput 'run_task_iterations' 1)
        }
        $updated = (Apply-PowerControllerStep -State $state -Step $inputStep) -or $updated
    }

    [pscustomobject]@{
        k1_after    = $state.status.k1
        k2_after    = $state.status.k2
        rls_updated = [bool]$updated
    }
}

function Invoke-LimitedOutputGoldenSample {
    param([Parameter(Mandatory = $true)][object]$Sample)

    $state = New-PowerControllerState -Config (Merge-PowerControllerConfig -Override (Get-ObjectValueOrDefault $Sample.input 'config' $null))
    foreach ($step in @(Get-ObjectValueOrDefault $Sample 'pre_steps' @())) {
        [void](Apply-PowerControllerStep -State $state -Step $step)
    }

    $inputStep = [pscustomobject]@{
        referee_online          = (Get-ObjectValueOrDefault $Sample.input 'referee_online' 1)
        robot_level             = (Get-ObjectValueOrDefault $Sample.input 'robot_level' 1)
        ref_limit_w             = (Get-ObjectValueOrDefault $Sample.input 'max_power_w' 100.0)
        buffer_energy           = (Get-ObjectValueOrDefault $Sample.input 'buffer_energy' 60.0)
        referee_power_w         = (Get-ObjectValueOrDefault $Sample.input 'referee_power_w' 0.0)
        cap_energy_percent      = (Get-ObjectValueOrDefault $Sample.input 'cap_energy_percent' 0)
        cap_online              = (Get-ObjectValueOrDefault $Sample.input 'cap_online' 0)
        motor_speeds            = (Get-ObjectValueOrDefault $Sample.input 'feedback_motor_speeds' @(0.0, 0.0, 0.0, 0.0))
        motor_torques           = (Get-ObjectValueOrDefault $Sample.input 'feedback_motor_torques' @(0.0, 0.0, 0.0, 0.0))
        motor_online            = (Get-ObjectValueOrDefault $Sample.input 'motor_online' @(1, 1, 1, 1))
        motor_disconnect_cycles = (Get-ObjectValueOrDefault $Sample.input 'motor_disconnect_cycles' $null)
        run_task_iterations     = (Get-ObjectValueOrDefault $Sample.input 'run_task_iterations' 1)
    }
    [void](Apply-PowerControllerStep -State $state -Step $inputStep)

    $result = Invoke-PowerGetLimitedOutputMirror -State $state -MotorObjects @($Sample.input.motor_objs)
    [pscustomobject]@{
        limited_output        = $result.limited_output
        cmd_power_sum_w       = $result.cmd_power_sum_w
        effective_max_power_w = $result.effective_max_power_w
        distribution_mode     = $result.distribution_mode
    }
}

function Assert-ScalarClose {
    param([string]$SampleId, [string]$Field, [double]$Actual, [double]$Expected, [double]$Tolerance)
    if ([Math]::Abs($Actual - $Expected) -gt $Tolerance) {
        throw "Sample '$SampleId' field '$Field' mismatch. Expected=$Expected Actual=$Actual Tolerance=$Tolerance"
    }
}

function Assert-ArrayClose {
    param([string]$SampleId, [string]$Field, [object[]]$Actual, [object[]]$Expected, [double]$Tolerance)

    if ($Actual.Count -ne $Expected.Count) {
        throw "Sample '$SampleId' field '$Field' length mismatch. Expected=$($Expected.Count) Actual=$($Actual.Count)"
    }

    for ($i = 0; $i -lt $Actual.Count; $i++) {
        Assert-ScalarClose -SampleId $SampleId -Field "${Field}[$i]" -Actual ([double]$Actual[$i]) -Expected ([double]$Expected[$i]) -Tolerance $Tolerance
    }
}
