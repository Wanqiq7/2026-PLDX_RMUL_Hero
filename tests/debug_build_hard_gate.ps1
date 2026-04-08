$ErrorActionPreference = 'Stop'

# Hard gate 运行时产物策略：
# 1. `.hard-gate-build/` 属于可再生产物，已纳入 .gitignore。
# 2. fresh 构建成功时删除本次临时目录，避免仓库卫生被运行时产物污染。
# 3. fresh 构建失败时保留最近若干份现场，便于复盘 configure/build 日志。
# 4. 启动前先裁剪到 4 份，结束后统一裁剪到 5 份，兼顾排障与磁盘占用。
$HardGateKeepLatestOnStart = 4
$HardGateKeepLatestAfterRun = 5

function Get-CMakeCacheValue {
    param(
        [string]$CachePath,
        [string]$Key
    )

    $match = Select-String -Path $CachePath -Pattern "^${Key}:[^=]*=(.+)$" | Select-Object -First 1
    if ($null -eq $match) {
        throw "Missing ${Key} in ${CachePath}"
    }
    return $match.Matches[0].Groups[1].Value
}

function Format-ArgumentDisplay {
    param(
        [string[]]$Arguments
    )

    return [string]::Join(' ', $Arguments)
}

function Merge-ProcessLogs {
    param(
        [string]$StdOutPath,
        [string]$StdErrPath,
        [string]$MergedPath
    )

    $stdout = ''
    $stderr = ''

    if (Test-Path $StdOutPath) {
        $stdout = Get-Content $StdOutPath -Raw
    }
    if (Test-Path $StdErrPath) {
        $stderr = Get-Content $StdErrPath -Raw
    }

    Set-Content -Path $MergedPath -Value ($stdout + $stderr) -Encoding UTF8
}

function Stop-ProcessTree {
    param(
        [uint32]$RootProcessId
    )

    if ($RootProcessId -eq 0 -or $RootProcessId -eq $PID) {
        return
    }

    try {
        & taskkill.exe /PID $RootProcessId /T /F | Out-Null
    } catch {
        # The process tree may have already exited.
    }
}

function Get-HardGateBuildProcesses {
    param(
        [string]$RepoRoot
    )

    $hardGateRoot = Join-Path $RepoRoot '.hard-gate-build'
    $processNames = @(
        'powershell.exe',
        'pwsh.exe',
        'cmd.exe',
        'cmake.exe',
        'ninja.exe',
        'arm-none-eabi-gcc.exe',
        'arm-none-eabi-g++.exe',
        'arm-none-eabi-ld.exe',
        'cc1.exe',
        'collect2.exe'
    )

    return @(Get-CimInstance Win32_Process | Where-Object {
            $_.Name -in $processNames -and
            $_.CommandLine -and
            $_.CommandLine.IndexOf($hardGateRoot, [System.StringComparison]::OrdinalIgnoreCase) -ge 0
        })
}

function Stop-HardGateBuildProcesses {
    param(
        [string]$RepoRoot
    )

    $staleProcesses = Get-HardGateBuildProcesses -RepoRoot $RepoRoot |
        Sort-Object ProcessId -Descending

    foreach ($process in $staleProcesses) {
        $processId = [uint32]$process.ProcessId
        if ($processId -eq $PID) {
            continue
        }
        Stop-ProcessTree -RootProcessId $processId
    }
}

function Prune-HardGateArtifacts {
    param(
        [string]$RootPath,
        [int]$KeepLatest = 4
    )

    if (-not (Test-Path $RootPath)) {
        return
    }

    $artifactDirs = @(Get-ChildItem -LiteralPath $RootPath -Directory -Force |
            Sort-Object LastWriteTime -Descending)

    if ($artifactDirs.Count -le $KeepLatest) {
        return
    }

    $artifactDirs | Select-Object -Skip $KeepLatest | ForEach-Object {
        Remove-Item -LiteralPath $_.FullName -Recurse -Force
    }
}

function Invoke-LoggedStep {
    param(
        [string]$ExePath,
        [string[]]$Arguments,
        [string]$LogPath,
        [int]$TimeoutSeconds = 900,
        [string[]]$SuccessMarkerPaths = @()
    )

    $stdoutPath = "$LogPath.stdout"
    $stderrPath = "$LogPath.stderr"
    $argumentDisplay = Format-ArgumentDisplay -Arguments $Arguments
    $startProcessArgs = @{
        FilePath               = $ExePath
        ArgumentList           = $Arguments
        PassThru               = $true
        NoNewWindow            = $true
        RedirectStandardOutput = $stdoutPath
        RedirectStandardError  = $stderrPath
    }

    Remove-Item -LiteralPath $LogPath, $stdoutPath, $stderrPath -Force -ErrorAction SilentlyContinue

    $process = $null
    try {
        $process = Start-Process @startProcessArgs
        $null = $process.Handle

        if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
            Merge-ProcessLogs -StdOutPath $stdoutPath -StdErrPath $stderrPath -MergedPath $LogPath
            Stop-ProcessTree -RootProcessId ([uint32]$process.Id)
            $markersSatisfied = ($SuccessMarkerPaths.Count -gt 0)
            foreach ($markerPath in $SuccessMarkerPaths) {
                if (-not (Test-Path $markerPath)) {
                    $markersSatisfied = $false
                    break
                }
            }
            if ($markersSatisfied) {
                Write-Output "Command reached timeout after ${TimeoutSeconds}s but success markers are present: $ExePath $argumentDisplay"
                return
            }
            throw "Command timed out after ${TimeoutSeconds}s: $ExePath $argumentDisplay"
        }

        $process.Refresh()
        Merge-ProcessLogs -StdOutPath $stdoutPath -StdErrPath $stderrPath -MergedPath $LogPath

        if ($process.ExitCode -ne 0) {
            Write-Output "Command failed: $ExePath $argumentDisplay"
            if (Test-Path $LogPath) {
                Get-Content $LogPath | Select-Object -Last 80
            }
            throw "Command failed: $ExePath $argumentDisplay"
        }
    } finally {
        Remove-Item -LiteralPath $stdoutPath, $stderrPath -Force -ErrorAction SilentlyContinue
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$hardGateRoot = Join-Path $repoRoot '.hard-gate-build'
$tmpRoot = Join-Path $hardGateRoot ([guid]::NewGuid().ToString())
$chassisBuild = Join-Path $tmpRoot 'ChassisDebug'
$gimbalBuild = Join-Path $tmpRoot 'GimbalDebug'
$chassisConfigureLog = Join-Path $tmpRoot 'chassis-configure.log'
$chassisBuildLog = Join-Path $tmpRoot 'chassis-build.log'
$gimbalConfigureLog = Join-Path $tmpRoot 'gimbal-configure.log'
$gimbalBuildLog = Join-Path $tmpRoot 'gimbal-build.log'
$chassisCache = Join-Path $repoRoot 'Chassis\build\Debug\CMakeCache.txt'
$cmakeExe = Get-CMakeCacheValue -CachePath $chassisCache -Key 'CMAKE_COMMAND'
$cCompiler = Get-CMakeCacheValue -CachePath $chassisCache -Key 'ARM_NONE_EABI_GCC'
$asmCompiler = $cCompiler
$buildSucceeded = $false

$chassisConfigureArgs = @(
    '-G'
    'Ninja'
    "-DCMAKE_C_COMPILER=$cCompiler"
    "-DCMAKE_ASM_COMPILER=$asmCompiler"
    '-S'
    (Join-Path $repoRoot 'Chassis')
    '-B'
    $chassisBuild
)

$gimbalConfigureArgs = @(
    '-G'
    'Ninja'
    "-DCMAKE_C_COMPILER=$cCompiler"
    "-DCMAKE_ASM_COMPILER=$asmCompiler"
    '-S'
    (Join-Path $repoRoot 'Gimbal')
    '-B'
    $gimbalBuild
)

try {
    New-Item -ItemType Directory -Force $hardGateRoot | Out-Null
    Stop-HardGateBuildProcesses -RepoRoot $repoRoot
    Prune-HardGateArtifacts -RootPath $hardGateRoot -KeepLatest $HardGateKeepLatestOnStart

    New-Item -ItemType Directory -Force $chassisBuild | Out-Null
    New-Item -ItemType Directory -Force $gimbalBuild | Out-Null

    Invoke-LoggedStep -ExePath $cmakeExe -Arguments $chassisConfigureArgs -LogPath $chassisConfigureLog
    Invoke-LoggedStep -ExePath $cmakeExe `
        -Arguments @('--build', $chassisBuild, '-j4') `
        -LogPath $chassisBuildLog `
        -SuccessMarkerPaths @(
            (Join-Path $chassisBuild 'basic_framework.elf'),
            (Join-Path $chassisBuild 'basic_framework.bin'),
            (Join-Path $chassisBuild 'basic_framework.hex')
        )
    Invoke-LoggedStep -ExePath $cmakeExe -Arguments $gimbalConfigureArgs -LogPath $gimbalConfigureLog
    Invoke-LoggedStep -ExePath $cmakeExe `
        -Arguments @('--build', $gimbalBuild, '-j4') `
        -LogPath $gimbalBuildLog `
        -SuccessMarkerPaths @(
            (Join-Path $gimbalBuild 'basic_framework.elf'),
            (Join-Path $gimbalBuild 'basic_framework.bin'),
            (Join-Path $gimbalBuild 'basic_framework.hex')
        )

    $buildSucceeded = $true
    Write-Output 'PASS: debug build hard gate checks'
} finally {
    Stop-HardGateBuildProcesses -RepoRoot $repoRoot

    if ($buildSucceeded -and (Test-Path $tmpRoot)) {
        Remove-Item -LiteralPath $tmpRoot -Recurse -Force
    }
    Prune-HardGateArtifacts -RootPath $hardGateRoot -KeepLatest $HardGateKeepLatestAfterRun
}
