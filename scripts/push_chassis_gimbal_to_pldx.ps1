param(
    [string]$RepoRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$TargetRepoUrl = '',
    [string]$TempRepoRoot = '',
    [string]$TargetBranch = 'main'
)

$ErrorActionPreference = 'Stop'

function Invoke-Git {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Arguments,
        [string]$WorkingDirectory = $RepoRoot
    )

    & git -C $WorkingDirectory @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "git command failed: git -C `"$WorkingDirectory`" $($Arguments -join ' ')"
    }
}

function Get-RepoStatusLines {
    param([string]$WorkingDirectory)

    $lines = & git -C $WorkingDirectory status --short
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to read git status for $WorkingDirectory"
    }

    @($lines | Where-Object { $_ -ne '' })
}

function Remove-IfExists {
    param([string]$LiteralPath)

    if (Test-Path -LiteralPath $LiteralPath) {
        Remove-Item -LiteralPath $LiteralPath -Recurse -Force
    }
}

if ([string]::IsNullOrWhiteSpace($TempRepoRoot)) {
    $TempRepoRoot = Join-Path $RepoRoot '.push-temp\PLDX_Hero'
}

if ([string]::IsNullOrWhiteSpace($TargetRepoUrl)) {
    $configuredTarget = (& git -C $RepoRoot config --get remote.pldx.url)
    if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($configuredTarget)) {
        $TargetRepoUrl = $configuredTarget.Trim()
    } else {
        $TargetRepoUrl = 'https://github.com/Wanqiq7/PLDX_Hero.git'
    }
}

$sourceStatus = Get-RepoStatusLines -WorkingDirectory $RepoRoot
$nonIgnoreOnly = @(
    $sourceStatus | Where-Object {
        $_ -notmatch '^\?\? \.push-temp/' -and
        $_ -notmatch '^\?\? \.merge-backup/' -and
        $_ -notmatch '^\?\? findings\.md$' -and
        $_ -notmatch '^\?\? progress\.md$' -and
        $_ -notmatch '^\?\? tmp_'
    }
)

if ($nonIgnoreOnly.Count -gt 0) {
    throw "Source repo has uncommitted changes. Commit or clean them before syncing to PLDX_Hero.`n$($nonIgnoreOnly -join "`n")"
}

$sourceHead = (& git -C $RepoRoot rev-parse --short HEAD).Trim()
if ($LASTEXITCODE -ne 0) {
    throw 'Failed to resolve source HEAD.'
}

$archivePath = Join-Path $RepoRoot '.push-temp\pldx-sync.zip'

try {
    Remove-IfExists -LiteralPath $TempRepoRoot
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $TempRepoRoot) | Out-Null
    Invoke-Git -WorkingDirectory $RepoRoot -Arguments @('clone', '--depth', '1', $TargetRepoUrl, $TempRepoRoot)

    Remove-IfExists -LiteralPath $archivePath
    Invoke-Git -WorkingDirectory $RepoRoot -Arguments @('archive', '--format=zip', "--output=$archivePath", 'HEAD', 'Chassis', 'Gimbal')

    Remove-IfExists -LiteralPath (Join-Path $TempRepoRoot 'Chassis')
    Remove-IfExists -LiteralPath (Join-Path $TempRepoRoot 'Gimbal')
    Expand-Archive -LiteralPath $archivePath -DestinationPath $TempRepoRoot -Force
    Remove-IfExists -LiteralPath $archivePath

    $targetStatus = Get-RepoStatusLines -WorkingDirectory $TempRepoRoot
    if ($targetStatus.Count -eq 0) {
        Write-Output "PLDX_Hero is already up to date with source HEAD $sourceHead."
        return
    }

    Invoke-Git -WorkingDirectory $TempRepoRoot -Arguments @('add', 'Chassis', 'Gimbal')
    Invoke-Git -WorkingDirectory $TempRepoRoot -Arguments @('commit', '-m', "sync: update Chassis and Gimbal from HeroCode $sourceHead")
    Invoke-Git -WorkingDirectory $TempRepoRoot -Arguments @('push', 'origin', "HEAD:$TargetBranch")
    Write-Output "Pushed Chassis and Gimbal to PLDX_Hero/$TargetBranch from source HEAD $sourceHead."
}
finally {
    Remove-IfExists -LiteralPath $archivePath
    Remove-IfExists -LiteralPath $TempRepoRoot
}
