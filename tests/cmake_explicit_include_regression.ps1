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

function Assert-True {
    param(
        [bool]$Condition,
        [string]$Message
    )

    if (-not $Condition) {
        throw $Message
    }
}

function Get-VariableBlock {
    param(
        [string[]]$Lines,
        [string]$VariableName
    )

    $startLineIndex = -1

    for ($index = 0; $index -lt $Lines.Count; $index++) {
        if ($Lines[$index].Trim() -eq "set($VariableName") {
            $startLineIndex = $index + 1
            break
        }
    }

    if ($startLineIndex -lt 0) {
        throw "Missing CMake variable block: ${VariableName}"
    }

    $bodyLines = [System.Collections.Generic.List[string]]::new()
    for ($index = $startLineIndex; $index -lt $Lines.Count; $index++) {
        if ($Lines[$index].Trim() -eq ')') {
            return ($bodyLines -join "`n")
        }

        $bodyLines.Add($Lines[$index])
    }

    throw "Unterminated CMake variable block: ${VariableName}"
}

function Get-VariableEntries {
    param(
        [string[]]$Lines,
        [string]$VariableName
    )

    $block = Get-VariableBlock -Lines $Lines -VariableName $VariableName

    return @(
        $block -split '\r?\n' |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_ -and -not $_.StartsWith('#') }
    )
}

function Get-InlineCommentedVariableEntries {
    param(
        [string[]]$Lines,
        [string]$VariableName
    )

    $block = Get-VariableBlock -Lines $Lines -VariableName $VariableName
    $entries = @()

    foreach ($line in ($block -split '\r?\n')) {
        $trimmedLine = $line.Trim()
        if (-not $trimmedLine) {
            continue
        }

        $match = [regex]::Match(
            $trimmedLine,
            '^(?<path>\$\{CMAKE_SOURCE_DIR\}/[A-Za-z0-9_./-]+)\s+#\s*(?<comment>.+)$'
        )

        if (-not $match.Success) {
            throw "${VariableName} entries must stay one include root per line and carry an inline audit comment. Offending line: [$trimmedLine]"
        }

        $entries += [PSCustomObject]@{
            Path    = $match.Groups['path'].Value
            Comment = $match.Groups['comment'].Value.Trim()
        }
    }

    return @($entries)
}

function Test-CmakeIncludeContract {
    param(
        [string]$CmakeText,
        [string[]]$CmakeLines,
        [System.Collections.Specialized.OrderedDictionary]$ExpectedRootLevelExceptions
    )

    $commonIncludeBlock = Get-VariableBlock -Lines $CmakeLines -VariableName 'COMMON_INCLUDE_DIRS'
    $boardIncludeBlock = Get-VariableBlock -Lines $CmakeLines -VariableName 'BOARD_INCLUDE_DIRS'
    $rootExceptionEntries = Get-InlineCommentedVariableEntries -Lines $CmakeLines -VariableName 'ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS'

    Assert-NoRegex $CmakeText 'function\s*\(\s*include_sub_directories_recursively' 'FAIL: CMake still exposes recursive include directories'
    Assert-NoRegex $CmakeText 'include_sub_directories_recursively\s*\(' 'FAIL: CMake still exposes recursive include directories'
    Assert-Regex $CmakeText 'target_include_directories\s*\(\s*\$\{PROJECT_NAME\}\s+PRIVATE' 'CMake must switch to target_include_directories with explicit include roots.'
    Assert-Regex $CmakeText 'set\s*\(\s*ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS' 'CMake must isolate unavoidable root-level public include exceptions in a dedicated variable.'
    Assert-Regex $CmakeText '#[^\r\n]*audit:root-level-public-include-exceptions[^\r\n]*\r?\nset\s*\(\s*ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS' 'Root-level include exceptions must be documented by an adjacent whitelist comment.'
    Assert-NoRegex $commonIncludeBlock '^\s*\$\{CMAKE_SOURCE_DIR\}/bsp\s*$' 'COMMON_INCLUDE_DIRS must not keep the bsp root path.'
    Assert-NoRegex $commonIncludeBlock '^\s*\$\{CMAKE_SOURCE_DIR\}/modules\s*$' 'COMMON_INCLUDE_DIRS must not keep the modules root path.'
    Assert-NoRegex $commonIncludeBlock '^\s*\$\{CMAKE_SOURCE_DIR\}/application\s*$' 'COMMON_INCLUDE_DIRS must not keep the application root path.'
    Assert-NoRegex $boardIncludeBlock '^\s*\$\{CMAKE_SOURCE_DIR\}/bsp\s*$' 'BOARD_INCLUDE_DIRS must not keep the bsp root path.'
    Assert-NoRegex $boardIncludeBlock '^\s*\$\{CMAKE_SOURCE_DIR\}/modules\s*$' 'BOARD_INCLUDE_DIRS must not keep the modules root path.'
    Assert-NoRegex $boardIncludeBlock '^\s*\$\{CMAKE_SOURCE_DIR\}/application\s*$' 'BOARD_INCLUDE_DIRS must not keep the application root path.'
    Assert-Regex $CmakeText 'target_include_directories\s*\(\s*\$\{PROJECT_NAME\}\s+PRIVATE[\s\S]*?\$\{ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS\}' 'CMake target must include the documented root-level public include exception list explicitly.'
    Assert-True ($rootExceptionEntries.Count -eq $ExpectedRootLevelExceptions.Count) 'ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS must stay a small fixed whitelist.'

    $expectedRootLevelPaths = @($ExpectedRootLevelExceptions.Keys)
    $actualRootLevelPaths = @($rootExceptionEntries | ForEach-Object { $_.Path })
    Assert-True ($null -eq (Compare-Object -ReferenceObject $expectedRootLevelPaths -DifferenceObject $actualRootLevelPaths)) 'ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS must not add unaudited root-level directories.'

    foreach ($expectedPath in $ExpectedRootLevelExceptions.Keys) {
        $matchingEntries = @($rootExceptionEntries | Where-Object { $_.Path -eq $expectedPath })
        Assert-True ($matchingEntries.Count -eq 1) "ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS must contain exactly one audited entry for $expectedPath."
        Assert-True ($matchingEntries[0].Comment.StartsWith($ExpectedRootLevelExceptions[$expectedPath], [System.StringComparison]::Ordinal)) "ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS entry $expectedPath must keep its audited rationale tag."
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$chassisCmakePath = Join-Path $repoRoot 'Chassis\CMakeLists.txt'
$gimbalCmakePath = Join-Path $repoRoot 'Gimbal\CMakeLists.txt'
$utf8Encoding = [System.Text.Encoding]::UTF8
$chassisCmakeText = [System.IO.File]::ReadAllText($chassisCmakePath, $utf8Encoding)
$gimbalCmakeText = [System.IO.File]::ReadAllText($gimbalCmakePath, $utf8Encoding)
$chassisCmakeLines = [System.IO.File]::ReadAllLines($chassisCmakePath, $utf8Encoding)
$gimbalCmakeLines = [System.IO.File]::ReadAllLines($gimbalCmakePath, $utf8Encoding)
$expectedRootLevelExceptions = [ordered]@{
    '${CMAKE_SOURCE_DIR}/bsp'         = 'audit:bsp-root-public'
    '${CMAKE_SOURCE_DIR}/modules'     = 'audit:modules-root-public'
    '${CMAKE_SOURCE_DIR}/application' = 'audit:application-root-public'
}

Test-CmakeIncludeContract -CmakeText $chassisCmakeText -CmakeLines $chassisCmakeLines -ExpectedRootLevelExceptions $expectedRootLevelExceptions
Test-CmakeIncludeContract -CmakeText $gimbalCmakeText -CmakeLines $gimbalCmakeLines -ExpectedRootLevelExceptions $expectedRootLevelExceptions

Assert-Regex $chassisCmakeText '\$\{CMAKE_SOURCE_DIR\}/modules/algorithm' 'Chassis CMake must expose the algorithm root explicitly.'
Assert-Regex $chassisCmakeText '\$\{CMAKE_SOURCE_DIR\}/modules/power_controller' 'Chassis CMake must expose the power_controller public header root explicitly.'
Assert-Regex $chassisCmakeText '\$\{CMAKE_SOURCE_DIR\}/application/chassis' 'Chassis CMake must expose the chassis application header root explicitly.'

Assert-Regex $gimbalCmakeText '\$\{CMAKE_SOURCE_DIR\}/modules/algorithm' 'Gimbal CMake must expose the algorithm root explicitly.'
Assert-Regex $gimbalCmakeText '\$\{CMAKE_SOURCE_DIR\}/modules/vision_comm' 'Gimbal CMake must expose the vision_comm module header root explicitly.'
Assert-Regex $gimbalCmakeText '\$\{CMAKE_SOURCE_DIR\}/application/gimbal' 'Gimbal CMake must expose the gimbal application header root explicitly.'
Assert-Regex $gimbalCmakeText '\$\{CMAKE_SOURCE_DIR\}/application/vision' 'Gimbal CMake must expose the vision application header root explicitly.'

Write-Output 'PASS: cmake explicit include regression checks'
