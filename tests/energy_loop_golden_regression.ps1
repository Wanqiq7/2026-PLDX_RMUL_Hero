$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'power_controller_golden_regression_common.ps1')

$paths = Get-PowerControllerRegressionPaths -ScriptPath $MyInvocation.MyCommand.Path
Assert-PowerControllerRegressionAssets -Paths $paths

$fixturePath = Join-Path $paths.RepoRoot 'tests\fixtures\power_controller\energy_loop_samples.json'
$document = Get-PowerControllerFixtureDocument -FixturePath $fixturePath

foreach ($sample in $document.samples) {
    $actual = Invoke-EnergyLoopGoldenSample -Sample $sample
    foreach ($field in @('allowed_power_w', 'upper_limit_w', 'lower_limit_w', 'ref_limit_w', 'hard_limit_w', 'buffer_feedback')) {
        Assert-ScalarClose -SampleId $sample.sample_id -Field $field `
            -Actual ([double]$actual.$field) `
            -Expected ([double]$sample.expected_state.$field) `
            -Tolerance ([double]$sample.tolerance.$field)
    }

    if ([int]$actual.error_flags -ne [int]$sample.expected_state.error_flags) {
        throw "Sample '$($sample.sample_id)' field 'error_flags' mismatch. Expected=$($sample.expected_state.error_flags) Actual=$($actual.error_flags)"
    }

    if ([int]$actual.robot_level -ne [int]$sample.expected_state.robot_level) {
        throw "Sample '$($sample.sample_id)' field 'robot_level' mismatch. Expected=$($sample.expected_state.robot_level) Actual=$($actual.robot_level)"
    }
}

Write-Output 'PASS: all energy_loop golden samples matched within tolerance'
