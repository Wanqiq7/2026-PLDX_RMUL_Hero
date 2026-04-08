$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'power_controller_golden_regression_common.ps1')

$paths = Get-PowerControllerRegressionPaths -ScriptPath $MyInvocation.MyCommand.Path
Assert-PowerControllerRegressionAssets -Paths $paths

$fixturePath = Join-Path $paths.RepoRoot 'tests\fixtures\power_controller\limited_output_samples.json'
$document = Get-PowerControllerFixtureDocument -FixturePath $fixturePath

foreach ($sample in $document.samples) {
    $actual = Invoke-LimitedOutputGoldenSample -Sample $sample
    Assert-ArrayClose -SampleId $sample.sample_id -Field 'limited_output' `
        -Actual @($actual.limited_output) `
        -Expected @($sample.expected_output.limited_output) `
        -Tolerance ([double]$sample.tolerance.limited_output)

    foreach ($field in @('cmd_power_sum_w', 'effective_max_power_w')) {
        Assert-ScalarClose -SampleId $sample.sample_id -Field $field `
            -Actual ([double]$actual.$field) `
            -Expected ([double]$sample.expected_output.$field) `
            -Tolerance ([double]$sample.tolerance.$field)
    }

    if ([string]$actual.distribution_mode -ne [string]$sample.expected_output.distribution_mode) {
        throw "Sample '$($sample.sample_id)' field 'distribution_mode' mismatch. Expected=$($sample.expected_output.distribution_mode) Actual=$($actual.distribution_mode)"
    }
}

Write-Output 'PASS: all limited_output golden samples matched within tolerance'
