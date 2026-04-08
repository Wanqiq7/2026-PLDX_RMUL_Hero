$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'power_controller_golden_regression_common.ps1')

$paths = Get-PowerControllerRegressionPaths -ScriptPath $MyInvocation.MyCommand.Path
Assert-PowerControllerRegressionAssets -Paths $paths

$fixturePath = Join-Path $paths.RepoRoot 'tests\fixtures\power_controller\predict_power_samples.json'
$document = Get-PowerControllerFixtureDocument -FixturePath $fixturePath

foreach ($sample in $document.samples) {
    $actual = Invoke-PredictPowerGoldenSample -Sample $sample
    Assert-ScalarClose -SampleId $sample.sample_id -Field 'predicted_power_w' `
        -Actual ([double]$actual.predicted_power_w) `
        -Expected ([double]$sample.expected_output.predicted_power_w) `
        -Tolerance ([double]$sample.tolerance.predicted_power_w)
}

Write-Output 'PASS: all predict_power golden samples matched within tolerance'
