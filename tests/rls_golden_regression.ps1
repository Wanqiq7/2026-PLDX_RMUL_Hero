$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'power_controller_golden_regression_common.ps1')

$paths = Get-PowerControllerRegressionPaths -ScriptPath $MyInvocation.MyCommand.Path
Assert-PowerControllerRegressionAssets -Paths $paths

$fixturePath = Join-Path $paths.RepoRoot 'tests\fixtures\power_controller\rls_samples.json'
$document = Get-PowerControllerFixtureDocument -FixturePath $fixturePath

foreach ($sample in $document.samples) {
    $actual = Invoke-RlsGoldenSample -Sample $sample
    foreach ($field in @('k1_after', 'k2_after')) {
        Assert-ScalarClose -SampleId $sample.sample_id -Field $field `
            -Actual ([double]$actual.$field) `
            -Expected ([double]$sample.expected_state.$field) `
            -Tolerance ([double]$sample.tolerance.$field)
    }

    if ([bool]$actual.rls_updated -ne [bool]$sample.expected_state.rls_updated) {
        throw "Sample '$($sample.sample_id)' field 'rls_updated' mismatch. Expected=$($sample.expected_state.rls_updated) Actual=$($actual.rls_updated)"
    }
}

Write-Output 'PASS: all rls golden samples matched within tolerance'
