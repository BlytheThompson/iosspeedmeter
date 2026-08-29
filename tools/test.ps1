# Run the PerformanceTimerCore test suite and print a compact summary.
#
# Writes the full log to build/test.log so failures can be inspected without
# re-running, and avoids piping the live stream through Select-Object (which
# breaks the pipe and corrupts $LASTEXITCODE).
param([string]$Filter = "")

$ErrorActionPreference = "Continue"
Set-Location $PSScriptRoot\..
New-Item -ItemType Directory -Force -Path build | Out-Null
$log = "build\test.log"

$swiftArgs = "test"
if ($Filter -ne "") { $swiftArgs = "test --filter $Filter" }

& cmd.exe /c "tools\swift.cmd $swiftArgs" 2>&1 | Out-File -FilePath $log -Encoding utf8

# Assertion failures also contain "error:", so classify them FIRST and exclude
# them before looking for genuine compile errors.
$assertionPattern = "XCTAssert|XCTFail|XCTUnwrap|Asynchronous wait failed"
$failures = Select-String -Path $log -Pattern $assertionPattern

$compileErrors = Select-String -Path $log -Pattern "error:" |
    Where-Object { $_.Line -notmatch $assertionPattern }

if ($compileErrors) {
    Write-Output "=== COMPILE ERRORS ==="
    $compileErrors | Select-Object -First 25 | ForEach-Object { $_.Line.Trim() }
    Write-Output "RESULT: BUILD FAILED"
    exit 1
}

if ($failures) {
    Write-Output "=== TEST FAILURES ==="
    $failures | Select-Object -First 40 | ForEach-Object { $_.Line.Trim() }
}

$summary = Select-String -Path $log -Pattern "Executed \d+ tests" | Select-Object -Last 1
if ($summary) {
    Write-Output ("RESULT: " + $summary.Line.Trim())
} else {
    Write-Output "RESULT: no test summary found (see $log)"
    exit 1
}

if ($summary.Line -match "with 0 failures") { exit 0 } else { exit 1 }
