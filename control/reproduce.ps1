# reproduce.ps1 — S02 Rocket Orbit-Return: unattended full-mission reproduction
# =============================================================================
# Prereqs (checked below):
#   * Windows 10/11 + Besiege (The Splintered Sea / The Broken Beyond DLCs)
#   * BuildArena 2.0 repo checked out, `uv` on PATH, `uv run python -m control.run_example --help` works
#   * BuildArena ToolKit MOD active, Besiege installed via Steam
#   * Besiege already launched and sitting on the LONE ORB sandbox with the
#     rebuilt machine loaded (the S02 rules forbid artificial piloting: a human
#     may set up the scene, but not touch keys during the flight).
#   * A free Steam port for the ToolKit websocket (BuildArena preset picks one).
#
# Behavior: rebuild machine.bsg (prepare-only), then run the mission controller
# end-to-end with no key input.  All artifacts land under control/runs/... and
# are also mirrored next to this script in ./run_output/ for packaging.

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$example = "s02_rocket_orbit_return"
# Locate the BuildArena repo root (walk up until pyproject.toml is found).
# NOTE: this copy lives in control/submission/s02_rocket_orbit_return/control/,
# so the examples-layout "..\..\.." shortcut does NOT apply here.
$repo = $here
while ($true) {
    if (Test-Path (Join-Path $repo "pyproject.toml")) { break }
    $parent = Split-Path -Parent $repo
    if ($parent -eq $repo) { throw "Could not locate BuildArena repo root from $here" }
    $repo = $parent
}
$controlDir = Join-Path $repo "control"

Write-Host "== BuildArena S02 reproduction: $example ==" -ForegroundColor Cyan
Write-Host "repo   : $repo"
Write-Host "example: $here"

# --- 0. Preflight -----------------------------------------------------------
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    throw "uv not found on PATH. Install https://docs.astral.sh/uv/ first."
}
$uvVersion = (& uv --version 2>$null | Select-Object -First 1)
Write-Host "uv: $uvVersion"

# --- 1. Prepare (rebuild machine.json -> machine.bsg, validate controller) --
Write-Host "`n[1/4] Rebuilding machine.bsg + validating controller..." -ForegroundColor Cyan
Push-Location $repo
try {
    & uv run python -m control.run_example $example --prepare-only
    if ($LASTEXITCODE -ne 0) { throw "prepare-only failed (exit $LASTEXITCODE)" }
} finally {
    Pop-Location
}

# --- 2. Locate the prepared run directory ----------------------------------
$env:BUILDARENA_EXAMPLE = $example
$runDir = Join-Path $controlDir "runs\s02_rocket_orbit_return"
if (-not (Test-Path (Join-Path $runDir "mission.py"))) {
    # prepare_example may place artifacts under the example dir; discover it.
    $candidates = @(
        $runDir,
        (Join-Path $here "prepared"),
        (Join-Path $here "run")
    )
    $found = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
    if ($found) { $runDir = $found }
}
Write-Host "run dir: $runDir"

# --- 3. Launch the mission (no human input required) -----------------------
Write-Host "`n[2/4] Launching mission controller (autopilot only)..." -ForegroundColor Cyan
# Besiege must already be on the LONE ORB scene with the machine loaded.
$env:HEAVY_LAUNCHER_POLICY = "mission"
$env:HEAVY_LAUNCHER_REQUIRE_SUCCESS = "1"

Push-Location $repo
try {
    & uv run python -m control.run_example $example
    $code = $LASTEXITCODE
    if ($code -ne 0) {
        Write-Host "WARN: controller exited with code $code (see mission_summary.json)." -ForegroundColor Yellow
    }
} finally {
    Pop-Location
}

# --- 4. Package the submission artifacts -----------------------------------
Write-Host "`n[3/4] Packaging submission artifacts..." -ForegroundColor Cyan
$out = Join-Path $here "run_output"
New-Item -ItemType Directory -Force -Path $out | Out-Null
$summarySource = Get-ChildItem -Path $controlDir -Recurse -Filter "mission_summary.json" |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($summarySource) {
    Copy-Item $summarySource.FullName (Join-Path $out "mission_summary.json") -Force
    Write-Host "mission_summary.json -> $out"
} else {
    Write-Host "WARN: mission_summary.json not found" -ForegroundColor Yellow
}
$trajSource = Get-ChildItem -Path $controlDir -Recurse -Filter "trajectory.csv" |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($trajSource) {
    Copy-Item $trajSource.FullName (Join-Path $out "trajectory.csv") -Force
    Write-Host "trajectory.csv -> $out"
}

Write-Host "`n[4/4] Done." -ForegroundColor Green
Write-Host "Summary of run:"
if ($summarySource) {
    $summary = Get-Content $summarySource.FullName -Raw | ConvertFrom-Json
    Write-Host ("  done={0}  stage={1}  abort={2}" -f $summary.done, $summary.final_stage, $summary.abort_reason)
    Write-Host ("  sweep_deg={0}  min_coast_peri={1}  orbit_gate={2}" -f [math]::Round($summary.sweep_deg,0), [math]::Round($summary.min_coast_peri,1), $summary.orbit_gate_passed)
    Write-Host ("  integrity={0}  settled_hold={1}  recovery_ok={2}" -f $summary.final_machine_integrity, $summary.settled_hold_reached, $summary.recovery_ok)
}
exit 0
