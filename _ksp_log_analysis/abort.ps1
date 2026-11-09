<#
.SYNOPSIS
  Abort a live MechJeb Sol autotest run.

.DESCRIPTION
  The MechJebAutotest harness has NO abort file / hotkey / Finish("abort") hook.
  Mid-flight options:
    1) Soft (best-effort): remove autotest.cfg so a *new* scene load will not re-arm,
       and try to bring KSP to the foreground note — does NOT stop the current Drive loop.
    2) Hard (reliable): Stop-Process KSP_x64 (and any other KSP*). Use this to abort now.

  After hard abort, the CSV may lack a RESULT line; monitor.ps1 will FAIL ksp-exited.

.EXAMPLE
  .\abort.ps1              # hard kill Sol KSP
  .\abort.ps1 -SoftOnly    # only disarm autotest.cfg (will not stop flight)
#>
param(
  [string]$KspDir = "C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program-Sol-V0.9.2",
  [switch]$SoftOnly,
  [switch]$DisarmCfg
)

$cfg = Join-Path $KspDir "GameData\MechJeb2\autotest.cfg"
$cfgBak = Join-Path $KspDir "GameData\MechJeb2\autotest.cfg.abort-disabled"

Write-Host "Abort target: $KspDir"

if ($DisarmCfg -or $SoftOnly) {
  if (Test-Path $cfg) {
    Move-Item -LiteralPath $cfg -Destination $cfgBak -Force
    Write-Host "Disarmed harness: moved autotest.cfg -> autotest.cfg.abort-disabled"
    Write-Host "Re-arm later: Move-Item '$cfgBak' '$cfg'"
  } else {
    Write-Host "autotest.cfg already absent (harness idle on next load)."
  }
}

$procs = @(Get-Process | Where-Object { $_.ProcessName -match '^(KSP_x64|KSP|KerbalSpaceProgram)$' })
if (-not $procs) {
  Write-Host "No KSP process running."
  exit 0
}

$procs | Format-Table Id, ProcessName, Path -AutoSize

if ($SoftOnly) {
  Write-Host "SoftOnly: left KSP running (harness has no in-flight abort API)."
  Write-Host "To kill: .\abort.ps1"
  exit 0
}

Write-Host "Hard abort: Stop-Process..."
$procs | Stop-Process -Force
Start-Sleep -Seconds 2
$left = @(Get-Process | Where-Object { $_.ProcessName -match '^(KSP_x64|KSP|KerbalSpaceProgram)$' })
if ($left) {
  Write-Host "Still alive; Stop-Process -Force again..."
  $left | Stop-Process -Force
  Start-Sleep -Seconds 2
}
$left = @(Get-Process | Where-Object { $_.ProcessName -match '^(KSP_x64|KSP|KerbalSpaceProgram)$' })
if ($left) {
  Write-Host "FAIL: KSP still running:" -ForegroundColor Red
  $left | Format-Table Id, ProcessName
  exit 1
}
Write-Host "KSP stopped."

# Optional: disarm so an accidental relaunch without intending autotest is safer
if (-not (Test-Path $cfgBak) -and (Test-Path $cfg)) {
  Write-Host "(tip) To prevent auto re-arm on next launch: .\abort.ps1 -DisarmCfg -SoftOnly"
}
