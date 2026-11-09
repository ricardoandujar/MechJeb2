<#
.SYNOPSIS
  Live / incremental monitor for MechJeb Sol autotest (CSV + KSP.log).

.EXAMPLE
  .\monitor.ps1
  .\monitor.ps1 -SinceUt 1100
  .\monitor.ps1 -SinceWall 01:41:00
  .\monitor.ps1 -Once
  .\monitor.ps1 -CsvPath "...\20261109_012236.csv" -Once
#>
param(
  [string]$KspDir = "C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program-Sol-V0.9.2",
  [string]$CsvPath = "",
  [double]$SinceUt = -1,
  [string]$SinceWall = "",
  [switch]$Once,
  [int]$PollMs = 1000
)

$ErrorActionPreference = "Continue"
$autotestDir = Join-Path $KspDir "GameData\MechJeb2\autotest"
$logPath     = Join-Path $KspDir "KSP.log"
$script:logOffset = 0
$script:pastFailUt = -1
$script:seenResult = $false
$script:prevStepKey = $null
$script:csvRowsConsumed = 0
$script:minDist = [double]::PositiveInfinity
$script:minDistUt = -1
$script:distRisingSince = -1
$script:header = $null
$script:col = @{}

function Get-NewestCsv {
  Get-ChildItem (Join-Path $autotestDir "*.csv") -EA SilentlyContinue |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
}

function Parse-CsvLine([string]$line) {
  $cols = New-Object System.Collections.Generic.List[string]
  $sb = New-Object System.Text.StringBuilder
  $inQ = $false
  for ($i = 0; $i -lt $line.Length; $i++) {
    $ch = $line[$i]
    if ($ch -eq [char]34) {
      if ($inQ -and ($i + 1) -lt $line.Length -and $line[$i+1] -eq [char]34) {
        [void]$sb.Append([char]34); $i++; continue
      }
      $inQ = -not $inQ; continue
    }
    if ($ch -eq [char]44 -and -not $inQ) {
      $cols.Add($sb.ToString()); [void]$sb.Clear(); continue
    }
    [void]$sb.Append($ch)
  }
  $cols.Add($sb.ToString())
  return ,$cols.ToArray()
}

function Set-Header([string]$hdr) {
  $script:header = $hdr
  $names = Parse-CsvLine $hdr
  $script:col = @{}
  for ($i = 0; $i -lt $names.Length; $i++) {
    $n = $names[$i] -replace '^\ufeff',''
    $script:col[$n] = $i
  }
}

function Get-C([string[]]$cols, [string]$name) {
  if (-not $script:col.ContainsKey($name)) { return "" }
  $i = $script:col[$name]
  if ($i -ge $cols.Length) { return "" }
  return $cols[$i]
}

function Get-D([string]$s) {
  if ([string]::IsNullOrWhiteSpace($s)) { return $null }
  $x = 0.0
  if ([double]::TryParse($s, [Globalization.NumberStyles]::Float,
      [Globalization.CultureInfo]::InvariantCulture, [ref]$x)) { return $x }
  return $null
}

function Write-Mode($ut,$step,$state,$dist,$agl,$vs,$hs,$apa,$thr,$extra) {
  $s = "MODE ut={0:F1} step={1} dist={2} AGL={3} vs={4} hs={5} ApA={6} thr={7}" -f `
    $ut,$step,$dist,$agl,$vs,$hs,$apa,$thr
  if ($extra) { $s += " $extra" }
  Write-Host $s
}

function Write-Fail([string]$why, [string]$detail) {
  Write-Host ("FAIL {0} :: {1}" -f $why, $detail) -ForegroundColor Red
}

function Handle-DataRow([string]$line) {
  if ($line -match '^RESULT') {
    Write-Host "RESULT $line"
    if ($line -match 'outcome=(crash|timeout)') { Write-Fail $Matches[1] $line }
    $script:seenResult = $true
    return
  }
  $cols = Parse-CsvLine $line
  $ut = Get-D (Get-C $cols 'time')
  if ($null -eq $ut) { return }
  if ($SinceUt -ge 0 -and $ut -lt $SinceUt) { return }

  $step = Get-C $cols 'step'
  $state = Get-C $cols 'autopilot_state'
  $alt = Get-D (Get-C $cols 'altitude')
  $ter = Get-D (Get-C $cols 'terrain_altitude')
  $agl = if ($null -ne $alt -and $null -ne $ter) { $alt - $ter } else { $null }
  $vs = Get-D (Get-C $cols 'vertical_speed')
  $hs = Get-D (Get-C $cols 'horizontal_speed')
  $dist = Get-D (Get-C $cols 'distance_to_target')
  $apa = Get-D (Get-C $cols 'ApA')
  $thr = Get-D (Get-C $cols 'throttle')
  $padDh = Get-C $cols 'padDh'
  $padDt = Get-C $cols 'padDt'
  $hoopCenter = Get-C $cols 'hoop_center'
  $predH = Get-C $cols 'pred_h'
  $hoopMiss = Get-C $cols 'hoop_miss'
  $frac = Get-C $cols 'frac'
  $past = Get-C $cols 'apoapsis_past_pad'
  $burnStarted = Get-C $cols 'burn_started'
  $aPlan = Get-C $cols 'a_plan'
  $aClip = Get-C $cols 'a_clip'
  $handoff = Get-C $cols 'handoff'
  $burn = Get-C $cols 'burn_axis'

  $stShort = if ($state.Length -gt 48) { $state.Substring(0,48) } else { $state }
  $stepKey = "$step|$stShort"
  $extra = @()
  if ($handoff) { $extra += "handoff=$handoff" }
  if ($padDh) { $extra += "padDh=$padDh padDt=$padDt frac=$frac past=$past" }
  if ($aPlan) { $extra += "aPlan=$aPlan aClip=$aClip burnStarted=$burnStarted burn=$burn" }
  if ($hoopCenter) { $extra += "hoopCenter=$hoopCenter predH=$predH miss=$hoopMiss" }
  $extraS = [string]::Join(' ', $extra)

  if ($stepKey -ne $script:prevStepKey) {
    Write-Mode $ut $step $stShort `
      $(if ($null -ne $dist) { "{0:F0}" -f $dist } else { "?" }) `
      $(if ($null -ne $agl)  { "{0:F0}" -f $agl }  else { "?" }) `
      $(if ($null -ne $vs)   { "{0:F1}" -f $vs }   else { "?" }) `
      $(if ($null -ne $hs)   { "{0:F1}" -f $hs }   else { "?" }) `
      $(if ($null -ne $apa)  { "{0:F0}" -f $apa }  else { "?" }) `
      $(if ($null -ne $thr)  { "{0:F3}" -f $thr }  else { "?" }) `
      $extraS
    $script:prevStepKey = $stepKey
  }

  if ($null -ne $dist) {
    if ($dist + 1.0 -lt $script:minDist) {
      $script:minDist = $dist; $script:minDistUt = $ut; $script:distRisingSince = -1
    } elseif ($script:minDistUt -ge 0 -and $dist -gt ($script:minDist + 50)) {
      if ($script:distRisingSince -lt 0) { $script:distRisingSince = $ut }
      elseif (($ut - $script:distRisingSince) -gt 60 -and $script:distRisingSince -lt 1e8) {
        Write-Fail "pad-receding" ("dist={0:F0} min={1:F0} risingFor={2:F0}s ut={3:F1}" -f $dist,$script:minDist,($ut-$script:distRisingSince),$ut)
        $script:distRisingSince = 1e9
      }
    }
  }

  if ($past -eq '1' -and $step -like 'STATE_*') {
    if ($script:pastFailUt -lt 0 -or ($ut - $script:pastFailUt) -gt 30) {
      Write-Fail "ap-past-no-lb" ("step={0} dist={1} padDh={2} frac={3} ut={4:F1}" -f $step,$dist,$padDh,$frac,$ut)
      $script:pastFailUt = $ut
    }
  }

  $aClipN = Get-D $aClip
  if ($null -ne $agl -and $null -ne $vs -and $vs -lt -1 -and $null -ne $aClipN -and $aClipN -gt 0.1) {
    $hStop = ($vs * $vs) / (2.0 * $aClipN)
    if ($agl -lt $hStop) {
      Write-Fail "agl-below-stop" ("AGL={0:F0} hStop={1:F0} vs={2:F1} aClip={3} ut={4:F1}" -f $agl,$hStop,$vs,$aClip,$ut)
    }
  }

  if ($handoff -match 'TargetSubOrbit.*LandingBurn' -or $handoff -match 'TSO.*LB') {
    $hc = Get-D $hoopCenter
    $ph = Get-D $predH
    $hm = Get-D $hoopMiss
    $bad = $false
    $why = @()
    if ($past -eq '1') { $bad = $true; $why += 'apPast' }
    if ($null -ne $hc -and $null -ne $ph -and [Math]::Abs($ph - $hc) -gt 600) {
      $bad = $true; $why += ("|predH-hoop|={0:F0}" -f [Math]::Abs($ph - $hc))
    }
    if ($null -ne $hm -and $hm -gt 2000) { $bad = $true; $why += "miss=$hm" }
    # Legacy CSVs without hoop columns: fall back to |padDh|>600 as rough reject signal
    if ($null -eq $hc -and $null -eq $ph) {
      $pd = Get-D $padDh
      if (($null -ne $pd -and [Math]::Abs($pd) -gt 600) -or $past -eq '1') {
        $bad = $true; $why += "legacy-padDh=$padDh"
      }
    }
    if ($bad) {
      Write-Fail "handoff-hoop-bad" ("hoopCenter={0} predH={1} miss={2} past={3} dist={4} ut={5:F1} why={6}" -f $hoopCenter,$predH,$hoopMiss,$past,$dist,$ut,($why -join ','))
    } else {
      Write-Host ("HOOP ut={0:F1} hoopCenter={1} predH={2} miss={3} past={4} dist={5}" -f $ut,$hoopCenter,$predH,$hoopMiss,$past,$dist)
    }
  }
}

function Sync-Csv([string]$path) {
  if (-not (Test-Path -LiteralPath $path)) { return }
  $lines = @(Get-Content -LiteralPath $path -EA SilentlyContinue)
  if ($lines.Count -eq 0) { return }
  if ($null -eq $script:header) { Set-Header $lines[0]; $script:csvRowsConsumed = 1 }
  while ($script:csvRowsConsumed -lt $lines.Count) {
    Handle-DataRow $lines[$script:csvRowsConsumed]
    $script:csvRowsConsumed++
  }
}

function Pass-Wall([string]$ln) {
  if (-not $SinceWall) { return $true }
  if ($ln -match '\[(?:LOG|WRN|ERR) (\d{2}:\d{2}:\d{2})') {
    try {
      return ([TimeSpan]::Parse($Matches[1]) -ge [TimeSpan]::Parse($SinceWall))
    } catch { return $true }
  }
  return $true
}

function Tail-Log {
  if (-not (Test-Path $logPath)) { return }
  $fs = [IO.File]::Open($logPath, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]::ReadWrite)
  try {
    if ($script:logOffset -gt $fs.Length) { $script:logOffset = 0 }
    [void]$fs.Seek($script:logOffset, [IO.SeekOrigin]::Begin)
    $sr = New-Object IO.StreamReader($fs)
    while (($ln = $sr.ReadLine()) -ne $null) {
      $script:logOffset = $fs.Position
      if (-not (Pass-Wall $ln)) { continue }
      if ($ln -match 'crashed through terrain|Exploded!!') {
        Write-Fail "crash" $ln.Substring(0, [Math]::Min(220, $ln.Length))
      }
      if ($ln -match '\[MechJebAutotest\] RESULT outcome=') {
        $r = ($ln -replace '^.*RESULT ','RESULT ')
        Write-Host "RESULT-LOG $r"
        if ($r -match 'outcome=(crash|timeout)') { Write-Fail $Matches[1] $r }
        $script:seenResult = $true
      }
      if ($ln -match '\[TSO->LB\] direct handoff:') {
        Write-Host ("HANDOFF-LOG {0}" -f $ln.Substring(0, [Math]::Min(320, $ln.Length)))
        $hc = $null; $ph = $null; $ms = $null
        if ($ln -match 'hoopCenter=([-\d.]+)') { $hc = [double]$Matches[1] }
        if ($ln -match 'predH=([-\d.]+)') { $ph = [double]$Matches[1] }
        if ($ln -match 'miss=([-\d.]+)') { $ms = [double]$Matches[1] }
        if ($null -ne $hc -and $null -ne $ph) {
          if ([Math]::Abs($ph - $hc) -gt 600 -or ($null -ne $ms -and $ms -gt 2000) -or $ln -match 'apPast=1') {
            Write-Fail "handoff-hoop-bad" ("log hoopCenter={0} predH={1} miss={2}" -f $hc,$ph,$ms)
          } else {
            Write-Host ("HOOP-LOG hoopCenter={0} predH={1} miss={2}" -f $hc,$ph,$ms)
          }
        } elseif ($ln -match 'ok=0|hoop-fail') {
          Write-Fail "handoff-hoop-bad" $ln.Substring(0, [Math]::Min(220, $ln.Length))
        }
        if ($ln -match 'step=STATE_PROGRADE' -and $ln -match 'dist=(\d+)') {
          if ([int]$Matches[1] -gt 30000) {
            Write-Fail "early-prograde-handoff" ("PROGRADE handoff dist={0}" -f $Matches[1])
          }
        }
      }
      if ($ln -match '\[TSO->LB\] waiting: hoop-fail') {
        Write-Host ("HOOP-WAIT {0}" -f $ln.Substring(0, [Math]::Min(280, $ln.Length)))
      }
      if ($ln -match '\[ShouldStartBurn2:agl\]') {
        Write-Host ("IGNITE-AGL {0}" -f $ln.Substring(0, [Math]::Min(220, $ln.Length)))
      }
      if ($ln -match 'burnStarted=True' -and $ln -match 'TimeWarp|warp rate|WarpRate') {
        Write-Fail "burn-while-warp" $ln.Substring(0, [Math]::Min(220, $ln.Length))
      }
      # Physics-warp while burn: LB-1Hz after agl latch often still under warp â€” flag if agl ignite then crash soon
      if ($ln -match 'RESULT outcome=timeout') {
        Write-Fail "timeout" $ln.Substring(0, [Math]::Min(220, $ln.Length))
      }
      if ($ln -match 'Timeout extended') {
        Write-Host ("TIMEOUT-EXT {0}" -f $ln.Substring(0, [Math]::Min(200, $ln.Length)))
      }
    }
    $sr.Dispose()
  } finally { $fs.Close() }
}

# --- main ---
if (-not $CsvPath) {
  $c = Get-NewestCsv
  if (-not $c) { Write-Error "No CSV in $autotestDir"; exit 1 }
  $CsvPath = $c.FullName
}
Write-Host "CSV  $CsvPath"
Write-Host "LOG  $logPath"
Write-Host "NOTE Hoop: |predH - hoopCenter|<=600, miss<=2000; hoopCenter=max(ComputeOptimal gate, terrain clearNeed)."
Write-Host "----"

# Start following log from EOF unless SinceWall set (then scan from start of file for matching stamps)
if ($SinceWall) { $script:logOffset = 0 }
elseif (Test-Path $logPath) { $script:logOffset = (Get-Item $logPath).Length }

Sync-Csv $CsvPath
Tail-Log

if ($Once) { Write-Host "DONE once-shot."; exit 0 }

Write-Host "Following (Ctrl+C stops monitor only). Abort run: .\abort.ps1"
while (-not $script:seenResult) {
  Sync-Csv $CsvPath
  $newest = Get-NewestCsv
  if ($newest -and $newest.FullName -ne $CsvPath) {
    Write-Host "NEW-CSV $($newest.Name) - switching"
    $CsvPath = $newest.FullName
    $script:header = $null
    $script:csvRowsConsumed = 0
    $script:prevStepKey = $null
    $script:minDist = [double]::PositiveInfinity
    $script:distRisingSince = -1
    Sync-Csv $CsvPath
  }
  Tail-Log
  if (-not (Get-Process KSP_x64 -EA SilentlyContinue)) {
    Write-Host "KSP not running."
    if (-not $script:seenResult) { Write-Fail "ksp-exited" "no RESULT yet" }
    break
  }
  Start-Sleep -Milliseconds $PollMs
}
Write-Host "Monitor end."

