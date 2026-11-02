# Installs a crane cluster worker for the current Windows user as a scheduled task
# that starts at logon, runs hidden (pythonw) at below-normal priority, and restarts on failure.
#   powershell -ExecutionPolicy Bypass -File install.ps1 -Coordinator URL -Invite CODE [-Name NAME]
#              [-Source DIR|GIT_URL] [-NoTask]
# Needs Python >= 3.9 (python.org or `winget install Python.Python.3.12`) and git.
# Docker Desktop is needed only for container backends. Re-running updates the code.
param(
  [Parameter(Mandatory = $true)][string]$Coordinator,
  [string]$Invite = "",
  [string]$Name = "",
  [string]$Source = "",
  [switch]$NoTask
)
$ErrorActionPreference = "Stop"

$py = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $py) { $py = (Get-Command py -ErrorAction SilentlyContinue).Source }
if (-not $py) { throw "Python 3.9+ is required" }
& $py -c "import sys; sys.exit(sys.version_info < (3, 9))"
if ($LASTEXITCODE -ne 0) { throw "Python 3.9+ is required" }
if (-not (Get-Command git -ErrorAction SilentlyContinue)) { throw "git is required" }
$pyw = Join-Path (Split-Path $py) "pythonw.exe"
if (-not (Test-Path $pyw)) { $pyw = $py }

$prefix = Join-Path $env:LOCALAPPDATA "crane-cluster"
$confDir = Join-Path $env:APPDATA "crane-cluster"
$config = Join-Path $confDir "worker.yaml"
$lib = Join-Path $prefix "lib"
New-Item -ItemType Directory -Force -Path $lib, $confDir | Out-Null

# 1. code: a copy of the crane_cluster package
if (-not $Source) { $Source = Split-Path $PSScriptRoot }
$tmp = Join-Path ([IO.Path]::GetTempPath()) ("crane-" + [guid]::NewGuid())
if (Test-Path (Join-Path $Source "crane_cluster")) { $pkg = Join-Path $Source "crane_cluster" }
elseif (Test-Path (Join-Path $Source "cluster\crane_cluster")) { $pkg = Join-Path $Source "cluster\crane_cluster" }
else {
  $env:GIT_TERMINAL_PROMPT = "0"
  git clone -q --depth 1 $Source $tmp
  if ($LASTEXITCODE -ne 0) { throw "git clone $Source failed" }
  $pkg = Join-Path $tmp "cluster\crane_cluster"
}
$dest = Join-Path $lib "crane_cluster"
if (Test-Path $dest) { Remove-Item -Recurse -Force $dest }
Copy-Item -Recurse $pkg $dest
if (Test-Path $tmp) { Remove-Item -Recurse -Force $tmp }
$env:PYTHONPATH = $lib

# 2. config (kept if present)
if (-not (Test-Path $config)) {
  $lines = @("coordinator: $Coordinator")
  if ($Name) { $lines += "name: $Name" }
  $lines += @("policy:", "  nice: 10", "  pause_on_activity: true", "backends:", "  dummy: {type: dummy}")
  Set-Content -Path $config -Value $lines -Encoding UTF8
  Write-Host "wrote $config (edit backends, caps and policy there)"
}

# 3. join
if ($Invite) {
  $joinArgs = @("-m", "crane_cluster.worker", "--config", $config, "join", "--coordinator", $Coordinator, "--invite", $Invite)
  if ($Name) { $joinArgs += @("--name", $Name) }
  & $py @joinArgs
  if ($LASTEXITCODE -ne 0) { throw "join failed" }
}

# 4. scheduled task
if ($NoTask) { Write-Host "run with: `$env:PYTHONPATH='$lib'; python -m crane_cluster.worker --config '$config' run"; exit 0 }
# A .pyw launcher run by pythonw.exe: no console window, and no environment variables needed.
$launcher = Join-Path $prefix "crane-worker.pyw"
$libLit = $lib.Replace("\", "\\")
$confLit = $config.Replace("\", "\\")
Set-Content -Path $launcher -Encoding UTF8 -Value @(
  "import logging, sys",
  "sys.path.insert(0, '$libLit')",
  "logging.basicConfig(filename=r'$prefix\worker.log', level=logging.INFO)",
  "from crane_cluster.worker import main",
  "main(['--config', '$confLit', 'run'])"
)
$action = New-ScheduledTaskAction -Execute $pyw -Argument "`"$launcher`"" -WorkingDirectory $prefix
$trigger = New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME
$settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable `
  -ExecutionTimeLimit ([TimeSpan]::Zero) -RestartCount 999 -RestartInterval (New-TimeSpan -Minutes 1) -Priority 7 -Hidden
Register-ScheduledTask -TaskName "crane-worker" -Action $action -Trigger $trigger -Settings $settings `
  -Description "crane cluster worker" -Force | Out-Null
Start-ScheduledTask -TaskName "crane-worker"
Write-Host "registered and started scheduled task 'crane-worker' (Task Scheduler > crane-worker)"
