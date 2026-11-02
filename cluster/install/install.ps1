# Installs a crane cluster worker for the current Windows user as a scheduled task
# that runs hidden (pythonw) at below-normal priority and restarts on failure, and provisions what
# its backends need (install\provision.yaml): uv and a Python venv with -Packages, portable MinGit,
# a static ffmpeg, the .NET SDK, a Vulkan check, a Unity player and UnityEngine.CoreModule.dll when
# given, and the repos the backends check out, all in the user profile (no admin, no system PATH change).
#   powershell -ExecutionPolicy Bypass -File install.ps1 -Coordinator URL -Invite CODE [-Name NAME]
#              [-Source DIR|GIT_URL] [-Python EXE] [-Packages numpy,pyyaml,...] [-Background] [-NoTask]
#              [-Tools DIR] [-Player ZIP|DIR|URL] [-UnityManaged DIR|DLL|URL] [-Minimal]
# Python >= 3.9: -Python, else a venv made by uv (downloaded to ~\.local\bin when missing), else python/py
# on PATH (the Microsoft Store stubs don't count). -Minimal skips the provisioning (tools, repos).
# Docker Desktop is needed only for container backends. Re-running updates the code and upgrades the
# tools and packages; it restarts a running worker only while it is idle.
# -Background: the task runs whether or not anyone is logged in (S4U, Windows session 0), so nothing it
# starts can show a window on a desktop; a second hidden task in the user's desktop session
# (crane-idleprobe, at logon) reports input idle time for pause_on_activity.
param(
  [Parameter(Mandatory = $true)][string]$Coordinator,
  [string]$Invite = "",
  [string]$Name = "",
  [string]$Source = "",
  [string]$Python = "",
  [string[]]$Packages = @("numpy", "pyyaml", "py_trees", "mcap", "mcap-ros2-support"),
  [switch]$Background,
  [switch]$NoTask,
  [string]$Tools = (Join-Path $env:USERPROFILE "Tools"),
  [string]$Player = "",
  [string]$UnityManaged = "",
  [switch]$Minimal
)
$ErrorActionPreference = "Stop"
$Packages = @($Packages | ForEach-Object { $_ -split "," } | Where-Object { $_ })  # -File passes "a,b" as one string

function Test-Python([string]$exe) {
  if (-not $exe -or $exe -like "*\WindowsApps\*") { return $false }
  & $exe -c "import sys; sys.exit(sys.version_info < (3, 9))" 2>$null
  return $LASTEXITCODE -eq 0
}

$prefix = Join-Path $env:LOCALAPPDATA "crane-cluster"
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$ProgressPreference = "SilentlyContinue"  # Invoke-WebRequest is many times slower with its progress bar
$uv = (Get-Command uv -ErrorAction SilentlyContinue).Source
$uvHome = Join-Path $env:USERPROFILE ".local\bin"
if (-not $uv -and (Test-Path "$uvHome\uv.exe")) { $uv = "$uvHome\uv.exe" }
if (-not $uv -and -not $Python -and -not $Minimal) {
  # uv's standalone build from its latest GitHub release, user-level
  $zip = Join-Path ([IO.Path]::GetTempPath()) "uv-$([guid]::NewGuid()).zip"
  Invoke-WebRequest -UseBasicParsing "https://github.com/astral-sh/uv/releases/latest/download/uv-x86_64-pc-windows-msvc.zip" -OutFile $zip
  New-Item -ItemType Directory -Force -Path $uvHome | Out-Null
  Expand-Archive -Force $zip $uvHome
  Remove-Item $zip
  $uv = "$uvHome\uv.exe"
  Write-Host "installed uv at $uv"
}
$py = $Python
if (-not $py -and $uv) {
  $venv = Join-Path $prefix "venv"
  if (-not (Test-Path "$venv\Scripts\python.exe")) { & $uv venv -q --python 3.12 $venv; if ($LASTEXITCODE -ne 0) { throw "uv venv failed" } }
  $py = "$venv\Scripts\python.exe"
  if ($Packages) { & $uv pip install -q -U --python $py @Packages; if ($LASTEXITCODE -ne 0) { throw "uv pip install failed" } }
}
if (-not $py) { $py = (Get-Command python -ErrorAction SilentlyContinue).Source }
if (-not (Test-Python $py)) { $py = (Get-Command py -ErrorAction SilentlyContinue).Source }
if (-not (Test-Python $py)) { throw "Python 3.9+ is required (-Python EXE, uv, or python.org)" }
if ($Packages -and -not $uv) { & $py -m pip install -q @Packages; if ($LASTEXITCODE -ne 0) { throw "pip install failed" } }
$pyw = Join-Path (Split-Path $py) "pythonw.exe"
if (-not (Test-Path $pyw)) { $pyw = $py }

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
  if (-not (Get-Command git -ErrorAction SilentlyContinue)) { throw "git is required to install from $Source" }
  $env:GIT_TERMINAL_PROMPT = "0"
  git clone -q --depth 1 $Source $tmp
  if ($LASTEXITCODE -ne 0) { throw "git clone $Source failed" }
  $pkg = Join-Path $tmp "cluster\crane_cluster"
}
$dest = Join-Path $lib "crane_cluster"
if (Test-Path $dest) { Remove-Item -Recurse -Force $dest }
Copy-Item -Recurse $pkg $dest
$plan = Join-Path (Split-Path $pkg) "install\provision.yaml"
if (Test-Path $plan) { Copy-Item -Force $plan (Join-Path $prefix "provision.yaml") }
if (Test-Path $tmp) { Remove-Item -Recurse -Force $tmp }
$env:PYTHONPATH = $lib
$bin = Join-Path $prefix "bin"
New-Item -ItemType Directory -Force -Path $bin | Out-Null
foreach ($tool in "worker", "hyp") {
  Set-Content -Path (Join-Path $bin "crane-$tool.cmd") -Encoding ASCII -Value @(
    "@set PYTHONPATH=$lib", "@`"$py`" -m crane_cluster.$tool %*")
}

# 2. config (kept if present)
if (-not (Test-Path $config)) {
  $lines = @("coordinator: $Coordinator")
  if ($Name) { $lines += "name: $Name" }
  $lines += @("policy:", "  nice: 10", "  pause_on_activity: true", "backends:", "  dummy: {type: dummy}")
  Set-Content -Path $config -Value $lines -Encoding UTF8
  Write-Host "wrote $config (edit backends, caps and policy there)"
}

# 3. provisioning: tools, repos, and the backends they make possible (added to the config when missing)
if (-not $Minimal) {
  $provArgs = @("-m", "crane_cluster.provision", "--plan", (Join-Path $prefix "provision.yaml"), "--tools", $Tools, "--config", $config)
  if ($Player) { $provArgs += @("--player", $Player) }
  if ($UnityManaged) { $provArgs += @("--unity-managed", $UnityManaged) }
  & $py @provArgs
  if ($LASTEXITCODE -ne 0) { throw "provisioning failed" }
}

# 4. join
if ($Invite) {
  $joinArgs = @("-m", "crane_cluster.worker", "--config", $config, "join", "--coordinator", $Coordinator, "--invite", $Invite)
  if ($Name) { $joinArgs += @("--name", $Name) }
  & $py @joinArgs
  if ($LASTEXITCODE -ne 0) { throw "join failed" }
}

# 5. scheduled task
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
$settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable `
  -ExecutionTimeLimit ([TimeSpan]::Zero) -RestartCount 999 -RestartInterval (New-TimeSpan -Minutes 1) -Priority 7 -Hidden `
  -MultipleInstances IgnoreNew
$user = [Security.Principal.WindowsIdentity]::GetCurrent().Name  # COLLIN\me (USERDOMAIN is WORKGROUP over ssh)
if ($Background) {
  # S4U: runs without a password whether or not the user is logged in, in session 0 (no visible desktop).
  # The repeating trigger (re)starts it after a reboot without needing an administrator's startup trigger.
  $principal = New-ScheduledTaskPrincipal -UserId $user -LogonType S4U -RunLevel Limited
  $triggers = @((New-ScheduledTaskTrigger -AtLogOn -User $user),
    (New-ScheduledTaskTrigger -Once -At (Get-Date).Date -RepetitionInterval (New-TimeSpan -Minutes 5)))
  Register-ScheduledTask -TaskName "crane-worker" -Action $action -Trigger $triggers -Settings $settings `
    -Principal $principal -Description "crane cluster worker (background, session 0)" -Force | Out-Null
  $probe = Join-Path $prefix "crane-idleprobe.pyw"
  Set-Content -Path $probe -Encoding UTF8 -Value @(
    "import sys",
    "sys.path.insert(0, '$libLit')",
    "from crane_cluster.idleprobe import main",
    "main([])"
  )
  $probeAction = New-ScheduledTaskAction -Execute $pyw -Argument "`"$probe`"" -WorkingDirectory $prefix
  $probePrincipal = New-ScheduledTaskPrincipal -UserId $user -LogonType Interactive -RunLevel Limited
  Register-ScheduledTask -TaskName "crane-idleprobe" -Action $probeAction -Trigger (New-ScheduledTaskTrigger -AtLogOn -User $user) `
    -Settings $settings -Principal $probePrincipal -Description "reports desktop idle time to the crane worker" -Force | Out-Null
  Start-ScheduledTask -TaskName "crane-idleprobe"
} else {
  $trigger = New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME
  Register-ScheduledTask -TaskName "crane-worker" -Action $action -Trigger $trigger -Settings $settings `
    -Description "crane cluster worker" -Force | Out-Null
}
$state = $null
try { $state = (Get-Content -Raw (Join-Path $prefix "state.json") | ConvertFrom-Json).state } catch { }
if ((Get-ScheduledTask -TaskName "crane-worker").State -eq "Running") {
  if ($state -in "idle", "paused", "stopped") {  # no job held: restart now on the new code and tools
    Stop-ScheduledTask -TaskName "crane-worker"; Start-ScheduledTask -TaskName "crane-worker"
    Write-Host "restarted the idle worker on the new code"
  } else { Write-Host "the worker is busy ($state): restart it after its job: Stop-ScheduledTask crane-worker; Start-ScheduledTask crane-worker" }
} else { Start-ScheduledTask -TaskName "crane-worker"; Write-Host "started scheduled task 'crane-worker'" }
Write-Host "toggle and status: $bin\crane-worker.cmd on|off|status"
