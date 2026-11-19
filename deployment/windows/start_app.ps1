#requires -Version 7.2

[CmdletBinding()]
param(
    [switch] $DryRun,
    [string] $WslProjectRoot,
    [switch] $SkipIsaac,
    [switch] $NoApp,
    # Used by Stop: close only this checkout's dedicated App window.
    [switch] $CloseApp,
    [string] $AppProfile = (Join-Path $env:LOCALAPPDATA 'VisionGuidedPick/BrowserProfile')
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Get-DedicatedBrowserProcess {
    param([string] $Browser, [string] $Profile)
    $name = [IO.Path]::GetFileName($Browser)
    if ($name -notin @('chrome.exe', 'msedge.exe')) { throw 'Unsupported App browser.' }
    $wanted = [IO.Path]::GetFullPath($Profile).TrimEnd('\', '/')
    $matching = @(Get-CimInstance -ClassName Win32_Process -Filter "Name = '$name'" -ErrorAction Stop |
        Where-Object {
            $line = [string] $_.CommandLine
            if ($line -match '(?:^|\s)--type(?:=|\s)') { return $false }
            $match = [regex]::Match($line, '(?:^|\s)--user-data-dir=(?:"([^"\r\n]+)"|([^\s"]+))')
            if (-not $match.Success) { return $false }
            $actual = if ($match.Groups[1].Success) { $match.Groups[1].Value } else { $match.Groups[2].Value }
            try { $actual = [IO.Path]::GetFullPath($actual).TrimEnd('\', '/') } catch { return $false }
            return [string]::Equals($actual, $wanted, [StringComparison]::OrdinalIgnoreCase)
        })
    if ($matching.Count -gt 1) { throw 'Multiple main browser processes own the dedicated App profile; services were left running.' }
    if ($matching.Count -eq 1) {
        return Get-Process -Id $matching[0].ProcessId -ErrorAction SilentlyContinue
    }
    return $null
}

function Wait-DedicatedBrowserProcess {
    param($Process, [string] $Browser, [string] $Profile)
    while ($null -ne $Process) {
        $Process.WaitForExit()
        $code = $Process.ExitCode
        # Chromium may hand this launch to an existing main process. Follow that
        # exact dedicated-profile owner rather than treating launcher exit as close.
        $next = Get-DedicatedBrowserProcess -Browser $Browser -Profile $Profile
        if ($null -eq $next) { return $code }
        Write-Host "Following existing App browser process $($next.Id)."
        $Process = $next
    }
}

function Close-DedicatedBrowserApp {
    param([string] $Profile, [int] $TimeoutSeconds = 10)
    foreach ($name in @('msedge.exe', 'chrome.exe')) {
        $app = Get-DedicatedBrowserProcess -Browser $name -Profile $Profile
        if ($null -eq $app) { continue }
        # Same as the window close button: the browser exits 0 and the waiting
        # App-closed Stop finds its Web instance already stopped.
        [void] $app.CloseMainWindow()
        if (-not $app.WaitForExit($TimeoutSeconds * 1000)) {
            Stop-Process -Id $app.Id -Force -ErrorAction SilentlyContinue
            if (-not $app.WaitForExit(5000)) { throw "App window $($app.Id) is still running." }
        }
        Write-Host 'App window closed.'
    }
}

if ($CloseApp) {
    Close-DedicatedBrowserApp -Profile $AppProfile
    exit 0
}

$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$isaacLauncher = Join-Path $projectRoot 'deployment/windows/start_isaac.ps1'
$locationsPath = Join-Path $projectRoot 'deployment/runtime/local/locations.json'
$pwshExe = Join-Path $PSHOME 'pwsh.exe'

$requiredPaths = @($locationsPath)
if (-not $SkipIsaac) { $requiredPaths += @($isaacLauncher, $pwshExe) }
foreach ($requiredPath in $requiredPaths) {
    if (-not (Test-Path -LiteralPath $requiredPath -PathType Leaf)) {
        throw "Required startup file was not found: $requiredPath"
    }
}

$locations = Get-Content -LiteralPath $locationsPath -Raw | ConvertFrom-Json
$distribution = [string] $locations.wsl_distribution
if ([string]::IsNullOrWhiteSpace($distribution)) {
    throw "wsl_distribution is missing from $locationsPath"
}

$wslExe = (Get-Command -Name 'wsl.exe' -ErrorAction Stop).Source
if ([string]::IsNullOrWhiteSpace($WslProjectRoot)) {
    $launcherPath = Join-Path $projectRoot 'deployment/runtime/local/launcher.json'
    if (-not (Test-Path -LiteralPath $launcherPath -PathType Leaf)) {
        throw 'Run python deployment/runtime/setup.py first: a separate WSL checkout must be configured.'
    }
    $launcher = Get-Content -LiteralPath $launcherPath -Raw | ConvertFrom-Json
    if ($launcher.schema_version -ne 1) { throw 'Unsupported launcher.json schema; run Setup.' }
    $WslProjectRoot = [string] $launcher.wsl_project_root
}
if (-not $WslProjectRoot.StartsWith('/') -or $WslProjectRoot -match '[\r\n]' -or
    $WslProjectRoot -match '(^|/)\.\.(/|$)' -or $WslProjectRoot -match '^/mnt/[a-z](/|$)') {
    throw 'Use an absolute separate checkout in the WSL Linux filesystem; run Setup.'
}
$wslArgs = @('-d', $distribution, '--cd', $WslProjectRoot, '--exec')

$isaacDryRunExit = 0
if ($SkipIsaac) {
    Write-Host 'Skipping Isaac Sim preflight and startup; use your separately launched simulator.'
} else {
    Write-Host 'Preflight: Windows Isaac Sim runtime (windows_wsl).'
    & $pwshExe -NoProfile -File $isaacLauncher -RuntimeProfile windows_wsl -DryRun
    $isaacDryRunExit = $LASTEXITCODE
}

Write-Host "Preflight: WSL application in $distribution."
& $wslExe @wslArgs bash ./start.sh --runtime-profile windows_wsl --dry-run --no-browser
$wslDryRunExit = $LASTEXITCODE

if ($isaacDryRunExit -ne 0 -or $wslDryRunExit -ne 0) {
    Write-Host "Preflight failed (Isaac=$isaacDryRunExit, WSL=$wslDryRunExit); nothing was launched." -ForegroundColor Red
    exit 1
}

if ($DryRun) {
    Write-Host 'Dry run passed; no simulator or ROS processes were launched.'
    exit 0
}

if (-not $SkipIsaac) {
    Write-Host 'Starting Isaac Sim with the project ROS Bridge and optional MCP extension.'
    & $pwshExe -NoProfile -File $isaacLauncher -RuntimeProfile windows_wsl
    $isaacExit = $LASTEXITCODE
    if ($isaacExit -ne 0) {
        Write-Host "Isaac Sim startup failed with exit code $isaacExit; WSL application was not launched." -ForegroundColor Red
        exit 1
    }
}

Write-Host 'Starting the WSL ROS 2 application.'
& $wslExe @wslArgs bash ./start.sh --runtime-profile windows_wsl --no-browser
$wslExit = $LASTEXITCODE
if ($wslExit -ne 0) {
    Write-Host "WSL application startup failed with exit code $wslExit. Any running Isaac Sim instance was left alone." -ForegroundColor Red
    exit 1
}

Write-Host 'Wait for the USD to load, then press Isaac Sim Play.'
Write-Host 'Start Observe and then use Auto Pick Start to run each arm.'
$recordJson = & $wslExe @wslArgs cat logs/teach_target/process.json
if ($LASTEXITCODE -ne 0) { throw 'Could not read the Web service identity.' }
$record = $recordJson | ConvertFrom-Json
$webInstance = "$($record.pid):$($record.start)"
$webUri = [uri] $record.url
$productUrl = "http://127.0.0.1:$($webUri.Port)/"
Write-Host "Product UI: $productUrl"
# WSL service readiness does not establish Windows localhost forwarding readiness.
$deadline = [DateTime]::UtcNow.AddSeconds(15)
$webReady = $false
while ([DateTime]::UtcNow -lt $deadline) {
    try {
        $response = Invoke-WebRequest -Uri $productUrl -TimeoutSec 2 -ErrorAction Stop
        if ($response.StatusCode -eq 200) { $webReady = $true; break }
    } catch { }
    Start-Sleep -Milliseconds 200
}
if (-not $webReady) {
    throw "WSL backend started but the Windows UI URL is unavailable: $productUrl. Check WSL localhost forwarding; services were left running."
}
if ($NoApp) {
    Write-Host 'Backend and Web are running without an App window. Open the Product UI in a browser.'
    exit 0
}
$browser = $null
foreach ($exe in @('msedge.exe', 'chrome.exe')) {
    foreach ($hive in @('HKCU:', 'HKLM:')) {
        $key = "$hive\SOFTWARE\Microsoft\Windows\CurrentVersion\App Paths\$exe"
        $registration = Get-Item -LiteralPath $key -ErrorAction SilentlyContinue
        if ($registration) {
            $candidate = $registration.GetValue('')
            if ($candidate -and (Test-Path -LiteralPath $candidate -PathType Leaf)) {
                $browser = $candidate
                break
            }
        }
    }
    if ($browser) { break }
}
if (-not $browser) {
    foreach ($folder in @($env:ProgramFiles, ${env:ProgramFiles(x86)}, $env:LOCALAPPDATA)) {
        if (-not $folder) { continue }
        foreach ($relative in @('Microsoft/Edge/Application/msedge.exe', 'Google/Chrome/Application/chrome.exe')) {
            $candidate = Join-Path $folder $relative
            if (Test-Path -LiteralPath $candidate -PathType Leaf) {
                $browser = $candidate
                break
            }
        }
        if ($browser) { break }
    }
}

Write-Host "Product UI: $productUrl"
if ($browser) {
    $appLock = [Threading.Mutex]::new($false, 'Local\VisionGuidedPickAppWindow')
    try {
        if (-not $appLock.WaitOne(0)) {
            Write-Host 'VisionGuidedPick app window is already running.'
            exit 0
        }
        try {
            $profile = $AppProfile
            $appArgs = @("--user-data-dir=`"$profile`"", '--no-first-run',
                '--disable-background-mode', '--window-size=1520,850', "--app=$productUrl")
            $existingApp = Get-DedicatedBrowserProcess -Browser $browser -Profile $profile
            $launched = Start-Process -FilePath $browser -ArgumentList $appArgs -PassThru -ErrorAction Stop
            $app = if ($null -ne $existingApp -and -not $existingApp.HasExited) { $existingApp } else { $launched }
            $appExit = Wait-DedicatedBrowserProcess -Process $app -Browser $browser -Profile $profile
            if ($appExit -eq 0) {
                & $wslExe @wslArgs bash ./stop.sh --app-closed --web-instance $webInstance
                if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
            }
        } finally {
            $appLock.ReleaseMutex()
        }
    } finally {
        $appLock.Dispose()
    }
}

if (-not $browser) {
    Write-Host "No supported App browser was found. Services remain running; open $productUrl in a browser."
}
