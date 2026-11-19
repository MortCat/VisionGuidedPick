#requires -Version 7.2
[CmdletBinding()]
param(
    [string]$RuntimeProfile = 'windows_wsl',
    [string]$Locations = '',
    [switch]$DryRun
)
$ErrorActionPreference = 'Stop'
$runtimeDirectory = Join-Path (Split-Path $PSScriptRoot -Parent) 'runtime'
if (!$Locations) { $Locations = Join-Path $runtimeDirectory 'local/locations.json' }
if (!(Test-Path -LiteralPath $Locations)) {
    throw 'Create deployment/runtime/local/locations.json from locations.windows.example.json; paths are relative to the JSON location.'
}
# Bootstrap only the SDK path. The shared stdlib resolver validates the complete JSON,
# including duplicate keys, before any simulator process can be launched.
$bootstrap = Get-Content -Raw -LiteralPath $Locations | ConvertFrom-Json
$locationsDirectory = Split-Path ([IO.Path]::GetFullPath($Locations)) -Parent
$sdkRoot = [IO.Path]::GetFullPath([IO.Path]::Combine($locationsDirectory, [string]$bootstrap.isaac_root))
$python = Join-Path $sdkRoot 'kit/python/python.exe'
if (!(Test-Path -LiteralPath $python)) { throw "SDK Python missing: $python" }
$report = & $python (Join-Path $runtimeDirectory 'runtime_profile.py') isaac-plan --runtime-profile $RuntimeProfile --locations $Locations
$result = $LASTEXITCODE
if (!$report) { throw 'Isaac preflight did not return a launch plan' }
$plan = ($report -join "`n") | ConvertFrom-Json
$plan | ConvertTo-Json -Depth 12
if ($DryRun) { exit $result }
if ($result -ne 0) { throw 'Isaac preflight failed; correct the reported errors before launching.' }
$kitPath = [IO.Path]::GetFullPath($plan.argv[0])
if (Get-Process -Name kit -ErrorAction SilentlyContinue | Where-Object { !$_.Path -or $_.Path -eq $kitPath }) {
    throw 'Isaac/Kit is already running with this SDK or an unreadable process path. Close it normally before changing its ROS environment.'
}
$start = [Diagnostics.ProcessStartInfo]::new()
$start.FileName = $kitPath
$start.WorkingDirectory = $sdkRoot
$start.UseShellExecute = $false
$start.CreateNoWindow = $true
foreach ($argument in $plan.argv | Select-Object -Skip 1) { $start.ArgumentList.Add([string]$argument) }
$remove = @('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','FASTDDS_DEFAULT_PROFILES_FILE',
    'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_BUILTIN_TRANSPORTS','ROS_LOCALHOST_ONLY',
    'ROS_AUTOMATIC_DISCOVERY_RANGE','ROS_STATIC_PEERS','ROS_DISCOVERY_SERVER',
    'RMW_FASTRTPS_USE_QOS_FROM_XML','RMW_FASTRTPS_PUBLICATION_MODE','ROS_SECURITY_ENABLE',
    'AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','PYTHONPATH','PYTHONHOME','LD_LIBRARY_PATH')
foreach ($name in $remove) { [void]$start.Environment.Remove($name) }
foreach ($name in @($start.Environment.Keys)) {
    if ($name -like 'ZENOH_*') { [void]$start.Environment.Remove($name) }
}
$cleanPath = ($start.Environment['PATH'] -split ';' | Where-Object {
    $_ -and $_ -notmatch '(?i)([\\/]\.pixi[\\/]|[\\/]ros2?(?:[_-][^\\/;]+)?[\\/]|isaacsim\.ros2\.core)'
}) -join ';'
$start.Environment['PATH'] = [string]$plan.environment.PTC_RUNTIME_ROS_LIB + ';' + $cleanPath
foreach ($property in $plan.environment.PSObject.Properties) {
    if ($property.Name -ne 'PTC_RUNTIME_ROS_LIB') { $start.Environment[$property.Name] = [string]$property.Value }
}
$child = [Diagnostics.Process]::Start($start)
"Isaac started: PID $($child.Id); Bridge requested. Press Play; continuous project mode starts tasks after readiness."
