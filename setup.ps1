$ErrorActionPreference = 'Stop'
& python (Join-Path $PSScriptRoot 'deployment/runtime/setup.py') @args
exit $LASTEXITCODE
