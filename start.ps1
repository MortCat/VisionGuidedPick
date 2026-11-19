$ErrorActionPreference = 'Stop'
& python (Join-Path $PSScriptRoot 'deployment/runtime/application.py') @args
exit $LASTEXITCODE
