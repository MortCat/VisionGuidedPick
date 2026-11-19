$ErrorActionPreference = 'Stop'
& python (Join-Path $PSScriptRoot 'deployment/runtime/application.py') --stop @args
exit $LASTEXITCODE
