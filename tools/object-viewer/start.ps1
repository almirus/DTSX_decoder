$ErrorActionPreference = "Stop"

$viewerPath = Join-Path $PSScriptRoot "index.html"
if (-not (Test-Path -LiteralPath $viewerPath -PathType Leaf)) {
    throw "Object Viewer is incomplete: $viewerPath was not found"
}

Start-Process -FilePath $viewerPath
