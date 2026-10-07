$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$files = git -C $root ls-files --cached --others --exclude-standard |
    ForEach-Object { Join-Path $root $_ } |
    Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } |
    ForEach-Object { Get-Item -LiteralPath $_ }
$projectFiles = $files | Where-Object {
    $_.FullName -notmatch '[\\/]third_party[\\/]'
}
$textExtensions = '.cpp', '.hpp', '.md', '.ps1', '.bat', '.txt', '.json', '.yaml', '.yml'
$textFiles = $projectFiles | Where-Object { $_.Extension -in $textExtensions }

$trailing = $textFiles | Select-String -Pattern '[ \t]+$'
if ($trailing) {
    $trailing | Select-Object -First 20 | Format-Table -AutoSize
    throw 'trailing whitespace found'
}

$markerPattern = ('TO' + 'DO|' + 'FIX' + 'ME|' + ('<' * 7) + '|' + ('>' * 7))
$markers = $textFiles | Select-String -Pattern $markerPattern
if ($markers) {
    $markers | Select-Object -First 20 | Format-Table -AutoSize
    throw 'unfinished or conflict marker found'
}

$codeFiles = $projectFiles | Where-Object { $_.Extension -in '.cpp', '.hpp' }
$unsafeProcess = $codeFiles | Select-String -Pattern '\b(system|_popen|popen)\s*\('
if ($unsafeProcess) {
    $unsafeProcess | Select-Object -First 20 | Format-Table -AutoSize
    throw 'shell-based process launch found'
}

$buildText = Get-Content (Join-Path $root 'build.bat') -Raw
$listed = [regex]::Matches($buildText, 'src\\[^\s^]+\.cpp') |
    ForEach-Object { $_.Value } | Sort-Object -Unique
$actual = Get-ChildItem (Join-Path $root 'src') -Recurse -Filter *.cpp |
    ForEach-Object { $_.FullName.Substring($root.Length + 1) -replace '/', '\' } |
    Sort-Object -Unique
$sourceDiff = Compare-Object $listed $actual
if ($sourceDiff) {
    $sourceDiff | Format-Table -AutoSize
    throw 'build.bat source list differs from src tree'
}

if ($buildText -notmatch '/std:c\+\+17' -or $buildText -notmatch '/utf-8') {
    throw 'build.bat must enable C++17 and UTF-8'
}

$clangFormat = Get-Command clang-format -ErrorAction SilentlyContinue
if ($clangFormat) {
    & $clangFormat.Source --dry-run --Werror @($codeFiles.FullName)
    if ($LASTEXITCODE -ne 0) {
        throw 'clang-format check failed'
    }
}

Write-Output 'dtsx-decode lint-clean'
