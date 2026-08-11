param(
    [Parameter(Mandatory = $true)]
    [string]$ReferenceExe,

    [Parameter(Mandatory = $true)]
    [string]$CandidateExe,

    [Alias("Input")]
    [string[]]$Inputs,
    [string[]]$ReferenceExtraArgs = @(),
    [string[]]$CandidateExtraArgs = @(),
    [ValidateSet(0, 1, 2)]
    [int]$ReferenceThreads = 0,
    [ValidateSet(0, 1, 2)]
    [int]$CandidateThreads = 0,
    [string]$WorkRoot,
    [string]$Duration = "10s",
    [switch]$KeepOutput
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Resolve-ExistingFile([string]$Path, [string]$Description) {
    $resolved = Resolve-Path -LiteralPath $Path -ErrorAction Stop
    if (-not (Test-Path -LiteralPath $resolved.Path -PathType Leaf)) {
        throw "$Description is not a file: $Path"
    }
    return $resolved.Path
}

function Invoke-Decoder(
    [string]$Exe,
    [string[]]$Arguments,
    [string]$Label
) {
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    & $Exe @Arguments | Out-Host
    $exitCode = $LASTEXITCODE
    $timer.Stop()
    if ($exitCode -ne 0) {
        throw "$Label failed with exit code $exitCode"
    }
    return $timer.Elapsed
}

function Get-OutputManifest([string]$Root) {
    $manifest = [ordered]@{}
    $rootPrefix = [System.IO.Path]::GetFullPath($Root).TrimEnd(
        [System.IO.Path]::DirectorySeparatorChar,
        [System.IO.Path]::AltDirectorySeparatorChar) + [System.IO.Path]::DirectorySeparatorChar
    Get-ChildItem -LiteralPath $Root -Recurse -File |
        Sort-Object FullName |
        ForEach-Object {
            $relative = $_.FullName.Substring($rootPrefix.Length)
            $manifest[$relative] = [pscustomobject]@{
                Length = $_.Length
                Hash = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
            }
        }
    return $manifest
}

function Assert-ManifestsEqual(
    [System.Collections.IDictionary]$Reference,
    [System.Collections.IDictionary]$Candidate,
    [string]$Label
) {
    $referenceNames = @($Reference.Keys)
    $candidateNames = @($Candidate.Keys)
    $missing = @($referenceNames | Where-Object { -not $Candidate.Contains($_) })
    $extra = @($candidateNames | Where-Object { -not $Reference.Contains($_) })
    if ($missing.Count -ne 0 -or $extra.Count -ne 0) {
        throw "$Label output set differs; missing=[$($missing -join ', ')], extra=[$($extra -join ', ')]"
    }
    foreach ($name in $referenceNames) {
        $left = $Reference[$name]
        $right = $Candidate[$name]
        if ($left.Length -ne $right.Length -or $left.Hash -ne $right.Hash) {
            throw "$Label differs at $name (reference $($left.Length) bytes/$($left.Hash), candidate $($right.Length) bytes/$($right.Hash))"
        }
    }
}

$referenceExePath = Resolve-ExistingFile $ReferenceExe "reference decoder"
$candidateExePath = Resolve-ExistingFile $CandidateExe "candidate decoder"
$referenceArguments = @($ReferenceExtraArgs)
$candidateArguments = @($CandidateExtraArgs)
if ($ReferenceThreads -ne 0) {
    $referenceArguments += @("--threads", $ReferenceThreads.ToString())
}
if ($CandidateThreads -ne 0) {
    $candidateArguments += @("--threads", $CandidateThreads.ToString())
}
$repositoryRoot = Split-Path -Parent $PSScriptRoot

if (-not $Inputs -or $Inputs.Count -eq 0) {
    $defaults = @(
        (Join-Path $repositoryRoot "test_output\Zoolander.2.2016.metadata-regression.dtshd"),
        (Join-Path $repositoryRoot "test_files\DTS-X 7.1.4.mkv"),
        (Join-Path $repositoryRoot "test_files\trinnov\[Trinnov] Experience Trailer (DTS-X Pro) {4K SDR & DTS-X Pro}.mkv")
    )
    $Inputs = @($defaults | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf })
}
if (-not $Inputs -or $Inputs.Count -eq 0) {
    throw "no regression input was supplied and no default fixture exists"
}

if ([string]::IsNullOrWhiteSpace($WorkRoot)) {
    $WorkRoot = Join-Path $repositoryRoot (
        "test_output\decoder-regression-" + [Guid]::NewGuid().ToString("N"))
}
$workRootPath = [System.IO.Path]::GetFullPath($WorkRoot)
New-Item -ItemType Directory -Path $workRootPath | Out-Null

$totalReference = [TimeSpan]::Zero
$totalCandidate = [TimeSpan]::Zero
try {
    $caseIndex = 0
    foreach ($inputPath in $Inputs) {
        $source = Resolve-ExistingFile $inputPath "regression input"
        $caseName = "{0:D2}-{1}" -f $caseIndex, [System.IO.Path]::GetFileNameWithoutExtension($source)
        $caseName = $caseName -replace '[^A-Za-z0-9._-]', '_'
        $referenceRoot = Join-Path $workRootPath "$caseName-reference"
        $candidateRoot = Join-Path $workRootPath "$caseName-candidate"
        New-Item -ItemType Directory -Path $referenceRoot | Out-Null
        New-Item -ItemType Directory -Path $candidateRoot | Out-Null

        $referenceMain = Join-Path $referenceRoot "render.wav"
        $candidateMain = Join-Path $candidateRoot "render.wav"
        $commonMain = @(
            "-i", $source,
            "-o", $referenceMain,
            "--layout", "7.1.4",
            "--duration", $Duration,
            "--overwrite"
        )
        $referenceElapsed = Invoke-Decoder $referenceExePath (
            $commonMain + $referenceArguments) "$caseName reference render"
        $commonMain[3] = $candidateMain
        $candidateElapsed = Invoke-Decoder $candidateExePath (
            $commonMain + $candidateArguments) "$caseName candidate render"
        $totalReference += $referenceElapsed
        $totalCandidate += $candidateElapsed
        $referenceManifest = Get-OutputManifest $referenceRoot
        $candidateManifest = Get-OutputManifest $candidateRoot
        Assert-ManifestsEqual $referenceManifest $candidateManifest "$caseName render"

        $referenceObjects = Join-Path $referenceRoot "objects"
        $candidateObjects = Join-Path $candidateRoot "objects"
        $referenceObjectArgs = @(
            "-i", $source,
            "--objects-output-dir", $referenceObjects,
            "--objects-output-bed",
            "--duration", $Duration,
            "--overwrite"
        ) + $referenceArguments
        $candidateObjectArgs = @(
            "-i", $source,
            "--objects-output-dir", $candidateObjects,
            "--objects-output-bed",
            "--duration", $Duration,
            "--overwrite"
        ) + $candidateArguments
        $referenceElapsed = Invoke-Decoder $referenceExePath $referenceObjectArgs "$caseName reference objects"
        $candidateElapsed = Invoke-Decoder $candidateExePath $candidateObjectArgs "$caseName candidate objects"
        $totalReference += $referenceElapsed
        $totalCandidate += $candidateElapsed
        $referenceManifest = Get-OutputManifest $referenceObjects
        $candidateManifest = Get-OutputManifest $candidateObjects
        Assert-ManifestsEqual $referenceManifest $candidateManifest "$caseName objects"

        Write-Host "PASS $caseName"
        ++$caseIndex
    }

    $ratio = if ($totalCandidate.TotalMilliseconds -gt 0.0) {
        $totalReference.TotalMilliseconds / $totalCandidate.TotalMilliseconds
    } else {
        0.0
    }
    Write-Host ("PASS bit-exact regression; reference={0:N2}s candidate={1:N2}s speed={2:N2}x" -f
        $totalReference.TotalSeconds, $totalCandidate.TotalSeconds, $ratio)
} finally {
    if (-not $KeepOutput -and (Test-Path -LiteralPath $workRootPath)) {
        Remove-Item -LiteralPath $workRootPath -Recurse -Force
    } elseif ($KeepOutput) {
        Write-Host "Outputs: $workRootPath"
    }
}
