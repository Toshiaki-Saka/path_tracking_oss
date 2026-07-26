# Unified build + simulation + animation launcher.
#
# Usage:
#   .\run_simulation.ps1 [-Target compare|cutin|all] [-BuildType Release|Debug]
#                        [-Reconfigure] [-NoAnimate] [-Save FILE]
#
# -Target     compare : path_tracking_compare only
#             cutin   : cutin_demo only
#             all     : both (default)
# -Reconfigure        : force a fresh CMake configure
# -NoAnimate          : skip animation generation
# -Save FILE          : save the animation to a file (MP4/GIF)

param(
    [ValidateSet("compare", "cutin", "all")]
    [string]$Target      = "all",
    [string]$BuildType   = "Release",
    [switch]$Reconfigure,
    [switch]$NoAnimate,
    [string]$Save        = ""
)

$ErrorActionPreference = "Stop"
$Root     = $PSScriptRoot
$BuildDir = Join-Path $Root "build"

# -- Helper: locate the Python executable ------------------------------------
function Get-Python {
    $py = (Get-Command python  -ErrorAction SilentlyContinue)?.Source
    if (-not $py) { $py = (Get-Command python3 -ErrorAction SilentlyContinue)?.Source }
    return $py
}

# -- Helper: resolve a built executable path (multi/single-config aware) ------
function Resolve-Exe([string]$Name) {
    $p = Join-Path $BuildDir "$BuildType\$Name.exe"
    if (-not (Test-Path $p)) { $p = Join-Path $BuildDir "$Name.exe" }
    if (-not (Test-Path $p)) { throw "$Name.exe was not found in the build directory." }
    return $p
}

# ====================================================================
# 1. CMake configure
# ====================================================================
if ($Reconfigure -or -not (Test-Path (Join-Path $BuildDir "CMakeCache.txt"))) {
    Write-Host "`n==> Configuring CMake ($BuildType) ..." -ForegroundColor Cyan
    cmake -S $Root -B $BuildDir -DCMAKE_BUILD_TYPE=$BuildType
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# ====================================================================
# 2. Build
# ====================================================================
$buildTargets = switch ($Target) {
    "compare" { @("path_tracking_compare") }
    "cutin"   { @("cutin_demo") }
    "all"     { @("path_tracking_compare", "cutin_demo") }
}

foreach ($t in $buildTargets) {
    Write-Host "`n==> Building: $t ..." -ForegroundColor Cyan
    cmake --build $BuildDir --config $BuildType --target $t --parallel
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# ====================================================================
# 3. Simulation & animation
# ====================================================================
$RouteCSV = Join-Path $Root "data\reference_route.csv"
$Python   = Get-Python

# -- path_tracking_compare ---------------------------------------------------
if ($Target -in @("compare", "all")) {
    $Exe       = Resolve-Exe "path_tracking_compare"
    $ResultDir = Split-Path $Exe

    Write-Host "`n==> Running simulation: path_tracking_compare ..." -ForegroundColor Cyan
    Push-Location $ResultDir
    & $Exe
    $ec = $LASTEXITCODE
    Pop-Location
    if ($ec -ne 0) { exit $ec }

    if (-not $NoAnimate) {
        if ($Python) {
            $AnimScript = Join-Path $Root "tools\animate_results.py"
            $AnimArgs   = @($AnimScript, $ResultDir, $RouteCSV)
            if ($Save -ne "") {
                $AnimArgs += @("--save", $Save)
                Write-Host "==> Saving animation to: $Save" -ForegroundColor Cyan
            } else {
                Write-Host "==> Launching animation (close the window to exit) ..." -ForegroundColor Cyan
            }
            & $Python @AnimArgs
            if ($LASTEXITCODE -ne 0) {
                Write-Warning "animate_results.py failed (exit $LASTEXITCODE)."
            }
        } else {
            Write-Warning "Python not found - skipping animation."
        }
    }
}

# -- cutin_demo --------------------------------------------------------------
if ($Target -in @("cutin", "all")) {
    $Exe       = Resolve-Exe "cutin_demo"
    $ResultDir = Split-Path $Exe

    Write-Host "`n==> Running simulation: cutin_demo ..." -ForegroundColor Cyan
    Push-Location $ResultDir
    & $Exe
    $ec = $LASTEXITCODE
    Pop-Location
    if ($ec -ne 0) { exit $ec }

    if (-not $NoAnimate) {
        if ($Python) {
            # 4-panel static plot
            $PlotScript = Join-Path $Root "tools\plot_cutin_demo.py"
            Write-Host "`n==> Generating 4-panel plot ..." -ForegroundColor Cyan
            & $Python $PlotScript $ResultDir $RouteCSV
            if ($LASTEXITCODE -eq 0) {
                Write-Host "  Plot: $(Join-Path $ResultDir 'fig_cutin_demo.png')" -ForegroundColor Green
            }

            # Animation (MP4, with GIF fallback)
            $AnimScript = Join-Path $Root "tools\animate_cutin.py"
            $AnimOut    = if ($Save -ne "") { $Save } else { Join-Path $ResultDir "cutin_animation.mp4" }
            Write-Host "`n==> Generating animation (MP4) ..." -ForegroundColor Cyan
            & $Python $AnimScript $ResultDir $RouteCSV --save $AnimOut --fps 20 --step 5
            if ($LASTEXITCODE -ne 0) {
                $AnimOut = [System.IO.Path]::ChangeExtension($AnimOut, ".gif")
                Write-Host "  MP4 failed - retrying with GIF ..." -ForegroundColor Yellow
                & $Python $AnimScript $ResultDir $RouteCSV --save $AnimOut --fps 15 --step 10
            }
            if ($LASTEXITCODE -eq 0) {
                Write-Host "  Animation: $AnimOut" -ForegroundColor Green
            } else {
                Write-Warning "Animation generation failed (install FFmpeg or Pillow)."
                Write-Host "  Interactive preview: python tools\animate_cutin.py $ResultDir $RouteCSV" -ForegroundColor Yellow
            }
        } else {
            Write-Warning "Python not found - skipping plot/animation."
        }
    }
}

Write-Host "`n==> Done." -ForegroundColor Green
exit 0
