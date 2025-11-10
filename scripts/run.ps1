param(
  [Parameter(Mandatory=$true)][string]$p
)

# ensure bin/ exists
if (!(Test-Path -Path "bin")) {
  New-Item -ItemType Directory -Path "bin" | Out-Null
}

# compile
g++ "$p.cpp" -o "bin\$p.exe"
if ($LASTEXITCODE -ne 0) {
  Write-Error "Compilation failed (g++ exited with code $LASTEXITCODE)"
  exit $LASTEXITCODE
}

# run
& "bin\$p.exe"