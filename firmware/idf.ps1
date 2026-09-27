# Run idf.py for this project with ESP-IDF v5.5.5's environment.
#   ./idf.ps1 build
#   ./idf.ps1 -p COM4 flash
# v5.5.5's export script needs a Python 3.11 first on PATH; uv's is used.
param([Parameter(ValueFromRemainingArguments = $true)] $IdfArgs)
$py = (Get-ChildItem "$env:APPDATA\uv\python" -Directory |
       Where-Object { $_.Name -like 'cpython-3.11*' } | Select-Object -First 1).FullName
if (-not $py) { throw "No uv Python 3.11 under $env:APPDATA\uv\python" }

# PATH is restored on the way out: a second run in the same shell would
# otherwise put uv's Python ahead of ESP-IDF's own environment, and idf.py
# fails with "No module named 'click'" (measured).
$savedPath = $env:PATH
$env:PATH = "$py;$env:PATH"
$code = 1
Push-Location $PSScriptRoot
try {
    . "$env:USERPROFILE\esp\v5.5.5\export.ps1" *> $null
    idf.py @IdfArgs
    $code = $LASTEXITCODE
} finally {
    Pop-Location
    $env:PATH = $savedPath
}
exit $code
