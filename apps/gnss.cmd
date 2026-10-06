@echo off
setlocal
set "gnss_dispatcher=%~dp0gnss.py"
if not exist "%gnss_dispatcher%" set "gnss_dispatcher=%~dp0gnss"
where py >nul 2>nul
if %ERRORLEVEL%==0 (
    py "%gnss_dispatcher%" %*
) else (
    python "%gnss_dispatcher%" %*
)
exit /b %ERRORLEVEL%
