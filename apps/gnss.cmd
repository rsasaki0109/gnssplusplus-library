@echo off
setlocal
where py >nul 2>nul
if %ERRORLEVEL%==0 (
    py "%~dp0gnss.py" %*
) else (
    python "%~dp0gnss.py" %*
)
exit /b %ERRORLEVEL%
