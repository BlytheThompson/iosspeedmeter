@echo off
REM Run a built executable with the Swift runtime DLLs on PATH.
REM Without this the exe exits with 0xC0000135 (DLL not found).
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set "SWIFT_ROOT=%LOCALAPPDATA%\Programs\Swift"
set "PATH=%SWIFT_ROOT%\Runtimes\6.3.3\usr\bin;%SWIFT_ROOT%\Toolchains\6.3.3+Asserts\usr\bin;%PATH%"
%*
