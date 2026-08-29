@echo off
REM Run the Swift toolchain inside the MSVC developer environment.
REM Swift-for-Windows needs link.exe + the Windows SDK (via vcvars64) and SDKROOT
REM pointing at the Swift Windows SDK, otherwise the stdlib will not load.
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set "SWIFT_ROOT=%LOCALAPPDATA%\Programs\Swift"
set "SDKROOT=%SWIFT_ROOT%\Platforms\6.3.3\Windows.platform\Developer\SDKs\Windows.sdk"
set "PATH=%SWIFT_ROOT%\Toolchains\6.3.3+Asserts\usr\bin;%SWIFT_ROOT%\Runtimes\6.3.3\usr\bin;%PATH%"
swift %*
