@echo off
setlocal

:: Follow Name of Web
set AdapterName=RoboterAnnaEthernet

:: KLI 
set IP_KLI=172.31.1.150
set Mask_KLI=255.255.0.0

:: RSI 
set IP_RSI=10.100.1.2
set Mask_RSI=255.255.255.0
:: ===========================================

:MENU
cls
echo ==============================================
echo       KUKA Network Switcher
echo ==============================================
echo.
echo Current Target Adapter: "%AdapterName%"
echo.
echo [1] Type "KLI" to switch to KLI (172.31...)
echo [2] Type "RSI" to switch to RSI (10.100...)
echo.
set /p UserInput=Please enter mode (KLI/RSI): 

:: Ignore Capital/ not
if /i "%UserInput%"=="KLI" goto SET_KLI
if /i "%UserInput%"=="RSI" goto SET_RSI
goto MENU

:SET_KLI
echo.
echo Switching to KLI mode (%IP_KLI%)...
netsh interface ip set address name="%AdapterName%" static %IP_KLI% %Mask_KLI%
if %errorlevel%==0 (
    echo [SUCCESS] IP changed to KLI settings.
) else (
    echo [FAILED] Please run as Administrator!
)
goto END

:SET_RSI
echo.
echo Switching to RSI mode (%IP_RSI%)...
netsh interface ip set address name="%AdapterName%" static %IP_RSI% %Mask_RSI%
if %errorlevel%==0 (
    echo [SUCCESS] IP changed to RSI settings.
) else (
    echo [FAILED] Please run as Administrator!
)
goto END

:END
echo.
echo IP configuration updated.
pause