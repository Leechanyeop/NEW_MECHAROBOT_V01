@echo off
:: [중요] 아두이노 시리얼 모니터에 뜨는 IP 주소를 아래에 입력하세요
set IP=192.168.0.16
set CMD=%~1

if "%CMD%"=="" set CMD=x

echo Sending [%CMD%] to Robot at %IP%...

:: PowerShell 명령을 더 안정적으로 호출
powershell -NoProfile -Command "& { $c=New-Object System.Net.Sockets.TcpClient('%IP%', 8888); $w=New-Object System.IO.StreamWriter($c.GetStream()); $w.Write('%CMD%'); $w.Flush(); $c.Close() }"

if ERRORLEVEL 1 (
    echo [Error] Could not connect to %IP%. Check IP and WiFi connection.
) else (
    echo [Success] Command sent.
)

