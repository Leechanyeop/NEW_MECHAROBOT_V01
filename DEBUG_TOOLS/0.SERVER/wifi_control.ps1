# WiFi Robot Controller
# 사용법: 이 파일을 실행하고 로봇의 IP 주소를 입력하세요.
# 'W', 'A', 'S', 'D' 키를 누르면 로봇이 즉시 움직입니다. 'X'는 정지.

param(
    [string]$ip = ""
)

if ($ip -eq "") {
    $ip = Read-Host "Enter Robot IP Address (e.g., 192.168.0.15)"
}

$port = 8888

try {
    Write-Host "Connecting to $ip : $port ..."
    $client = New-Object System.Net.Sockets.TcpClient($ip, $port)
    $stream = $client.GetStream()
    $writer = New-Object System.IO.StreamWriter($stream)
    $writer.AutoFlush = $true

    Write-Host "Connected! Controls:" -ForegroundColor Green
    Write-Host " [W] Forward"
    Write-Host " [S] Backward"
    Write-Host " [A] Left"
    Write-Host " [D] Right"
    Write-Host " [X] Stop (Spacebar also stops)"
    Write-Host " [L] Line Trace"
    Write-Host " [P] PID Trace"
    Write-Host " Press 'Q' to Quit."

    $running = $true
    while ($running) {
        if ([Console]::KeyAvailable) {
            $keyInfo = [Console]::ReadKey($true) # true = intercept (don't show char)
            $char = $keyInfo.KeyChar.ToString()
            $key = $keyInfo.Key.ToString()

            $cmd = ""
            
            switch ($keyInfo.Key) {
                "W" { $cmd = "w" }
                "S" { $cmd = "s" }
                "A" { $cmd = "a" }
                "D" { $cmd = "d" }
                "X" { $cmd = "x" }
                "Spacebar" { $cmd = "x" }
                "L" { $cmd = "l" }
                "P" { $cmd = "p" }
                "Q" { $running = $false; Write-Host "Quitting..." }
            }

            if ($cmd -ne "") {
                try {
                    $bytes = [System.Text.Encoding]::ASCII.GetBytes($cmd)
                    $stream.Write($bytes, 0, $bytes.Length)
                    Write-Host "Sent: $cmd" -NoNewline -ForegroundColor Yellow
                    Write-Host "`r" -NoNewline
                } catch {
                    Write-Host "Connection Lost!" -ForegroundColor Red
                    $running = $false
                }
            }
        }
        Start-Sleep -Milliseconds 50
    }

    $client.Close()
} catch {
    Write-Host "Failed to connect: $_" -ForegroundColor Red
}
