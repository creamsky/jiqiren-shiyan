@echo off
ssh -i "%USERPROFILE%\.ssh\robot_lab_ed25519" -o HostKeyAlias=10.226.235.72 -o IdentitiesOnly=yes -o ConnectTimeout=8 nvidia@10.107.60.72 "bash /home/nvidia/lab1/stop.sh"
echo.
pause
