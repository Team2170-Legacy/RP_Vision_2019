set /P ip=10.21.70.22
set /P copyfilename=CalibrationRecovery
make
pause
scp CalibrationRecovery pi@10.21.70.22:~/Calibration/CalibrationRecovery
scp CalibrationRecovery pi@10.21.70.22:~/Calibration/CalibrationRecovery.o
