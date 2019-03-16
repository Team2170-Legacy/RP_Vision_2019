set /P ip=10.21.70.50
set /P copyfilename=CalibrationRecovery
make
pause
scp CalibrationRecovery pi@10.21.70.50:~/RP_Vision_2019/Calibration/CalibrationRecovery
scp CalibrationRecovery pi@110.21.70.50:~/RP_Vision_2019/Calibration/CalibrationRecovery.o
scp GripPipeline.o pi@110.21.70.50:~/RP_Vision_2019/Calibration/GripPipeline.o