set /P ip=169.254.17.217 
set /P copyfilename=CalibrationRecovery
make
pause
scp CalibrationRecovery pi@169.254.17.217:~/RP_Vision_2019/CalibrationRecovery/CalibrationRecovery
scp CalibrationRecovery pi@169.254.17.217:~/RP_Vision_2019/CalibrationRecovery/CalibrationRecovery.o
scp GripPipeline.o pi@169.254.17.217:~/RP_Vision_2019/CalibrationRecovery/GripPipeline.o