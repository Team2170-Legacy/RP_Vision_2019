set /P ip=169.254.17.217 
set /P copyfilename=Calibration
make
pause
scp Calibration pi@169.254.17.217:~/RP_Vision_2019/Calibration/Calibration
scp Calibration pi@169.254.17.217:~/RP_Vision_2019/Calibration/Calibration.o
scp GripPipeline.o pi@169.254.17.217:~/RP_Vision_2019/Calibration/GripPipeline.o
