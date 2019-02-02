set /P ip=Enter PI IP: 
set /P copyfilename=Enter Executable Filename:
make
pause
scp %copyfilename% pi@%ip%:~/RP_Vision_2019/NetworkTables/%copyfilename%
pause
scp GripPipeline.o pi@%ip%:~/RP_Vision_2019/NetworkTables/GripPipeline.o