set /P ip=Enter PI IP: 
set /P copyfilename=Enter Executable Filename:
make
pause
scp %copyfilename% pi@%ip%:~/RP_Vision_2019/Network/%copyfilename%
pause
scp %copyfilename% pi@%ip%:~/RP_Vision_2019/Network/%copyfilename%.o
pause
scp GripPipeline.o pi@%ip%:~/RP_Vision_2019/Network/GripPipeline.o
pause