set /P ip=Enter PI IP: 
set /P copyfilename=Enter Project Name:
make
pause
scp %copyfilename% pi@%ip%:~/RP_Vision_2019/%copyfilename%/%copyfilename%
scp %copyfilename% pi@%ip%:~/RP_Vision_2019/%copyfilename%/%copyfilename%.o
scp GripPipeline.o pi@%ip%:~/RP_Vision_2019/%copyfilename%/GripPipeline.o
