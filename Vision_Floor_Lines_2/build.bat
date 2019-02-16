set /P ip=Enter PI IP: 
set /ProjectName=Enter Project Name:
set /P copyfilename=Enter Executable Filename:
make
pause
scp %copyfilename% pi@%ip%:~/RP_Vision_2019/%ProjectName%/%copyfilename%
pause
scp %copyfilename% pi@%ip%:~/RP_Vision_2019/%ProjectName%/%copyfilename%.o
pause
scp GripPipeline.o pi@%ip%:~/RP_Vision_2019/%ProjectName%/GripPipeline.o
pause