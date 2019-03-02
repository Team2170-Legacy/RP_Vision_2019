set /P ip=169.254.247.86
set /P copyfilename=Two_Camera_Test
make
pause
scp Two_Camera_Test pi@169.254.247.86:~/RP_Vision_2019/Two_Camera_Test/Two_Camera_Test 
scp Two_Camera_Test pi@169.254.247.86:~/RP_Vision_2019/Two_Camera_Test/Two_Camera_Test.o
scp GripPipeline.o pi@169.254.247.86:~/RP_Vision_2019/Two_Camera_Test/GripPipeline.o
