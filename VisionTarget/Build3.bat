set /P ip=10.21.70.50
set /P copyfilename=VisionTarget
make
pause
scp VisionTarget pi@10.21.70.50:~/RP_Vision_2019/VisionTarget/VisionTarget
scp VisionTarget pi@10.21.70.50:~/RP_Vision_2019/VisionTarget/VisionTarget.o
scp GripPipeline.o pi@10.21.70.50:~/RP_Vision_2019/VisionTarget/GripPipeline.o

