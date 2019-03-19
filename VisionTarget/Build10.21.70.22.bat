set /P ip=10.21.70.22
set /P copyfilename=VisionTarget
make
pause
scp VisionTarget pi@10.21.70.22:~/VisionTarget/VisionTarget
scp VisionTarget pi@10.21.70.22:~/VisionTarget/VisionTarget.o
scp GripPipeline.o pi@10.21.70.22:~/VisionTarget/GripPipeline.o

