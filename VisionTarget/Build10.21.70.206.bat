set /P ip=10.21.70.206
set /P copyfilename=VisionTarget
make
pause
scp VisionTarget pi@10.21.70.206:~/VisionTarget/VisionTarget
scp VisionTarget pi@10.21.70.206:~/VisionTarget/VisionTarget.o
scp GripPipeline.o pi@10.21.70.206:~/VisionTarget/GripPipeline.o

