set /P ip=169.254.11.97
set /P copyfilename=VisionTarget
make
pause
scp VisionTarget pi@169.254.11.97:~/VisionTarget/VisionTarget
scp VisionTarget pi@169.254.11.97:~/VisionTarget/VisionTarget.o
scp GripPipeline.o pi@169.254.11.97:~/VisionTarget/GripPipeline.o

