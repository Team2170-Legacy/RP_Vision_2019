set /P ip=10.21.70.22
set /P copyfilename=DistanceCalibration
make
pause
scp DistanceCalibration pi@10.21.70.22:~/DistanceCalibration/DistanceCalibration
scp DistanceCalibration pi@10.21.70.22:~/DistanceCalibration/DistanceCalibration.o
scp GripPipeline.o pi@10.21.70.22:~/DistanceCalibration/GripPipeline.o

