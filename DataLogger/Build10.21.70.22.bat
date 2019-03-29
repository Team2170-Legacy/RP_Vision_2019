set /P ip=10.21.70.22
set /P copyfilename=DataLogger
make
pause
scp DataLogger pi@10.21.70.22:~/DataLogger/DataLogger
scp DataLogger pi@10.21.70.22:~/DataLogger/DataLogger.o


