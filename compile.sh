g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"calicam.d" -MT"calicam.o" -o "calicam.o" "./calicam.cpp"
g++  -o "calicam"  ./calicam.o   -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_ccalib -lopencv_imgproc -lopencv_calib3d -lopencv_imgcodecs
rm *.d *.o
