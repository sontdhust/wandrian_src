rm -rf tmp
mkdir tmp
cp -r ../include ./tmp
cp -r ../src ./tmp
cd tmp/src/common
g++ -c point.cpp
g++ -c segment.cpp
g++ -c polygon.cpp
g++ -c environment.cpp
cd ..
g++ -o ../wandrian_run ../../test.cpp common/point.o common/segment.o common/polygon.o common/environment.o -lglut -lGL
cd ..
./wandrian_run