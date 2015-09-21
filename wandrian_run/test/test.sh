rm -rf tmp
mkdir tmp
cp -r ../include ./tmp
cp -r ../src ./tmp

cd tmp/src/common
g++ -c point.cpp
g++ -c segment.cpp
g++ -c vector.cpp
g++ -c polygon.cpp
g++ -c environment.cpp

cd ../plans/spiral_stc
g++ -c cell.cpp
g++ -c spiral_stc.cpp

cd ../..
g++ -o ../wandrian_run ../../test.cpp common/point.o common/vector.o common/segment.o common/polygon.o common/environment.o plans/spiral_stc/cell.o plans/spiral_stc/spiral_stc.o -lglut -lGL
cd ..
./wandrian_run