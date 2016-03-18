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

cd ../plans
g++ -c base_plan.cpp

cd boustrophedon_off
g++ -c cell.cpp
g++ -c obstacle.cpp
g++ -c space.cpp
g++ -c vertices.cpp
g++ -c environmentoff.cpp
g++ -c boustrophedon.cpp

cd ../..
g++ -o ../wandrian_run ../../test.cpp common/point.o common/vector.o common/segment.o common/polygon.o common/environment.o plans/base_plan.o plans/boustrophedon_off/cell.o plans/boustrophedon_off/obstacle.o plans/boustrophedon_off/vertices.o plans/boustrophedon_off/environmentoff.o plans/boustrophedon_off/space.o plans/boustrophedon_off/boustrophedon.o -lglut -lGL
cd ..
clear
clear
./wandrian_run $1

cd ..
rm -rf tmp
