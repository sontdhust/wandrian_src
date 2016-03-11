rm -rf tmp
mkdir tmp
cp -r ../include ./tmp
cp -r ../src ./tmp

cd tmp/src/common
g++ -c point.cpp
g++ -c segment.cpp
g++ -c vector.cpp
g++ -c polygon.cpp
g++ -c space.cpp

cd ../plans
g++ -c base_plan.cpp

cd boustrophedon_online
g++ -c cell.cpp
g++ -c boustrophedon_online.cpp

cd ../..
g++ -o ../wandrian_run ../../boustrophedon_online_test.cpp common/point.o common/vector.o \
common/segment.o common/polygon.o common/space.o plans/base_plan.o \
plans/boustrophedon_online/cell.o plans/boustrophedon_online/boustrophedon_online.o -lglut -lGL
cd ..
clear
clear
./wandrian_run $1

cd ..
rm -rf tmp