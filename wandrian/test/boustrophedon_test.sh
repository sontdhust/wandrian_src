rm -rf tmp
mkdir tmp
cp -r ../include ./tmp
cp -r ../src ./tmp

cd tmp/src/common
g++ -c point.cpp
g++ -c segment.cpp
g++ -c vector.cpp
g++ -c polygon.cpp
g++ -c rectangle.cpp

cd ../environment
g++ -c cell.cpp
g++ -c map.cpp

cd boustrophedon
g++ -c space.cpp
g++ -c extended_map.cpp
g++ -c vertices.cpp

cd ../../plans
g++ -c base_plan.cpp

cd boustrophedon
g++ -c boustrophedon.cpp

cd ../..
g++ -o ../wandrian \
../../boustrophedon_test.cpp \
common/point.o common/vector.o common/segment.o common/polygon.o common/rectangle.o \
plans/base_plan.o \
environment/cell.o environment/map.o \
environment/boustrophedon/space.o environment/boustrophedon/extended_map.o environment/boustrophedon/vertices.o \
plans/boustrophedon/boustrophedon.o \
-lglut -lGL
cd ..
clear
clear
./wandrian $1 $2

cd ..
rm -rf tmp
