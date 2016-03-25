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
g++ -c partially_occupiable.cpp
g++ -c partially_occupiable_cell.cpp
g++ -c space.cpp

cd ../plans
g++ -c base_plan.cpp

cd spiral_stc
g++ -c spiral_stc.cpp
g++ -c full_spiral_stc.cpp

cd ../boustrophedon_online
g++ -c boustrophedon_online.cpp

cd ../..
g++ -o ../wandrian \
../../test.cpp \
common/point.o common/vector.o common/segment.o common/polygon.o common/rectangle.o \
plans/base_plan.o \
environment/cell.o environment/partially_occupiable.o environment/partially_occupiable_cell.o environment/space.o \
plans/spiral_stc/spiral_stc.o plans/spiral_stc/full_spiral_stc.o \
plans/boustrophedon_online/boustrophedon_online.o \
-lglut -lGL
cd ..
clear
clear
./wandrian $1 $2 $3 $4

cd ..
rm -rf tmp
