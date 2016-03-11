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

cd ../environment
g++ -c cell.cpp
g++ -c partially_occupiable.cpp
g++ -c partially_occupiable_cell.cpp

cd ../plans
g++ -c base_plan.cpp

cd spiral_stc
g++ -c spiral_stc.cpp
g++ -c full_spiral_stc.cpp

cd ../..
g++ -o ../wandrian_run \
../../spiral_stc_test.cpp \
common/point.o common/vector.o common/segment.o common/polygon.o common/space.o \
plans/base_plan.o \
environment/cell.o environment/partially_occupiable.o environment/partially_occupiable_cell.o \
plans/spiral_stc/spiral_stc.o plans/spiral_stc/full_spiral_stc.o \
-lglut -lGL
cd ..
clear
clear
./wandrian_run $1 $2 $3

cd ..
rm -rf tmp