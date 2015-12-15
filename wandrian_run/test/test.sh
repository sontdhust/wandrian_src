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

cd online_boustrophedon
g++ -c cell.cpp
g++ -c online_boustrophedon.cpp

cd ../..
g++ -o ../wandrian_run ../../test.cpp common/point.o common/vector.o common/segment.o common/polygon.o common/environment.o plans/base_plan.o plans/online_boustrophedon/cell.o plans/online_boustrophedon/online_boustrophedon.o -lglut -lGL
cd ..
clear
clear
./wandrian_run $1

cd ..
rm -rf tmp