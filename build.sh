echo "Configuring and building Thirdparty/line_descriptor ..."

cd Thirdparty/line_descriptor
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../DBoW2

echo "Configuring and building Thirdparty/DBoW2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../../

#echo "Uncompress vocabulary ..."

#cd Vocabulary
#tar -xf ORBvoc.txt.tar.gz
#tar -xf LSvoc.txt.tar.gz
#cd ..

echo "Configuring and building PLI_SLAM ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
