name="MyStrategy"

if [ ! -f $name.cpp ]
then
    echo Unable to find $name.cpp
    exit 1
fi

rm -f $name

files=""

for i in *.cpp
do
    if [[ $i == "main.cpp" ]]; then continue; fi
    # if [[ $i == "runner-main.cpp" ]]; then continue; fi
    files="$files $i"
done

for i in model/*.cpp
do
    files="$files $i"
done

for i in csimplesocket/*.cpp
do
    files="$files $i"
done

g++-5 -std=c++11 -fno-optimize-sibling-calls -fno-strict-aliasing -DONLINE_JUDGE -D__APPLE__ -D_DARWIN -x c++ -O2 -Wall -Wno-unknown-pragmas -Wno-unused-but-set-variable -o $name $files
