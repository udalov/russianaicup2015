#!/bin/bash

files=""

for file in *.cpp *.h
do
    if [[ $file == "main.cpp" ]]; then continue; fi
    if [[ $file == "runner-main.cpp" ]]; then continue; fi
    files="$files $file"
done

rm -f out/solution.zip
zip -r out/solution.zip $files
