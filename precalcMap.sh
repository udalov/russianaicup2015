#!/bin/bash

for map in default map01 map02 map03 map04 map05 map06 map07 map08 map09 map10 map11 map12 map13 map14 map15 map16 map17 map18 map19 map20 map21 _fdoke _tyamgin _ud1
do
    sed -i '' "s/map=.*$/map=$map/" lib/local-runner-console.properties
    echo $map
    out/LocalRunner 2>/dev/null | tee -a maps
done
