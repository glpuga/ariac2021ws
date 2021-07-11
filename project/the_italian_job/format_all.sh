#!/bin/sh -

for d in `ls -d tij*`; do
    cd $d
    ./utils/fixformat.py
    cd ..
done;