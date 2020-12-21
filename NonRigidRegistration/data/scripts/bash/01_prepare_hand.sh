#!/bin/bash


cd $1

#create necessary directories and copy masks
if [ ! -d "landmarks" ] || [ ! -d "textures" ] || [ ! -d "results"  ];
    then

    # make directories
    mkdir landmarks
    mkdir textures
    mkdir results
fi

