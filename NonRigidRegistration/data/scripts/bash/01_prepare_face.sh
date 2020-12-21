#!/bin/bash


cd $1

if [ ! -d "contour_eyes" ] || [ ! -d "ears" ] || [ ! -d "landmarks" ] || [ ! -d "textures" ] || [ ! -d "results" ];
then
    mkdir contour_eyes
    mkdir ears
    mkdir landmarks
    mkdir textures
    mkdir results
fi
