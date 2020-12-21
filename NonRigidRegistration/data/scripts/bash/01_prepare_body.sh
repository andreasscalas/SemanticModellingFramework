#!/bin/bash


cd $1

#create necessary directories and copy masks
if [ ! -d "landmarks" ] || [ ! -d "textures" ] || [ ! -d "masks"  ] || [ ! -d "results"  ];
    then

    # make directories
    mkdir landmarks
    mkdir textures
    mkdir masks
    mkdir results

    # copy mask for armpits
    cp $CHARACTER_GENERATOR_DB/masks/armpits-229.png masks/armpits.png -v

    # copy mask for hands
    cp $CHARACTER_GENERATOR_DB/masks/hands-4758-dilat.png masks/hands.png -v
    cp $CHARACTER_GENERATOR_DB/masks/hands-dirichlet.png masks/hands-dirichlet.png -v
fi

