#!/bin/bash


#we need src and destination
if [ "$#" -lt 2 ]
  then
    echo "Usage: $0 <src> <dst>"
    exit 1
fi

src=$1
dst=$2

mkdir $dst -p

cp $src/points2character $dst -v
#cp $src/points2hand $dst -v
#cp $src/points2head $dst -v
cp $src/p2c_config.cfg $dst -v
cp $src/p2ch_config.cfg $dst -v
#cp $src/p2h_config.cfg $dst -v
#cp $src/p2head_config.cfg $dst -v
cp $src/start_body_and_hands_fitting.sh $dst -v
cp $src/start_body_and_hands_fitting_with_masks.sh $dst -v
cp $src/start_body_and_optional_face_fitting.sh $dst -v
#cp $src/start_hand_fitting.sh $dst -v
#cp $src/start_head_fitting.sh $dst -v
cp $src/*.xml $dst -v
cp $src/kernels $dst -Rv
cp $src/shaders $dst -Rv
cp $src/scripts $dst -Rv
