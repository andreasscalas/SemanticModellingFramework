#!/bin/bash

#export environment variables
export PHOTOSCAN_EXEC='/home/jachenba/techfak_home_jascha_share/uni/bin/metashape-pro/metashape.sh'
#make sure to end paths with a '/'
export CHARACTER_GENERATOR_DB='/home/andreas/Documenti/Progetti/Andreas_Statue_Project/data/CharacterGenerator_DB/'

#we need at least one project directory
if [ "$#" -lt 1 ]
  then
    echo "Usage: $0 <projectfolder_face> [<additional_args>]"
    echo "Arguments:"
    echo "    -c <config_file>"
    echo "    -o <output_file>"
    echo "    --hidewindow"
    exit 1
fi

start_time=$SECONDS


#use these variables to access project folders
projectfolder_face=$1
shift


working_dir=$(pwd)
scripts_dir="$working_dir/scripts/"

#prepare project folder
"$scripts_dir/bash/01_prepare_face.sh" $projectfolder_face

# copy camera calibrations if available
#if [ ! -e "$projectfolder_head/cameras.xml" ] && [ -e "cameras_face.xml" ]; 
#    then
#    cp "cameras_face.xml" "$projectfolder_face/cameras.xml" -v
#fi


#if pointset not computed yet, compute it
#if [ ! -f "$projectfolder_face/pointset.txt" ]; 
#    then

#    #convert images with white balance
#    "$scripts_dir/bash/00_convert_images.sh" $projectfolder_face "2.075334 1.0 1.707050 1.000552" "1.4"
#    #compute point cloud with photoscan
#    $PHOTOSCAN_EXEC -r "$scripts_dir/photoscan/photos2points.py" $projectfolder_face
#fi


#start fitting
./points2head -h $projectfolder_face -t $CHARACTER_GENERATOR_DB -c p2head_config.cfg $@


elapsed_time=$(($SECONDS - $start_time))

echo "Overall time: $(($elapsed_time/60)) min $(($elapsed_time%60)) sec"

exit 0
