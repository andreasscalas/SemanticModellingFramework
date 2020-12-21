#!/bin/bash


if [ "$#" != 3 ]
  then
    echo "Usage: $0 <projectfolder1> <white-balance-params> <dcraw-brightness-param>"
    exit 1
fi

#count raw files
count=`ls -1 $1/*.cr2 2>/dev/null | wc -l`

num_parallel_processes=4

#we have raw files
if [ $count -gt 0 ]; then
    #check if already converted
    count=`ls -1 $1/*.jpg 2>/dev/null | wc -l`
    if [ $count == 0 ]; then

        #first convert raw to ppm
        raws=$1/*.cr2
        c=0
        for r in $raws
        do
        
            c=$((c+1))
            
            echo "Converting $r to ppm"
#           # our scanner: 1.4 for body and face / würzi scanner: 1.0 for body and 2.7 for hands
            dcraw -W -b $3 -j -r $2 $r &
            
            if [ $((c % $num_parallel_processes)) -eq 0 ]; then
                wait
            fi
            
        done
        
        echo "Waiting for processes..."
        wait
        echo "...done"
        
        c=0
        #then, convert ppm to jpg
        for r in $raws
        do
            c=$((c+1))
            
            fname=`basename ${r}`
            jpgname=${fname:0:${#fname}-4}.jpg
            ppmname=${fname:0:${#fname}-4}.ppm
            echo "Converting $ppmname to $jpgname"
            convert $1/$ppmname $1/$jpgname &
            
            if [ $((c % $num_parallel_processes)) -eq 0 ]; then
                wait
            fi
        done
        
        echo "Waiting for processes..."
        wait
        echo "...done"
        
        rm $1/*.ppm
    else
        echo "Already converted. Done."
    fi
else
    echo "No RAW (cr2) files found in $1."
fi


