#!/bin/bash
N_WORLDS=40
M_RUNS=3
DynaBARN=true

for (( i=20; i<$N_WORLDS; i++ )) ; do
    if [ "$DynaBARN" = true ] ; then
        let "n = $i + 300" # DynaBARN world indices: [300, 359]
    else
        let "n = $i * 6" # 50 test BARN worlds with equal spacing indices: [0, 6, 12, ..., 294]
    fi

    for (( j=0; j<$M_RUNS; j++ )) ; do            
        # run the test
        ./singularity_run.sh /home/saadaghani/jackal_ws/src/nav-competition-icra2022/nav_competition_image.sif python3 run.py --world_idx $n --algo dyna_lflh 
        sleep 10
    done
done
