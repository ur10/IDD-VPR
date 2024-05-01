#!/bin/bash

#SBATCH -A ur10
#SBATCH -c 8
#SBATCH --gres=gpu:0
#SBATCH --nodelist gnode025
#SBATCH --mem-per-cpu=4G
#SBATCH --time=3-00:00:00
#SBATCH --output=output.txt


cd /scratch/ur10
mkdir 2023-10-03
cd 2023-10-03

sshpass -p "GBXdMlJ&EfbKAYBZ" rsync -r --info=progress2  utkarsh_rai@10.4.16.30:/mnt/base/idd_comprehensive/2023-10-03/route3 ./
