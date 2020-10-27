#!/bin/bash

train_data=../../data/Training_final_test

python2.7 train.py --checkpoint_dir=./results/best_final_1024 --f=0.5 --train_dir=$train_data --summary_freq=100 --batch_size=200 --max_epochs=100 --num_threads=6 --resume_train=False
