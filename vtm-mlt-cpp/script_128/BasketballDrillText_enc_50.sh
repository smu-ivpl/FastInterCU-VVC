#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrillText.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrillText_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrillText_q42.yuv -b ../result/RA_1sec_128/bin/BasketballDrillText_q42.bin -q 42 -f 50 &> ../result/RA_1sec_128/log/BasketballDrillText_q42.txt
echo BasketballDrillText_832x480_50 on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrillText.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrillText_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrillText_q37.yuv -b ../result/RA_1sec_128/bin/BasketballDrillText_q37.bin -q 37 -f 50 &> ../result/RA_1sec_128/log/BasketballDrillText_q37.txt
echo BasketballDrillText_832x480_50 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrillText.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrillText_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrillText_q32.yuv -b ../result/RA_1sec_128/bin/BasketballDrillText_q32.bin -q 32 -f 50 &> ../result/RA_1sec_128/log/BasketballDrillText_q32.txt
echo BasketballDrillText_832x480_50 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrillText.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrillText_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrillText_q27.yuv -b ../result/RA_1sec_128/bin/BasketballDrillText_q27.bin -q 27 -f 50 &> ../result/RA_1sec_128/log/BasketballDrillText_q27.txt
echo BasketballDrillText_832x480_50 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrillText.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrillText_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrillText_q22.yuv -b ../result/RA_1sec_128/bin/BasketballDrillText_q22.bin -q 22 -f 50 &> ../result/RA_1sec_128/log/BasketballDrillText_q22.txt
echo BasketballDrillText_832x480_50 on QP22 Done

CUDA_VISIBLE_DEVICES=0 ./BasketballDrive_enc_50.sh
