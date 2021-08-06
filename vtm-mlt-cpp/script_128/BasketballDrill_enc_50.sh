#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrill.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrill_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrill_q42.yuv -b ../result/RA_1sec_128/bin/BasketballDrill_q42.bin -q 42 -f 50 &> ../result/RA_1sec_128/log/BasketballDrill_q42.txt
echo BasketballDrill_832x480_50 on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrill.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrill_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrill_q37.yuv -b ../result/RA_1sec_128/bin/BasketballDrill_q37.bin -q 37 -f 50 &> ../result/RA_1sec_128/log/BasketballDrill_q37.txt
echo BasketballDrill_832x480_50 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrill.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrill_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrill_q32.yuv -b ../result/RA_1sec_128/bin/BasketballDrill_q32.bin -q 32 -f 50 &> ../result/RA_1sec_128/log/BasketballDrill_q32.txt
echo BasketballDrill_832x480_50 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrill.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrill_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrill_q27.yuv -b ../result/RA_1sec_128/bin/BasketballDrill_q27.bin -q 27 -f 50 &> ../result/RA_1sec_128/log/BasketballDrill_q27.txt
echo BasketballDrill_832x480_50 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BasketballDrill.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BasketballDrill_832x480_50.yuv -o ../result/RA_1sec_128/yuv/BasketballDrill_q22.yuv -b ../result/RA_1sec_128/bin/BasketballDrill_q22.bin -q 22 -f 50 &> ../result/RA_1sec_128/log/BasketballDrill_q22.txt
echo BasketballDrill_832x480_50 on QP22 Done

CUDA_VISIBLE_DEVICES=0 ./BasketballDrillText_enc_50.sh
