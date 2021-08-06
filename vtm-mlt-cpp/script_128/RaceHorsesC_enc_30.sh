#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorsesC.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorsesC_832x480_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorsesC_q42.yuv -b ../result/RA_1sec_128/bin/RaceHorsesC_q42.bin -q 42 -f 30 &> ../result/RA_1sec_128/log/RaceHorsesC_q42.txt
echo RaceHorsesC_832x480_30 on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorsesC.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorsesC_832x480_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorsesC_q37.yuv -b ../result/RA_1sec_128/bin/RaceHorsesC_q37.bin -q 37 -f 30 &> ../result/RA_1sec_128/log/RaceHorsesC_q37.txt
echo RaceHorsesC_832x480_30 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorsesC.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorsesC_832x480_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorsesC_q32.yuv -b ../result/RA_1sec_128/bin/RaceHorsesC_q32.bin -q 32 -f 30 &> ../result/RA_1sec_128/log/RaceHorsesC_q32.txt
echo RaceHorsesC_832x480_30 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorsesC.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorsesC_832x480_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorsesC_q27.yuv -b ../result/RA_1sec_128/bin/RaceHorsesC_q27.bin -q 27 -f 30 &> ../result/RA_1sec_128/log/RaceHorsesC_q27.txt
echo RaceHorsesC_832x480_30 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorsesC.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorsesC_832x480_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorsesC_q22.yuv -b ../result/RA_1sec_128/bin/RaceHorsesC_q22.bin -q 22 -f 30 &> ../result/RA_1sec_128/log/RaceHorsesC_q22.txt
echo RaceHorsesC_832x480_30 on QP22 Done

CUDA_VISIBLE_DEVICES=1 ./RaceHorses_enc_30.sh
