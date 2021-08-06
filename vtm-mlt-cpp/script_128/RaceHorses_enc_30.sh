#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorses.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorses_416x240_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorses_q42.yuv -b ../result/RA_1sec_128/bin/RaceHorses_q42.bin -q 42 -f 30 &> ../result/RA_1sec_128/log/RaceHorses_q42.txt
echo RaceHorses_416x240_30 on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorses.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorses_416x240_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorses_q37.yuv -b ../result/RA_1sec_128/bin/RaceHorses_q37.bin -q 37 -f 30 &> ../result/RA_1sec_128/log/RaceHorses_q37.txt
echo RaceHorses_416x240_30 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorses.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorses_416x240_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorses_q32.yuv -b ../result/RA_1sec_128/bin/RaceHorses_q32.bin -q 32 -f 30 &> ../result/RA_1sec_128/log/RaceHorses_q32.txt
echo RaceHorses_416x240_30 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorses.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorses_416x240_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorses_q27.yuv -b ../result/RA_1sec_128/bin/RaceHorses_q27.bin -q 27 -f 30 &> ../result/RA_1sec_128/log/RaceHorses_q27.txt
echo RaceHorses_416x240_30 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/RaceHorses.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//RaceHorses_416x240_30.yuv -o ../result/RA_1sec_128/yuv/RaceHorses_q22.yuv -b ../result/RA_1sec_128/bin/RaceHorses_q22.bin -q 22 -f 30 &> ../result/RA_1sec_128/log/RaceHorses_q22.txt
echo RaceHorses_416x240_30 on QP22 Done

CUDA_VISIBLE_DEVICES=1 ./RitualDance_enc_60.sh
