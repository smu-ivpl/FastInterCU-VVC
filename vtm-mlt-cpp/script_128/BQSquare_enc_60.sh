#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BQSquare.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BQSquare_416x240_60.yuv -o ../result/RA_1sec_128/yuv/BQSquare_q42.yuv -b ../result/RA_1sec_128/bin/BQSquare_q42.bin -q 42 -f 60 &> ../result/RA_1sec_128/log/BQSquare_q42.txt
echo BQSquare_416x240_60 on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BQSquare.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BQSquare_416x240_60.yuv -o ../result/RA_1sec_128/yuv/BQSquare_q37.yuv -b ../result/RA_1sec_128/bin/BQSquare_q37.bin -q 37 -f 60 &> ../result/RA_1sec_128/log/BQSquare_q37.txt
echo BQSquare_416x240_60 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BQSquare.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BQSquare_416x240_60.yuv -o ../result/RA_1sec_128/yuv/BQSquare_q32.yuv -b ../result/RA_1sec_128/bin/BQSquare_q32.bin -q 32 -f 60 &> ../result/RA_1sec_128/log/BQSquare_q32.txt
echo BQSquare_416x240_60 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BQSquare.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BQSquare_416x240_60.yuv -o ../result/RA_1sec_128/yuv/BQSquare_q27.yuv -b ../result/RA_1sec_128/bin/BQSquare_q27.bin -q 27 -f 60 &> ../result/RA_1sec_128/log/BQSquare_q27.txt
echo BQSquare_416x240_60 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/BQSquare.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//BQSquare_416x240_60.yuv -o ../result/RA_1sec_128/yuv/BQSquare_q22.yuv -b ../result/RA_1sec_128/bin/BQSquare_q22.bin -q 22 -f 60 &> ../result/RA_1sec_128/log/BQSquare_q22.txt
echo BQSquare_416x240_60 on QP22 Done

CUDA_VISIBLE_DEVICES=0 ./BQTerrace_enc_60.sh
