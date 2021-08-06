#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Tango2.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Tango2_3840x2160_60fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/Tango2_q42.yuv -b ../result/RA_1sec_128/bin/Tango2_q42.bin -q 42 -f 60 &> ../result/RA_1sec_128/log/Tango2_q42.txt
echo Tango2_3840x2160_60fps_10bit_420 on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Tango2.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Tango2_3840x2160_60fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/Tango2_q37.yuv -b ../result/RA_1sec_128/bin/Tango2_q37.bin -q 37 -f 60 &> ../result/RA_1sec_128/log/Tango2_q37.txt
echo Tango2_3840x2160_60fps_10bit_420 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Tango2.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Tango2_3840x2160_60fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/Tango2_q32.yuv -b ../result/RA_1sec_128/bin/Tango2_q32.bin -q 32 -f 60 &> ../result/RA_1sec_128/log/Tango2_q32.txt
echo Tango2_3840x2160_60fps_10bit_420 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Tango2.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Tango2_3840x2160_60fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/Tango2_q27.yuv -b ../result/RA_1sec_128/bin/Tango2_q27.bin -q 27 -f 60 &> ../result/RA_1sec_128/log/Tango2_q27.txt
echo Tango2_3840x2160_60fps_10bit_420 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Tango2.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Tango2_3840x2160_60fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/Tango2_q22.yuv -b ../result/RA_1sec_128/bin/Tango2_q22.bin -q 22 -f 60 &> ../result/RA_1sec_128/log/Tango2_q22.txt
echo Tango2_3840x2160_60fps_10bit_420 on QP22 Done

CUDA_VISIBLE_DEVICES=1 ./MarketPlace_enc_60.sh 
