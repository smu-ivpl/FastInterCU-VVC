#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Campfire.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Campfire_3840x2160_30fps_10bit_420_bt709_videoRange.yuv -o ../result/RA_1sec_128/yuv/Campfire_q42.yuv -b ../result/RA_1sec_128/bin/Campfire_q42.bin -q 42 -f 30 &> ../result/RA_1sec_128/log/Campfire_q42.txt
echo Campfire_3840x2160_30fps_10bit_420_bt709_videoRange on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Campfire.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Campfire_3840x2160_30fps_10bit_420_bt709_videoRange.yuv -o ../result/RA_1sec_128/yuv/Campfire_q37.yuv -b ../result/RA_1sec_128/bin/Campfire_q37.bin -q 37 -f 30 &> ../result/RA_1sec_128/log/Campfire_q37.txt
echo Campfire_3840x2160_30fps_10bit_420_bt709_videoRange on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Campfire.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Campfire_3840x2160_30fps_10bit_420_bt709_videoRange.yuv -o ../result/RA_1sec_128/yuv/Campfire_q32.yuv -b ../result/RA_1sec_128/bin/Campfire_q32.bin -q 32 -f 30 &> ../result/RA_1sec_128/log/Campfire_q32.txt
echo Campfire_3840x2160_30fps_10bit_420_bt709_videoRange on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Campfire.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Campfire_3840x2160_30fps_10bit_420_bt709_videoRange.yuv -o ../result/RA_1sec_128/yuv/Campfire_q27.yuv -b ../result/RA_1sec_128/bin/Campfire_q27.bin -q 27 -f 30 &> ../result/RA_1sec_128/log/Campfire_q27.txt
echo Campfire_3840x2160_30fps_10bit_420_bt709_videoRange on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/Campfire.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//Campfire_3840x2160_30fps_10bit_420_bt709_videoRange.yuv -o ../result/RA_1sec_128/yuv/Campfire_q22.yuv -b ../result/RA_1sec_128/bin/Campfire_q22.bin -q 22 -f 30 &> ../result/RA_1sec_128/log/Campfire_q22.txt
echo Campfire_3840x2160_30fps_10bit_420_bt709_videoRange on QP22 Done

CUDA_VISIBLE_DEVICES=0 ./CatRobot_enc_60.sh
