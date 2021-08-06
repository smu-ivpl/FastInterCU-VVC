#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/CatRobot.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//CatRobot_3840x2160_60fps_10bit_420_jvet.yuv -o ../result/RA_1sec_128/yuv/CatRobot_q42.yuv -b ../result/RA_1sec_128/bin/CatRobot_q42.bin -q 42 -f 60 &> ../result/RA_1sec_128/log/CatRobot_q42.txt
echo CatRobot_3840x2160_60fps_10bit_420_jvet on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/CatRobot.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//CatRobot_3840x2160_60fps_10bit_420_jvet.yuv -o ../result/RA_1sec_128/yuv/CatRobot_q37.yuv -b ../result/RA_1sec_128/bin/CatRobot_q37.bin -q 37 -f 60 &> ../result/RA_1sec_128/log/CatRobot_q37.txt
echo CatRobot_3840x2160_60fps_10bit_420_jvet on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/CatRobot.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//CatRobot_3840x2160_60fps_10bit_420_jvet.yuv -o ../result/RA_1sec_128/yuv/CatRobot_q32.yuv -b ../result/RA_1sec_128/bin/CatRobot_q32.bin -q 32 -f 60 &> ../result/RA_1sec_128/log/CatRobot_q32.txt
echo CatRobot_3840x2160_60fps_10bit_420_jvet on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/CatRobot.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//CatRobot_3840x2160_60fps_10bit_420_jvet.yuv -o ../result/RA_1sec_128/yuv/CatRobot_q27.yuv -b ../result/RA_1sec_128/bin/CatRobot_q27.bin -q 27 -f 60 &> ../result/RA_1sec_128/log/CatRobot_q27.txt
echo CatRobot_3840x2160_60fps_10bit_420_jvet on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/CatRobot.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//CatRobot_3840x2160_60fps_10bit_420_jvet.yuv -o ../result/RA_1sec_128/yuv/CatRobot_q22.yuv -b ../result/RA_1sec_128/bin/CatRobot_q22.bin -q 22 -f 60 &> ../result/RA_1sec_128/log/CatRobot_q22.txt
echo CatRobot_3840x2160_60fps_10bit_420_jvet on QP22 Done

CUDA_VISIBLE_DEVICES=0 ./DaylightRoad2_enc_60.sh
