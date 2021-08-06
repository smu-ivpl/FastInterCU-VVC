#! /bin/bash


#../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/ParkRunning3.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//ParkRunning3_3840x2160_50fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/ParkRunning3_q42.yuv -b ../result/RA_1sec_128/bin/ParkRunning3_q42.bin -q 42 -f 50 &> ../result/RA_1sec_128/log/ParkRunning3_q42.txt
#echo ParkRunning3_3840x2160_50fps_10bit_420 on QP42 Done

#../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/ParkRunning3.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//ParkRunning3_3840x2160_50fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/ParkRunning3_q37.yuv -b ../result/RA_1sec_128/bin/ParkRunning3_q37.bin -q 37 -f 50 &> ../result/RA_1sec_128/log/ParkRunning3_q37.txt
#echo ParkRunning3_3840x2160_50fps_10bit_420 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/ParkRunning3.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//ParkRunning3_3840x2160_50fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/ParkRunning3_q32.yuv -b ../result/RA_1sec_128/bin/ParkRunning3_q32.bin -q 32 -f 50 &> ../result/RA_1sec_128/log/ParkRunning3_q32.txt
echo ParkRunning3_3840x2160_50fps_10bit_420 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/ParkRunning3.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//ParkRunning3_3840x2160_50fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/ParkRunning3_q27.yuv -b ../result/RA_1sec_128/bin/ParkRunning3_q27.bin -q 27 -f 50 &> ../result/RA_1sec_128/log/ParkRunning3_q27.txt
echo ParkRunning3_3840x2160_50fps_10bit_420 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/ParkRunning3.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//ParkRunning3_3840x2160_50fps_10bit_420.yuv -o ../result/RA_1sec_128/yuv/ParkRunning3_q22.yuv -b ../result/RA_1sec_128/bin/ParkRunning3_q22.bin -q 22 -f 50 &> ../result/RA_1sec_128/log/ParkRunning3_q22.txt
echo ParkRunning3_3840x2160_50fps_10bit_420 on QP22 Done

