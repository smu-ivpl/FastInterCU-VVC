#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/FourPeople.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//FourPeople_1280x720_60.yuv -o ../result/RA_1sec_128/yuv/FourPeople_q42.yuv -b ../result/RA_1sec_128/bin/FourPeople_q42.bin -q 42 -f 60 &> ../result/RA_1sec_128/log/FourPeople_q42.txt
echo FourPeople_1280x720_60 on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/FourPeople.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//FourPeople_1280x720_60.yuv -o ../result/RA_1sec_128/yuv/FourPeople_q37.yuv -b ../result/RA_1sec_128/bin/FourPeople_q37.bin -q 37 -f 60 &> ../result/RA_1sec_128/log/FourPeople_q37.txt
echo FourPeople_1280x720_60 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/FourPeople.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//FourPeople_1280x720_60.yuv -o ../result/RA_1sec_128/yuv/FourPeople_q32.yuv -b ../result/RA_1sec_128/bin/FourPeople_q32.bin -q 32 -f 60 &> ../result/RA_1sec_128/log/FourPeople_q32.txt
echo FourPeople_1280x720_60 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/FourPeople.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//FourPeople_1280x720_60.yuv -o ../result/RA_1sec_128/yuv/FourPeople_q27.yuv -b ../result/RA_1sec_128/bin/FourPeople_q27.bin -q 27 -f 60 &> ../result/RA_1sec_128/log/FourPeople_q27.txt
echo FourPeople_1280x720_60 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/FourPeople.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//FourPeople_1280x720_60.yuv -o ../result/RA_1sec_128/yuv/FourPeople_q22.yuv -b ../result/RA_1sec_128/bin/FourPeople_q22.bin -q 22 -f 60 &> ../result/RA_1sec_128/log/FourPeople_q22.txt
echo FourPeople_1280x720_60 on QP22 Done

