#! /bin/bash


../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/SlideShow.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//SlideShow_1280x720_20.yuv -o ../result/RA_1sec_128/yuv/SlideShow_q42.yuv -b ../result/RA_1sec_128/bin/SlideShow_q42.bin -q 42 -f 20 &> ../result/RA_1sec_128/log/SlideShow_q42.txt
echo SlideShow_1280x720_20 on QP42 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/SlideShow.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//SlideShow_1280x720_20.yuv -o ../result/RA_1sec_128/yuv/SlideShow_q37.yuv -b ../result/RA_1sec_128/bin/SlideShow_q37.bin -q 37 -f 20 &> ../result/RA_1sec_128/log/SlideShow_q37.txt
echo SlideShow_1280x720_20 on QP37 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/SlideShow.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//SlideShow_1280x720_20.yuv -o ../result/RA_1sec_128/yuv/SlideShow_q32.yuv -b ../result/RA_1sec_128/bin/SlideShow_q32.bin -q 32 -f 20 &> ../result/RA_1sec_128/log/SlideShow_q32.txt
echo SlideShow_1280x720_20 on QP32 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/SlideShow.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//SlideShow_1280x720_20.yuv -o ../result/RA_1sec_128/yuv/SlideShow_q27.yuv -b ../result/RA_1sec_128/bin/SlideShow_q27.bin -q 27 -f 20 &> ../result/RA_1sec_128/log/SlideShow_q27.txt
echo SlideShow_1280x720_20 on QP27 Done

../bin/EncoderAppStatic_128 -c ../cfg/encoder_randomaccess_vtm.cfg -c ../cfg/per-sequence/SlideShow.cfg -i /home/ubuntu/whyeo/Dataset/VTMSequences//SlideShow_1280x720_20.yuv -o ../result/RA_1sec_128/yuv/SlideShow_q22.yuv -b ../result/RA_1sec_128/bin/SlideShow_q22.bin -q 22 -f 20 &> ../result/RA_1sec_128/log/SlideShow_q22.txt
echo SlideShow_1280x720_20 on QP22 Done

