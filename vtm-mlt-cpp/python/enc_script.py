import os
import numpy as np

orig_path = '/home/ubuntu/whyeo/Dataset/VTMSequences/'
fnames = os.listdir(orig_path)
qp = (42, 37, 32, 27, 22)

for n, fn in enumerate(fnames):
    seg1 = fn.split('.')
    seg2 = seg1[0].split('_')
    size = seg2[1].split('x')
    fps = int(seg2[2].split('fps')[0])
    w = int(size[0])
    h = int(size[1])

    # f_size = ((w * h) + (w * h // 2))
    #
    # total_size = os.path.getsize(os.path.join(orig_path, fn))
    # n_frame = (total_size // f_size) >> 1

    #n_frame = 50
    n_frame = fps

    seq_name = seg2[0]
    script = open('../script_16/%s_enc_%d.sh' % (seq_name, fps), 'w')
    script.writelines('#! /bin/bash\n\n\n')
    for i, q in enumerate(qp):
        result_name = seq_name + '_q' + str(q)
        script.writelines('../bin/EncoderAppStatic_16 '
                          '-c ../cfg/encoder_randomaccess_vtm.cfg '
                          '-c ../cfg/per-sequence/%s.cfg '
                          '-i %s/%s '
                          '-o ../result/RA_1sec_16/yuv/%s.yuv '
                          '-b ../result/RA_1sec_16/bin/%s.bin '
                          '-q %d '
                          '-f %d '
                          '&> ../result/RA_1sec_16/log/%s.txt\n' % (seq_name, orig_path, fn, result_name, result_name, q, n_frame, result_name))
        script.writelines('echo %s on QP%d Done\n\n' % (seg1[0], q))
    script.close()
    print(n + 1)
