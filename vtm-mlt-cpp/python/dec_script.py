import os
import numpy as np

orig_path = '/home/whyeo/VVCWorkspace/VTMSequences'
fnames = os.listdir(orig_path)
qp = (22, 27, 32, 37, 42)

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

    n_frame = 50
    #n_frame = fps

    seq_name = seg2[0]
    script = open('../script_resnetQP/%s_dec.sh' % seq_name, 'w')
    script.writelines('#! /bin/bash\n\n\n')
    for i, q in enumerate(qp):
        result_name = seq_name + '_q' + str(q)
        script.writelines('../bin/umake/gcc-8.4/x86_64/release/DecoderApp '
                          '-o ../DEC_RESULT_resnetQP/RA/yuv/%s.yuv '
                          '-b ../ENC_RESULT_resnetQP/RA/bin/%s.bin '
                          '&> ../DEC_RESULT_resnetQP/RA/log/%s.txt\n' % (result_name, result_name, result_name))
        script.writelines('echo %s on QP%d Done\n\n' % (seg1[0], q))
    script.close()
    print(n + 1)
