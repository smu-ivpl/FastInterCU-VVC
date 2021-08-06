import os
import torch
import models.archs.mlt_cu_or_pq_arch as arch
def main():
    #################
    # configurations
    #################
    device = torch.device('cuda')
    os.environ['CUDA_VISIBLE_DEVICES'] = '0'

    #model_base_path = '/home/ubuntu/whyeo/Thesis/mlt-cnn/experiments/PreAct_Pool_BigMLT_BCD_128x128_ORI_b128_Adam_lr4e-4_600k/models/'
    #model_source_path = model_base_path + 'net_190000.pth'
    #model_output_path = '/home/ubuntu/whyeo/VTM-11.0_BResnetOR/torch_model/MLTORI_splitMode_128.pt'
    model_base_path = '/home/ubuntu/whyeo/Thesis/mlt-cnn/experiments/Gap_Big_MLT_ABCD_16x16_ORPQ_b128_Adam_lr4e-4_600k/models/'
    model_source_path = model_base_path + 'net_250000.pth'
    # 128 - aBCD_240k: 0.817 // A_BCD_310k: 0.8186 // Freeze_~_A_BCD_152k: 0.823
    # 64 - ABCD_205k: 0.6123
    # 32 - ABCD_190k: 0.4725
    # 16 - ABCD_250k: 0.4926
    model_output_path = '/home/ubuntu/whyeo/VTM-11.0_BResnetOR/torch_model/MLTORPQ_splitMode_16.pt'

    model = arch.GapBigMltCuORPQ()
    load_net = torch.load(
        model_source_path, map_location=lambda storage, loc: storage)
    load_net = load_net['params']
    # remove unnecessary 'module.'
    import copy
    for k, v in copy.deepcopy(load_net).items():
        if k.startswith('module.'):
            load_net[k[7:]] = v
            load_net.pop(k)
    model.load_state_dict(load_net, strict=False)

    model.eval()
    model = model.to(device)

    org = torch.rand(1, 1, 128, 128).to(device)
    resi = torch.rand(1, 1, 128, 128).to(device)
    input = torch.cat((org, resi), 1)
    poc = torch.rand(1).to(device)
    qp = torch.rand(1).to(device)
    # n0 = torch.rand(1).to(device)
    # n1 = torch.rand(1).to(device)
    # n2 = torch.rand(1).to(device)
    # n3 = torch.rand(1).to(device)
    traced_script_module = torch.jit.trace(model, (input, poc, qp))#, n0, n1, n2, n3))

    traced_script_module.save(model_output_path)


if __name__ == '__main__':
    main()

"""
/* B3ResNet OrgResi */
import os
import torch
import models.archs.branch3_resnet_arch as arch
def main():
    #################
    # configurations
    #################
    device = torch.device('cuda')
    os.environ['CUDA_VISIBLE_DEVICES'] = '0'

    model_base_path = '/home/ubuntu/whyeo/Thesis/fast-cu-split-decision-cnn/experiments/Branch3ResNet_SMD_128x128_OrgResi_lr4e-4_600k_b128/models/'
    model_source_path = model_base_path + 'net_545000.pth'
    model_output_path = '/home/ubuntu/whyeo/VTM-11.0_BResnetOR/torch_model/BResNetOR_splitMode_128.pt'
    model = arch.B3ResNetOrgResi()

    model.load_state_dict(torch.load(model_source_path), strict=False) # True?
    model.eval()
    model = model.to(device)

    org = torch.rand(1, 1, 128, 128).to(device)
    resi = torch.rand(1, 1, 128, 128).to(device)
    input = torch.cat((org, resi), 1)
    poc = torch.rand(1).to(device)
    qp = torch.rand(1).to(device)
    traced_script_module = torch.jit.trace(model, (input, poc, qp))

    traced_script_module.save(model_output_path)


if __name__ == '__main__':
    main()

"""