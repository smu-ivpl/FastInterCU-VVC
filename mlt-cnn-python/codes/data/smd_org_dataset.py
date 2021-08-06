import os
import numpy as np
from PIL import Image
from pathlib import Path
from torch.utils import data as data
import torchvision.transforms as transforms
from csv import reader

class SMDOrgDataset(data.Dataset):
    """Org/ 128x128, 64x64, 32x32, 16x16 / CU image / Split mode decision (SMD) dataset for training.

    Dataset Directory structure:
    dataroot_org/sequence-name/org/baseQP_POC_X_Y.png (128,128,1)
    dataroot_pred/sequence-name/pred/baseQP_POC_X_Y.png (128,128,1)
    dataroot_pred/sequence-name/reco/baseQP_POC_X_Y.png (128,128,1)

    GT (gt): Ground-Truth; SPLIT mode; 0 ~ 5
    ORG (org): Orginal CU image (128,128,1)

    Args:
        opt (dict): Config for train dataset. It contains the following keys:
            dataroot_org (str): Data root path for original CU images.
            dataroot_reco (str): Data root path for reconstructed CU images.
    """
    def __init__(self, opt):
        super(SMDOrgDataset, self).__init__()
        self.opt = opt

        self.org_root = opt['dataroot_org']
        self.csv = opt['data_csv']

        all_list = []
        with open(self.csv, 'r') as f:
            csv_reader = reader(f)
            for row in csv_reader:
                all_list.append(row)
        self.all_info = all_list

        self.transform = transforms.ToTensor()

    def __getitem__(self, idx):
        seq_name = self.all_info[idx][0]
        baseQP = self.all_info[idx][1]
        poc = self.all_info[idx][2]
        x = self.all_info[idx][3]
        y = self.all_info[idx][4]

        img_fname = baseQP+'_'+poc+'_'+x+'_'+y+'.png'
        org_path = os.path.join(self.org_root, seq_name, 'org', img_fname)

        # Load 10bit image -> range[0, 1]
        img_org = np.asarray(Image.open(org_path)) / 1023

        org = self.transform(img_org).float()

        label = int(self.all_info[idx][5])
        cuQP = int(self.all_info[idx][6])
        cost = np.float32(self.all_info[idx][7])

        return {'org': org,
                'gt': label}#, 'poc': int(poc), 'qp': cuQP, 'cost': cost}

    def __len__(self):
        return len(self.all_info)