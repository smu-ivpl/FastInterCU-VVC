## Description

This project is for traning our proposed CNN architecture called Multi-Level Tree CNN (MLT-CNN). This implementation is based on [BasicSR](https://github.com/xinntao/BasicSR) template.


#### folders in `codes/`
- data : files for handling data
- metrics : metrics for validation
- models : models descripted
- options : configuration files to set model, dataset, and hyper parameters
- utils : utility files

#### main files in `codes/`
- train_mltcnn.py : for training MLT-CNN
- train.py : for traninng ResNet
- test_inference.py : for testing
- model2torchScript.py : for converting trained weights to [TorchScript](https://pytorch.org/docs/stable/jit.html) model

## Environment
- Ubuntu 16.04
- NVIDIA Tesla V100 x2
- Dependencies:
- cuda 10.1
- cudnn 7.6.3
- python 3.7.10
- pytorch 1.7.1
- torchvision 0.8.2
- tqdm 4.60.0
- tensorboard 2.6.0

## Anaconda environment setting
```
conda env create --file thesis_env.yaml
```
or install the packages specified in `thesis_env.yaml`

## Dataset
You can download it from [here](https://drive.google.com/drive/folders/1RsINyxcY0G9_hHUsyIw9Qol4S-fuYgHd?usp=sharing).

## Training
```
cd codes
python train_mltcnn.py -opt options/train/MLTCNN_128x128/Gap_BigMLT_A_BCD_128x128_ORPQ_b128_Adam_600k.yml (MLT-CNN)
python train_mltcnn.py -opt options/train/ResNet_128x128/train_GapBigResNet_ABCD_128x128_Org_b128_600k.yml (ResNet)
```

## Testing
```
cd codes
python test.py -opt options/test/SMD/MLTCNN_128x128/GapBigMLT_aBCD_128x128_ORPQ.yml
```

## Tensorboard
```
tensorboard --logdir=experiments
```
