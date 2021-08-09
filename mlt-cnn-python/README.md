## Description

This project is for traning our proposed CNN architecture called Multi-Level Tree CNN (MLT-CNN). This implementation is based on [BasicSR](https://github.com/xinntao/BasicSR) template.


#### folders in `codes/`
- data : files for handling data
- metrics : metrics for validation
- models : models descripted
- options : configuration files to set model, dataset, and hyper parameters
- utils : utility files

#### main files
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
conda env create --file environment.yaml
```
or install the packages specified in `thesis_env.yaml`

## Dataset

## Training

## Testing

## Tensorboard
