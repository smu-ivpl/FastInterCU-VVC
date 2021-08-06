import torch
import torch.nn as nn
import torch.nn.functional as F

class PreActBlock(nn.Module):
    '''Pre-activation version of the BasicBlock.
    https://github.com/kuangliu/pytorch-cifar/blob/master/models/preact_resnet.py
    '''
    expansion = 1

    def __init__(self, in_planes, planes, stride=1):
        super(PreActBlock, self).__init__()
        self.bn1 = nn.BatchNorm2d(in_planes)
        self.conv1 = nn.Conv2d(in_planes, planes, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, stride=1, padding=1, bias=False)

        if stride != 1 or in_planes != self.expansion*planes:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_planes, self.expansion*planes, kernel_size=1, stride=stride, bias=False)
            )

    def forward(self, x):
        out = F.relu(self.bn1(x))
        shortcut = self.shortcut(out) if hasattr(self, 'shortcut') else x
        out = self.conv1(out)
        out = self.conv2(F.relu(self.bn2(out)))
        out += shortcut
        return out


class BasicBlock(nn.Module):
    expansion = 1

    def __init__(self, in_planes, planes, stride=1):
        super(BasicBlock, self).__init__()
        self.conv1 = nn.Conv2d(
            in_planes, planes, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(planes)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3,
                               stride=1, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes)

        self.shortcut = nn.Sequential()
        if stride != 1 or in_planes != self.expansion*planes:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_planes, self.expansion*planes,
                          kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(self.expansion*planes)
            )

    def forward(self, x):
        out = F.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        out += self.shortcut(x)
        out = F.relu(out)
        return out

class MltCnnLvl1ORI(nn.Module): # it has pooling layer
    # 128x128: 2, 3, 4
    def __init__(self, block, num_blocks, num_cls_l1=2):
        super(MltCnnLvl1ORI, self).__init__()
        self.in_planes = 32

        self.conv1 = nn.Conv2d(2, 32, kernel_size=3,
                               stride=1, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(32)
        self.layer1 = self._make_layer(block, 32, num_blocks[0], stride=2)
        self.branch1 = nn.Linear(32*32*32+4, num_cls_l1)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)

    def _make_layer(self, block, planes, num_blocks, stride):
        strides = [stride] + [1]*(num_blocks-1)
        layers = []
        for stride in strides:
            layers.append(block(self.in_planes, planes, stride))
            self.in_planes = planes * block.expansion
        return nn.Sequential(*layers)

    def forward(self, x, poc, qp, n0, n1, n2, n3):
        poc = torch.unsqueeze(poc, 1)
        qp = torch.unsqueeze(qp, 1)
        n0 = torch.unsqueeze(n0, 1)
        n1 = torch.unsqueeze(n1, 1)
        n2 = torch.unsqueeze(n2, 1)
        n3 = torch.unsqueeze(n3, 1)

        #out = self.conv1(F.relu(self.bn1(x)))
        out = self.conv1(x) # modified for PreAct Resnet
        out = self.layer1(out)
        # --- First branch --- #
        lvl1 = F.avg_pool2d(out, 2)
        lvl1 = lvl1.view(lvl1.size(0), -1)
        lvl1 = torch.cat([lvl1, poc, qp, n0, n1+n2+n3], dim=1)
        lvl1 = self.branch1(lvl1)
        return lvl1


class MltCnnLvl1ORIv2(nn.Module): # it has pooling layer + 2resblock
    # 128x128: 2, 3, 4
    def __init__(self, block, num_blocks, num_cls_l1=2):
        super(MltCnnLvl1ORIv2, self).__init__()
        self.in_planes = 32

        self.conv1 = nn.Conv2d(2, 32, kernel_size=3,
                               stride=1, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(32)
        self.layer0 = self._make_layer(block, 32, num_blocks[0], stride=2)
        self.layer1 = self._make_layer(block, 64, num_blocks[1], stride=2)
        self.branch1 = nn.Linear(64*16*16+4, num_cls_l1)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)

    def _make_layer(self, block, planes, num_blocks, stride):
        strides = [stride] + [1]*(num_blocks-1)
        layers = []
        for stride in strides:
            layers.append(block(self.in_planes, planes, stride))
            self.in_planes = planes * block.expansion
        return nn.Sequential(*layers)

    def forward(self, x, poc, qp, n0, n1, n2, n3):
        poc = torch.unsqueeze(poc, 1)
        qp = torch.unsqueeze(qp, 1)
        n0 = torch.unsqueeze(n0, 1)
        n1 = torch.unsqueeze(n1, 1)
        n2 = torch.unsqueeze(n2, 1)
        n3 = torch.unsqueeze(n3, 1)

        #out = self.conv1(F.relu(self.bn1(x)))
        out = self.conv1(x) # modified for PreAct Resnet
        out = self.layer0(out)
        out = self.layer1(out)
        # --- First branch --- #
        lvl1 = F.avg_pool2d(out, 2)
        lvl1 = lvl1.view(lvl1.size(0), -1)
        lvl1 = torch.cat([lvl1, poc, qp, n0, n1+n2+n3], dim=1)
        lvl1 = self.branch1(lvl1)
        return lvl1

class MltCnnLvl1ORIv3(nn.Module): # it has GA pooling layer + 2 resblcok
    # 128x128: 2, 3, 4
    def __init__(self, block, num_blocks, num_cls_l1=2):
        super(MltCnnLvl1ORIv3, self).__init__()
        self.in_planes = 32

        self.conv1 = nn.Conv2d(2, 32, kernel_size=3,
                               stride=1, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(32)
        self.layer0 = self._make_layer(block, 32, num_blocks[0], stride=2)
        self.layer1 = self._make_layer(block, 64, num_blocks[1], stride=2)
        self.branch1 = nn.Linear(64+4, num_cls_l1)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)

    def _make_layer(self, block, planes, num_blocks, stride):
        strides = [stride] + [1]*(num_blocks-1)
        layers = []
        for stride in strides:
            layers.append(block(self.in_planes, planes, stride))
            self.in_planes = planes * block.expansion
        return nn.Sequential(*layers)

    def forward(self, x, poc, qp, n0, n1, n2, n3):
        poc = torch.unsqueeze(poc, 1)
        qp = torch.unsqueeze(qp, 1)
        n0 = torch.unsqueeze(n0, 1)
        n1 = torch.unsqueeze(n1, 1)
        n2 = torch.unsqueeze(n2, 1)
        n3 = torch.unsqueeze(n3, 1)

        #out = self.conv1(F.relu(self.bn1(x)))
        out = self.conv1(x) # modified for PreAct Resnet
        out = self.layer0(out)
        out = self.layer1(out)
        # --- First branch --- #
        lvl1 = F.adaptive_avg_pool2d(out, (1, 1))
        lvl1 = lvl1.view(lvl1.size(0), -1)
        lvl1 = torch.cat([lvl1, poc, qp, n0, n1+n2+n3], dim=1)
        lvl1 = self.branch1(lvl1)
        return lvl1

def Lvl1CtuORI(): # AvgPooling
    return MltCnnLvl1ORI(BasicBlock, [2])

def BigLvl1CtuORI(): # pool + one more layer
    return MltCnnLvl1ORIv2(BasicBlock, [2, 2])

def GapBigLvl1CtuORI(): # GApool + one more layer
    return MltCnnLvl1ORIv3(BasicBlock, [2, 2])