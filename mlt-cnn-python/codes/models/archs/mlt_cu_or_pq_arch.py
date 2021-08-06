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

class MltCnnL4ORPQv4(nn.Module): # it has GA pooling layer and one more resblock (Big)
    # 64x64: 2, 3, 4, 6
    def __init__(self, block, num_blocks, num_cls_l1=2, num_cls_l2=3, num_cls_l3=4, num_cls_l4=6):
        super(MltCnnL4ORPQv4, self).__init__()
        self.in_planes = 32

        self.conv1 = nn.Conv2d(2, 32, kernel_size=3,
                               stride=1, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(32)
        self.layer0 = self._make_layer(block, 32, num_blocks[0], stride=2)
        self.layer1 = self._make_layer(block, 64, num_blocks[1], stride=2)
        self.branch1 = nn.Linear(64+2, num_cls_l1)

        self.layer2 = self._make_layer(block, 96, num_blocks[2], stride=2)
        self.branch2 = nn.Linear(96+2, num_cls_l2)

        self.layer3 = self._make_layer(block, 128, num_blocks[3], stride=2)
        self.branch3 = nn.Linear(128+2, num_cls_l3)

        self.layer4 = self._make_layer(block, 256, num_blocks[4], stride=2)
        self.branch4 = nn.Linear(256+2, num_cls_l4)

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

    def forward(self, x, poc, qp):
        poc = torch.unsqueeze(poc, 1)
        qp = torch.unsqueeze(qp, 1)

        #out = self.conv1(F.relu(self.bn1(x)))
        out = self.conv1(x) # modified for PreAct Resnet
        out = self.layer0(out)
        out = self.layer1(out)
        # --- First branch --- #
        lvl1 = F.adaptive_avg_pool2d(out, (1, 1))
        lvl1 = lvl1.view(lvl1.size(0), -1)
        lvl1 = torch.cat([lvl1, poc, qp], dim=1)
        lvl1 = self.branch1(lvl1)
        out = self.layer2(out)
        # --- Second branch --- #
        lvl2 = F.adaptive_avg_pool2d(out, (1, 1))
        lvl2 = lvl2.view(lvl2.size(0), -1)
        lvl2 = torch.cat([lvl2, poc, qp], dim=1)
        lvl2 = self.branch2(lvl2)
        out = self.layer3(out)
        # --- Third branch --- #
        lvl3 = F.adaptive_avg_pool2d(out, (1, 1))
        lvl3 = lvl3.view(lvl3.size(0), -1)
        lvl3 = torch.cat([lvl3, poc, qp], dim=1)
        lvl3 = self.branch3(lvl3)
        out = self.layer4(out)
        # --- Fourth branch --- #
        lvl4 = F.adaptive_avg_pool2d(out, (1, 1))
        lvl4 = lvl4.view(lvl4.size(0), -1)
        lvl4 = torch.cat([lvl4, poc, qp], dim=1)
        lvl4 = self.branch4(lvl4)

        return lvl1, lvl2, lvl3, lvl4

def GapBigMltCuORPQ(): # Global Avg Pooling
    return MltCnnL4ORPQv4(BasicBlock, [2, 2, 2, 2, 2])