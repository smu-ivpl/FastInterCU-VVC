import math
import torch
from torch import autograd as autograd
from torch import nn as nn
from torch.nn import functional as F

from .loss_util import weighted_loss

_reduction_modes = ['none', 'mean', 'sum']

def nll_loss(pred, target):
    return F.nll_loss(pred, target)

def bce_loss(pred, target):
    return F.binary_cross_entropy(pred, target)

def ce_loss(pred, target):
    return F.cross_entropy(pred, target)

def weighted_ce_loss(pred, target, w):
    return F.cross_entropy(pred, target, w)

@weighted_loss
def l1_loss(pred, target):
    return F.l1_loss(pred, target, reduction='none')

@weighted_loss
def mse_loss(pred, target):
    return F.mse_loss(pred, target, reduction='none')

@weighted_loss
def charbonnier_loss(pred, target, eps=1e-12):
    return torch.sqrt((pred - target)**2 + eps)

class LabelSmoothingLoss(nn.Module):
    def __init__(self, classes, smoothing=0.0, dim=-1):
        super(LabelSmoothingLoss, self).__init__()
        self.confidence = 1.0 - smoothing
        self.smoothing = smoothing
        self.cls = classes
        self.dim = dim

    def forward(self, pred, target):
        pred = pred.log_softmax(dim=self.dim)
        with torch.no_grad():
            true_dist = torch.zeros_like(pred)
            true_dist.fill_(self.smoothing / (self.cls - 1))
            true_dist.scatter_(1, target.data.unsqueeze(1), self.confidence)
        return torch.mean(torch.sum(-true_dist * pred, dim=self.dim))

def weighted_mlt_ctu_loss(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter):
    # this weight is for training 128x128 model
    w1 = torch.FloatTensor([0.446294163, 0.553705837]).cuda()
    w2 = torch.FloatTensor([0.446294163, 0.691635945, 0.862069892]).cuda()
    w3 = torch.FloatTensor([0.446294163, 0.691635945, 0.931889401, 0.930180492]).cuda()
    l1_loss = weighted_ce_loss(l1_out, l1_label, w1)
    l2_loss = weighted_ce_loss(l2_out, l2_label, w2)
    l3_loss = weighted_ce_loss(l3_out, l3_label, w3)

    if current_iter <= 150000:
        total_loss = 0.97 * l1_loss + 0.02 * l2_loss + 0.01 * l3_loss
    if current_iter > 150000 and current_iter <= 300000:
        total_loss = 0.1 * l1_loss + 0.7 * l2_loss + 0.2 * l3_loss
    if current_iter > 300000 and current_iter <= 450000:
        total_loss = 0.1 * l1_loss + 0.1 * l2_loss + 0.8 * l3_loss
    if current_iter > 450000:
        total_loss = l3_loss

    return total_loss
def mlt_ctu_loss(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter):
    l1_loss = ce_loss(l1_out, l1_label)
    l2_loss = ce_loss(l2_out, l2_label)
    l3_loss = ce_loss(l3_out, l3_label)

    if current_iter <= 150000:
        total_loss = 0.97 * l1_loss + 0.02 * l2_loss + 0.01 * l3_loss
    if current_iter > 150000 and current_iter <= 300000:
        total_loss = 0.1 * l1_loss + 0.7 * l2_loss + 0.2 * l3_loss
    if current_iter > 300000 and current_iter <= 450000:
        total_loss = 0.1 * l1_loss + 0.1 * l2_loss + 0.8 * l3_loss
    if current_iter > 450000:
        total_loss = l3_loss

    return total_loss

def mlt_ctu_loss_even(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter):
    l1_loss = ce_loss(l1_out, l1_label)
    l2_loss = ce_loss(l2_out, l2_label)
    l3_loss = ce_loss(l3_out, l3_label)
    return 0.33 * l1_loss + 0.33 * l2_loss + 0.34 * l3_loss

def mlt_ctu_loss_adapt(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, l1_acc):
    l1_loss = ce_loss(l1_out, l1_label)
    l2_loss = ce_loss(l2_out, l2_label)
    l3_loss = ce_loss(l3_out, l3_label)

    if l1_acc < 0.9:
        total_loss = 0.97 * l1_loss + 0.02 * l2_loss + 0.01 * l3_loss
    else:
        total_loss = 0.01 * l1_loss + 0.495 * l2_loss + 0.495 * l3_loss
    # if l1_done and l2_acc < 0.8:
    #     total_loss = 0.01 * l1_loss + 0.97 * l2_loss + 0.2 * l3_loss
    #
    # if l2_done and l3_acc < 0.8:
    #     total_loss = 0.1 * l1_loss + 0.1 * l2_loss + 0.8 * l3_loss

    return total_loss



def mlt_ctu_loss2(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter):
    l1_loss = ce_loss(l1_out, l1_label)
    l2_loss = ce_loss(l2_out, l2_label)
    l3_loss = ce_loss(l3_out, l3_label)

    if current_iter <= 300000:
        total_loss = 0.97 * l1_loss + 0.02 * l2_loss + 0.01 * l3_loss
    if current_iter > 300000 and current_iter <= 600000:
        total_loss = 0.1 * l1_loss + 0.7 * l2_loss + 0.2 * l3_loss
    if current_iter > 600000 and current_iter <= 900000:
        total_loss = 0.1 * l1_loss + 0.1 * l2_loss + 0.8 * l3_loss
    if current_iter > 900000:
        total_loss = l3_loss

    return total_loss

def mlt_cu_loss(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, l4_out, l4_label, current_iter):
    l1_loss = ce_loss(l1_out, l1_label)
    l2_loss = ce_loss(l2_out, l2_label)
    l3_loss = ce_loss(l3_out, l3_label)
    l4_loss = ce_loss(l4_out, l4_label)

    if current_iter <= 120000:
        total_loss = 0.97 * l1_loss + 0.01 * l2_loss + 0.01 * l3_loss + 0.01 * l4_loss
    if current_iter > 120000 and current_iter <= 240000:
        total_loss = 0.1 * l1_loss + 0.7 * l2_loss + 0.1 * l3_loss + 0.1 * l4_loss
    if current_iter > 240000 and current_iter <= 360000:
        total_loss = 0.1 * l1_loss + 0.1 * l2_loss + 0.7 * l3_loss + 0.1 * l4_loss
    if current_iter > 360000 and current_iter <= 480000:
        total_loss = 0.1 * l1_loss + 0.1 * l2_loss + 0.2 * l3_loss + 0.6 * l4_loss
    if current_iter > 480000:
        total_loss = l4_loss

    return total_loss
class MltCtuLossAdapt(nn.Module):
    def __init__(self):
        super(MltCtuLossAdapt, self).__init__()
    def forward(self, l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, l1_acc, l2_acc, l3_acc):
        return mlt_ctu_loss_adapt(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, l1_acc, l2_acc, l3_acc)
class MltCtuLoss2(nn.Module):
    def __init__(self):
        super(MltCtuLoss2, self).__init__()
    def forward(self, l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter):
        return mlt_ctu_loss2(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter)

class MltCuLoss(nn.Module):
    def __init__(self):
        super(MltCuLoss, self).__init__()
    def forward(self, l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, l4_out, l4_label, current_iter):
        return mlt_cu_loss(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, l4_out, l4_label, current_iter)

class MltCtuLoss(nn.Module):
    def __init__(self):
        super(MltCtuLoss, self).__init__()
    def forward(self, l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter):
        return mlt_ctu_loss(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter)

class MltCtuLossEven(nn.Module):
    def __init__(self):
        super(MltCtuLossEven, self).__init__()
    def forward(self, l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter):
        return mlt_ctu_loss_even(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter)

class WeightedMltCtuLoss(nn.Module):
    def __init__(self):
        super(WeightedMltCtuLoss, self).__init__()
    def forward(self, l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter):
        return weighted_mlt_ctu_loss(l1_out, l1_label, l2_out, l2_label, l3_out, l3_label, current_iter)
class CELoss(nn.Module):
    def __init__(self):
        super(CELoss, self).__init__()
    def forward(self, pred, target, **kwargs):
        return ce_loss(pred, target)

class L1Loss(nn.Module):
    """L1 (mean absolute error, MAE) loss.

    Args:
        loss_weight (float): Loss weight for L1 loss. Default: 1.0.
        reduction (str): Specifies the reduction to apply to the output.
            Supported choices are 'none' | 'mean' | 'sum'. Default: 'mean'.
    """

    def __init__(self, loss_weight=1.0, reduction='mean'):
        super(L1Loss, self).__init__()
        if reduction not in ['none', 'mean', 'sum']:
            raise ValueError(f'Unsupported reduction mode: {reduction}. '
                             f'Supported ones are: {_reduction_modes}')

        self.loss_weight = loss_weight
        self.reduction = reduction

    def forward(self, pred, target, weight=None, **kwargs):
        """
        Args:
            pred (Tensor): of shape (N, C, H, W). Predicted tensor.
            target (Tensor): of shape (N, C, H, W). Ground truth tensor.
            weight (Tensor, optional): of shape (N, C, H, W). Element-wise
                weights. Default: None.
        """
        return self.loss_weight * l1_loss(
            pred, target, weight, reduction=self.reduction)


class MSELoss(nn.Module):
    """MSE (L2) loss.

    Args:
        loss_weight (float): Loss weight for MSE loss. Default: 1.0.
        reduction (str): Specifies the reduction to apply to the output.
            Supported choices are 'none' | 'mean' | 'sum'. Default: 'mean'.
    """

    def __init__(self, loss_weight=1.0, reduction='mean'):
        super(MSELoss, self).__init__()
        if reduction not in ['none', 'mean', 'sum']:
            raise ValueError(f'Unsupported reduction mode: {reduction}. '
                             f'Supported ones are: {_reduction_modes}')

        self.loss_weight = loss_weight
        self.reduction = reduction

    def forward(self, pred, target, weight=None, **kwargs):
        """
        Args:
            pred (Tensor): of shape (N, C, H, W). Predicted tensor.
            target (Tensor): of shape (N, C, H, W). Ground truth tensor.
            weight (Tensor, optional): of shape (N, C, H, W). Element-wise
                weights. Default: None.
        """
        return self.loss_weight * mse_loss(
            pred, target, weight, reduction=self.reduction)
