import torch

def calculate_accuracy(output, target):
    with torch.no_grad():
        pred = torch.argmax(output, dim=1)
        assert pred.shape[0] == len(target)
        correct = 0
        correct += torch.sum(pred == target).item()
    return correct / len(target)

def mlt_l1_acc(l1_out, l1_label):
    return calculate_accuracy(l1_out, l1_label)
def mlt_l2_acc(l2_out, l2_label):
    return calculate_accuracy(l2_out, l2_label)
def mlt_l3_acc(l3_out, l3_label):
    return calculate_accuracy(l3_out, l3_label)
def mlt_l4_acc(l4_out, l4_label):
    return calculate_accuracy(l4_out, l4_label)
