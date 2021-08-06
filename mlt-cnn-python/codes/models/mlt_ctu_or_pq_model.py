import importlib
import torch
from collections import OrderedDict
from copy import deepcopy
from os import path as osp
from tqdm import tqdm

from .archs import define_network
from .mlt_base_model import MltBaseModel
from utils import get_root_logger, imwrite, tensor2img

loss_module = importlib.import_module('models.losses')
metric_module = importlib.import_module('metrics')

class MltCtuORPQModel(MltBaseModel):
    """MLT-CNN for 128x128."""

    def __init__(self, opt):
        super(MltCtuORPQModel, self).__init__(opt)

        # define network
        self.net = define_network(deepcopy(opt['network']))
        self.net = self.model_to_device(self.net)
        self.print_network(self.net)

        # load pretrained models
        load_path = self.opt['path'].get('pretrain_network', None)
        if load_path is not None:
            self.load_network(self.net, load_path,
                              self.opt['path'].get('strict_load', True))

        if self.is_train:
            self.init_training_settings()
            self.log_dict = OrderedDict()

    def init_training_settings(self):
        self.net.train()
        train_opt = self.opt['train']

        # define losses
        if train_opt.get('criterion'):
            loss_type = train_opt['criterion'].pop('type')
            loss_type = getattr(loss_module, loss_type)
            self.criterion = loss_type(**train_opt['criterion']).to(
                self.device)
        else:
            self.criterion = None

        if self.criterion is None:
            raise ValueError('Loss function is None.')

        # set up optimizers and schedulers
        self.setup_optimizers()
        self.setup_schedulers()

    def setup_optimizers(self):
        train_opt = self.opt['train']
        optim_params = []
        for k, v in self.net.named_parameters():
            if v.requires_grad:
                optim_params.append(v)
            else:
                logger = get_root_logger()
                logger.warning(f'Params {k} will not be optimized.')

        optim_type = train_opt['optim'].pop('type')
        if optim_type == 'Adam':
            self.optimizer = torch.optim.Adam(optim_params,
                                                **train_opt['optim'])
        elif optim_type == 'SGD':
            self.optimizer = torch.optim.SGD(optim_params, **train_opt['optim'])
        else:
            raise NotImplementedError(
                f'optimizer {optim_type} is not supperted yet.')
        self.optimizers.append(self.optimizer)

    def feed_data(self, data):
        self.org = data['org']
        self.resi = data['resi']
        # [batch, C, H, W]
        concat2img = torch.cat((self.org, self.resi), 1)
        self.concat_img = concat2img.to(self.device)

        self.l1_gt, self.l2_gt, self.l3_gt\
            = data['l1_gt'].to(self.device), data['l2_gt'].to(self.device), data['l3_gt'].to(self.device)
        self.poc = data['poc'].to(self.device)
        self.qp = data['qp'].to(self.device)

    def optimize_parameters(self, current_iter):
        # optimize network

        self.optimizer.zero_grad()
        self.l1_out, self.l2_out, self.l3_out = self.net(self.concat_img, self.poc, self.qp)

        loss = self.criterion(self.l1_out, self.l1_gt, self.l2_out, self.l2_gt,
                              self.l3_out, self.l3_gt, current_iter)
        loss.backward()
        self.optimizer.step()

        # set log
        self.log_dict['loss'] = loss.item()

    def test(self):
        self.net.eval()
        with torch.no_grad():
            self.l1_out, self.l2_out, self.l3_out = self.net(self.concat_img, self.poc, self.qp)
        self.net.train()

    def dist_validation(self, dataloader, current_iter, tb_logger):
        logger = get_root_logger()
        logger.info('Only support single GPU validation.')
        self.nondist_validation(dataloader, current_iter, tb_logger)

    def nondist_validation(self, dataloader, current_iter, tb_logger, save_img=None):
        dataset_name = dataloader.dataset.opt['name']
        with_metrics = self.opt['val'].get('metrics') is not None
        if with_metrics:
            self.metric_results = {
                metric: 0
                for metric in self.opt['val']['metrics'].keys()
            }
        pbar = tqdm(total=len(dataloader), unit='image')

        for idx, val_data in enumerate(dataloader):
            self.feed_data(val_data)
            self.test()

            if with_metrics:
                # calculate metrics
                opt_metric = deepcopy(self.opt['val']['metrics'])
                for name, opt_ in opt_metric.items():
                    if name == 'l1':
                        output = self.l1_out
                        gt = self.l1_gt
                    elif name == 'l2':
                        output = self.l2_out
                        gt = self.l2_gt
                    elif name == 'l3':
                        output = self.l3_out
                        gt = self.l3_gt
                    metric_type = opt_.pop('type')
                    self.metric_results[name] += getattr(
                        metric_module, metric_type)(output, gt)#, **opt_)

            # tentative for out of GPU memory
            del self.org
            del self.resi
            del self.concat_img
            del self.l1_out
            del self.l2_out
            del self.l3_out
            del self.poc
            del self.qp
            del self.l1_gt
            del self.l2_gt
            del self.l3_gt
            torch.cuda.empty_cache()

            pbar.update(1)
            pbar.set_description(f'Test {idx}')
        pbar.close()

        if with_metrics:
            for metric in self.metric_results.keys():
                self.metric_results[metric] /= (idx + 1)

            self._log_validation_metric_values(current_iter, dataset_name,
                                               tb_logger)

    def _log_validation_metric_values(self, current_iter, dataset_name,
                                      tb_logger):
        log_str = f'Validation {dataset_name}\n'
        for metric, value in self.metric_results.items():
            log_str += f'\t # {metric}: {value:.4f}\n'
        logger = get_root_logger()
        logger.info(log_str)
        if tb_logger:
            for metric, value in self.metric_results.items():
                tb_logger.add_scalar(f'metrics/{metric}', value, current_iter)

    def get_current_visuals(self):
        out_dict = OrderedDict()
        out_dict['org'] = self.org.detach().cpu()
        out_dict['result'] = self.output.detach().cpu()
        if hasattr(self, 'gt'):
            out_dict['gt'] = self.gt.detach().cpu()
        return out_dict

    def save(self, epoch, current_iter):
        self.save_network(self.net, 'net', current_iter)
        self.save_training_state(epoch, current_iter)
