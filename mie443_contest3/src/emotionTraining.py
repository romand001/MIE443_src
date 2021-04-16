#!/usr/bin/env python3

!pip install livelossplot==0.5.1 --quiet

import torch
import torch.nn as nn
from torchvision import transforms
import argparse
from tqdm import tqdm
import matplotlib.pyplot as plt
from IPython.display import clear_output
from livelossplot import PlotLosses
import numpy as np
from PIL import Image


class Swish(nn.Module):
    def forward(self, input_tensor):
        return input_tensor * torch.sigmoid(input_tensor)

class MinusPointFive(object):

    def __call__(self, tensor):
        return torch.sub(tensor, 0.5077)

    def __repr__(self):
        return self.__class__.__name__ + '()'

class EmotionClassificationNet8(nn.Module):

    def __init__(self):
        super(EmotionClassificationNet8, self).__init__()
        self.cnn = nn.Sequential(
            nn.Conv2d(1, 32, 3, padding=1),
            nn.LeakyReLU(0.2),
            nn.Conv2d(32, 32, 3, padding=1),
            nn.LeakyReLU(0.2),
            nn.MaxPool2d(2, 2),

            nn.Conv2d(32, 64, 3, padding=1),
            nn.LeakyReLU(0.2),
            nn.Conv2d(64, 64, 3, padding=1),
            nn.LeakyReLU(0.2),
            nn.MaxPool2d(2, 2),

            nn.Conv2d(64, 96, 3, padding=1, dilation=2),
            nn.LeakyReLU(0.2),
            nn.Dropout(0.25),
            nn.Conv2d(96, 96, 3, padding=1),
            nn.LeakyReLU(0.2),
            nn.MaxPool2d(2, 2),

            nn.Conv2d(96, 128, 3, padding=1, dilation=2),
            nn.LeakyReLU(0.2),
            nn.Dropout(0.25),
            nn.Conv2d(128, 128, 3, padding=1),
            nn.LeakyReLU(0.2),
            nn.MaxPool2d(2, 2),


        )
        self.nn = nn.Sequential(
            nn.Linear(128, 64),
            Swish(),
            nn.Dropout(0.4),
            nn.Linear(64, 7)
        )

    def forward(self, x, test_mode=False):

        batch_size = x.shape[0]
        feats = self.cnn(x)

        out = self.nn(feats.view(batch_size, -1))
        #
        # If we are testing then return prediction index.
        if test_mode:
            _, out = torch.max(out, 1)
        return out

class CustomTensorDataset(torch.utils.data.Dataset):
    """TensorDataset with support for transforms.
    """
    def __init__(self, tensors, transform=None):
        assert all(tensors[0].size(0) == tensor.size(0) for tensor in tensors)
        self.tensors = tensors
        self.transform = transform

    def __getitem__(self, index):
        x = self.tensors[0][index]

        if self.transform:
            x = self.transform(x)

        y = self.tensors[1][index]

        # plt.imshow(Image.fromarray( np.squeeze(np.array(((x+0.5)*255), dtype='uint8')) ))
        # plt.show()
        
        return x, y

    def __len__(self):
        return self.tensors[0].size(0)

def getDataloader(batch_size, val_split = 0.2, augment=False):

    # imgs, labels = torch.load(path_to_data + 'train_split.pth')
    imgs, labels = torch.load('train_split.pth')

    probs = torch.ones(imgs.shape[0]) * val_split
    mask = torch.bernoulli(probs).bool()

    train = (imgs[~mask], labels[~mask])
    val = (imgs[mask], labels[mask])

    weights_label = train[1].unique(return_counts=True, sorted=True)[1].float().reciprocal()
    weights = torch.zeros_like(train[1], dtype=torch.float)
    for idx, label in enumerate(train[1]):
        weights[idx] = weights_label[label]
    sampler = torch.utils.data.sampler.WeightedRandomSampler(weights, len(weights))


    train_transform = transforms.Compose([
        transforms.Normalize((-0.5077,), (1.0,)),
        transforms.ToPILImage(),
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.RandomRotation(15, fill=(0,)),
        transforms.RandomPerspective(0.25, p=0.5),
        #transforms.CenterCrop(40),
        transforms.ToTensor(),
        MinusPointFive()
        #transforms.Normalize((0.5,), (1.0,))
    ])

    val_transform = transforms.Compose([
        #transforms.ToPILImage(),
        # transforms.CenterCrop(40),
        #transforms.ToTensor()
        #transforms.Normalize((0.5,), (1.0,))
    ])

    if augment:
        train_dataset = CustomTensorDataset(tensors=(train[0], train[1]), 
                                            transform=train_transform)
        val_dataset = CustomTensorDataset(tensors=(val[0], val[1]), 
                                            transform=val_transform)
    else:
        train_dataset = CustomTensorDataset(tensors=(train[0], train[1]), 
                                            transform=None)
        val_dataset = CustomTensorDataset(tensors=(val[0], val[1]), 
                                            transform=None)

    
    train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size,
                                            num_workers=2, sampler=sampler)
    val_loader = torch.utils.data.DataLoader(val_dataset, batch_size=batch_size,
                                            shuffle=False, num_workers=2)
    return train_loader, val_loader

def train_loop(mdl, loss_fn, optim, dl, device):
    n_batch_loss = 50
    running_loss = 0
    for nex, ex in enumerate(dl):
        ims, labels, = ex
        ims = ims.to(device)
        labels = labels.to(device)
        #
        # Optimization.
        optim.zero_grad()
        outs = mdl(ims)
        loss = loss_fn(outs, labels)
        loss.backward(loss)
        optim.step()
        #
        # Statistics
        running_loss +=  loss.item()
        nex += 1
        if nex % n_batch_loss == 0:
            status = 'L: %.4f '%(loss / n_batch_loss)
            running_loss = 0

    return mdl

def calc_acc(mdl, dl, dl_type, device):
    with torch.no_grad():
        total = 0
        ncorrect = 0
        for nex, ex in enumerate(dl):
            ims, labels, = ex
            ims = ims.to(device)
            labels = labels.to(device)
            predicted = mdl(ims, True)
            total += labels.size(0)
            ncorrect += (predicted == labels).sum().item()
            status = '%s ACC: %.4f '%(dl_type, float(ncorrect) / total)

    #print(total)
    return float(ncorrect) / total

def weight_reset(m):
    if isinstance(m, nn.Conv2d) or isinstance(m, nn.Linear):
        m.reset_parameters()

def splitKFolds(dataset, k_folds):
    indices = np.arange(len(dataset), dtype='int')
    np.random.shuffle(indices)

    k_indices = np.array_split(indices, k_folds)

    folds = []
    
    for i in range(k_folds):
        fold = [np.array([], dtype='int'), np.array([], dtype='int')] # test set index, train_indices, test_indices
        for j in range(k_folds):
            # this is the test set
            if i == j:
                fold[1] = k_indices[j] # set test indices to jth fold
            # this is one of the train sets
            else:
                fold[0] = np.concatenate((fold[0], k_indices[j])) # add jth fold to train indices

        folds.append(fold)
    
    return folds

def validateKFolds(model, k_folds=5, epochs=20, batch_size=128, augment = False):

    try:
        # data = torch.load(path_to_data + 'train_split.pth')
        data = torch.load('train_split.pth')
        dataset = torch.utils.data.TensorDataset(*data)
        print(len(dataset))
    except:
        print('could not load data!')
        return -1

    weights_label = data[1].unique(return_counts=True, sorted=True)[1].float().reciprocal()
    weights = torch.zeros_like(data[1], dtype=torch.float)
    for idx, label in enumerate(data[1]):
        weights[idx] = weights_label[label]
    

    ce_loss = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters())

    device = torch.device('cuda:0')
    model = model.to(device)

    torch.manual_seed(42)

    results = []

    for fold, (train_ids, test_ids) in enumerate(splitKFolds(dataset, k_folds)):

        print(fold, train_ids, test_ids)

        train_weights = weights.clone()[train_ids]

        train_sampler = torch.utils.data.sampler.WeightedRandomSampler(
            train_weights, train_weights.shape[0])

        train_transform = transforms.Compose([
            transforms.Normalize((-0.5077,), (1.0,)),
            transforms.ToPILImage(),
            transforms.RandomHorizontalFlip(p=0.5),
            transforms.RandomRotation(15, fill=(0,)),
            transforms.RandomPerspective(0.25, p=0.5),
            #transforms.CenterCrop(40),
            transforms.ToTensor(),
            MinusPointFive()
            #transforms.Normalize((0.5,), (1.0,))
        ])

        test_transform = transforms.Compose([
            # transforms.ToPILImage(),
            # transforms.CenterCrop(40),
            # transforms.ToTensor()
            #transforms.Normalize((0.5,), (0.5,))
        ])

        train_tensors = (data[0][train_ids], data[1][train_ids])
        test_tensors = (data[0][test_ids], data[1][test_ids])

        print(data[0][train_ids].shape, data[1][train_ids].shape)

        if augment:
            train_dataset = CustomTensorDataset(tensors=train_tensors, 
                                                transform=train_transform)
            test_dataset = CustomTensorDataset(tensors=test_tensors, 
                                                transform=test_transform)
        else:
            train_dataset = CustomTensorDataset(tensors=train_tensors, 
                                                transform=None)
            test_dataset = CustomTensorDataset(tensors=test_tensors, 
                                                transform=None)

        train_dl = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size,
                                        sampler=train_sampler, num_workers=4)
        test_dl = torch.utils.data.DataLoader(test_dataset, batch_size=batch_size,
                                        num_workers=4)
    
        plotlosses = PlotLosses(groups={f'Fold {fold+1} Accuracy': ['train', 'test']})

        # reset weights
        model.apply(weight_reset)

        for epoch in range(epochs):
            
            model.train(True)
            model = train_loop(model, ce_loss, optimizer, train_dl, device)
            model.train(False)

            train_acc = calc_acc(model, train_dl, 'train', device)
            test_acc = calc_acc(model, test_dl, 'test', device)

            plotlosses.update({
                'train': train_acc,
                'test': test_acc
            })
            plotlosses.send()
        
        results.append(test_acc)

    return results

def train(model, augment=False, full_data = False, batch_size = 128, learning_rate = 0.005, epochs = 50):

    if full_data:
        train_dl, val_dl = getDataloader(batch_size, val_split=0, augment=augment)
    else:
        train_dl, val_dl = getDataloader(batch_size, val_split=0.2, augment=augment)

    for i, (img, label) in enumerate(train_dl):
        print("train", torch.min(img), torch.max(img), sep=', ')
        break
    for i, (img, label) in enumerate(val_dl):
        print("val", torch.min(img), torch.max(img), sep=', ')
        break

    ce_loss = nn.CrossEntropyLoss()

    optimizer = torch.optim.Adam(model.parameters())

    device = torch.device('cuda:0')
    model.to(device)

    plotlosses = PlotLosses(groups={'Accuracy': ['train', 'val']})

    best_val = -float('inf')
    for epoch in range(epochs):

        model.train(True)
        model = train_loop(model, ce_loss, optimizer, train_dl, device)

        model.train(False)
        train_acc = calc_acc(model, train_dl, 'train', device)

        val_acc = calc_acc(model, val_dl, 'val', device)

        plotlosses.update({
            'train': train_acc,
            'val': val_acc
        })
        plotlosses.send()

        # Early stopping.
        if val_acc > best_val:
            best_val = val_acc
            # torch.save(model.state_dict(), path_to_data + 'mdl_best.pth')
            torch.save(model.state_dict(), 'mdl_best.pth')

if __name__ == "__main__":
    model = EmotionClassificationNet8()
    train(model, epochs=100, augment=True)


