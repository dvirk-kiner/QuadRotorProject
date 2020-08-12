from __future__ import print_function
from __future__ import division
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import torchvision
from torchvision import datasets, models, transforms
import matplotlib.pyplot as plt
import time
import os
import copy
import logging

print("PyTorch Version: ", torch.__version__)
print("Torchvision Version: ", torchvision.__version__)


def set_parameter_requires_grad(model):
    for param in model.parameters():
        param.requires_grad = True


def initialize_model(model_name, num_classes, input_size=224, use_pretrained=False):
    # Initialize these variables which will be set in this if statement. Each of these
    #   variables is model specific.
    model_ft = None

    if model_name == "densenet":
        """ Densenet"""
        model_ft = models.densenet201(pretrained=use_pretrained)
        set_parameter_requires_grad(model_ft)
        num_ftrs = model_ft.classifier.in_features
        model_ft.classifier = nn.Linear(num_ftrs, num_classes)
        input_size = input_size

    else:
        print("Invalid model name, exiting...")
        exit()

    return model_ft, input_size


def train_model(model, dataloaders, loss_func, optimizer, logger, num_epochs=25):
    logger.info(
        f'Start training\n---------------------------------------------------')

    since = time.time()

    train_acc_history = []
    train_loss_history = []

    val_acc_history = []
    val_loss_history = []
    # model weights
    best_model_wts = copy.deepcopy(model.state_dict())
    best_acc = 0.0

    for epoch in range(num_epochs):
        logger.info(f'Epoch {epoch}/{num_epochs - 1}')
        logger.info('-' * 10)

        # Each epoch has a training and validation phase
        for phase in ['train', 'val']:
            if phase == 'train':
                model.train()  # Set model to training mode
            else:
                model.eval()   # Set model to evaluate mode

            running_loss = 0.0
            running_corrects = 0

            # Iterate over data.
            for inputs, labels in dataloaders[phase]:
                inputs = inputs.to(device)
                labels = labels.to(device)

                # zero the parameter gradients
                optimizer.zero_grad()

                # forward
                # track history if only in train
                with torch.set_grad_enabled(phase == 'train'):
                    # Get model outputs and calculate loss
                    outputs = model(inputs)
                    loss = loss_func(outputs, labels)

                    _, preds = torch.max(outputs, 1)

                    # backward + optimize only if in training phase
                    if phase == 'train':
                        loss.backward()
                        optimizer.step()

                # statistics
                running_loss += loss.item() * inputs.size(0)
                running_corrects += torch.sum(preds == labels.data)

            epoch_loss = running_loss / len(dataloaders[phase].dataset)
            epoch_acc = running_corrects.double(
            ) / len(dataloaders[phase].dataset)

            logger.info(f'{phase} Loss: {epoch_loss} Acc: {epoch_acc}')

            # deep copy the model
            if phase == 'val' and epoch_acc > best_acc:
                best_acc = epoch_acc
                best_model_wts = copy.deepcopy(model.state_dict())
            if phase == 'val':
                val_acc_history.append(epoch_acc)
                val_loss_history.append(epoch_loss)
            if phase == 'train':
                train_acc_history.append(epoch_acc)
                train_loss_history.append(epoch_loss)

    print()

    time_elapsed = time.time() - since
    logger.info(
        f'Training complete in {time_elapsed // 60}m {time_elapsed % 60}s')
    logger.info(f'Best val Acc: {best_acc}')

    # load best model weights
    model.load_state_dict(best_model_wts)
    return model, {'val_acc': val_acc_history,
                   'val_loss': val_loss_history,
                   'train_acc': train_acc_history,
                   'train_loss': train_loss_history,
                   }


"""    ---   MAIN   ---     """

# model = models.densenet201(pretrained=False)
# in_features = model.classifier.in_features
# model.classifier = nn.Linear(in_features, 3, True)
# print(model)

# Top level data directory. Here we assume the format of the directory conforms
#   to the ImageFolder structure
data_dir = "./data_set"


# Models to choose from [resnet, alexnet, vgg, squeezenet, densenet, inception]
model_name = "densenet"

# Number of classes in the dataset
num_classes = 3

# Batch size for training (change depending on how much memory you have)
batch_size = 10

# Number of epochs to train for
num_epochs = 30

input_size = 252

learning_rate = 0.0001

momentum = 0.9

# TODO: Can be ADAM
optimaizer = "SGD"

# path for saving the model
path_to_model = 'saved_models/' + model_name + '_' + optimaizer + '.pt'

log_file = path_to_model.replace('.pt', '.log')
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
handler = logging.FileHandler(log_file, '+w')
handler.setLevel(logging.INFO)
logger.addHandler(handler)

logger.info(f'Starting CNN training...')
logger.info(f'Initialize Model...')

# Initialize the model for this run
model_ft, input_size = initialize_model(
    model_name, num_classes, input_size=input_size, use_pretrained=False)

# Print the model we just instantiated
print(model_ft)

# Data augmentation and normalization for training
# Just normalization for validation
data_transforms = {
    'train': transforms.Compose([
        # transforms.RandomResizedCrop(input_size),
        # transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]),
    'val': transforms.Compose([
        # transforms.Resize(input_size),
        # transforms.CenterCrop(input_size),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]),
    'test': transforms.Compose([
        # transforms.RandomResizedCrop(input_size),
        # transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]),
}


print("Initializing Datasets and Dataloaders...")

logger.info(f'Create training and validation datasets')

# Create training and validation datasets
image_datasets = {x: datasets.ImageFolder(os.path.join(
    data_dir, x), data_transforms[x]) for x in ['train', 'val']}
# Create training and validation dataloaders
dataloaders_dict = {x: torch.utils.data.DataLoader(
    image_datasets[x], batch_size=batch_size, shuffle=True, num_workers=4) for x in ['train', 'val']}

# Detect if we have a GPU available
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
logger.info(f'Device -> {device}')

# Send the model to GPU
model_ft = model_ft.to(device)

# Gather the parameters to be optimized/updated in this run.
params_to_update = model_ft.parameters()


# Observe that all parameters are being optimized
optimizer_ft = optim.SGD(params_to_update, lr=learning_rate, momentum=momentum)
logger.info(f'Optimizer -> {optimaizer}')
logger.info(f'Learning Rate -> {learning_rate}')
logger.info(f'Momentum -> {momentum}')


# Setup the loss fxn
loss_func = nn.CrossEntropyLoss()
logger.info(f'Loss Func -> CrossEntropyLoss')

# Train and evaluate
model_ft, hist = train_model(model_ft, dataloaders_dict, loss_func, optimizer_ft,
                             logger, num_epochs=num_epochs)
logger.info(f'RESULT:')
logger.info(f'val_acc = {hist["val_acc"]}')
logger.info(f'val_loss = {hist["val_loss"]}')
logger.info(f'train_acc = {hist["train_acc"]}')
logger.info(f'train_loss = {hist["train_loss"]}')

torch.save(model_ft, path_to_model)
