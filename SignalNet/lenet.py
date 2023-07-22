import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
from torchvision.datasets import MNIST
from torch.utils.data import DataLoader
import numpy as np
import os
import struct
from tqdm import tqdm
from collections import OrderedDict


def float_to_hex(f):
    return hex(struct.unpack("<I", struct.pack("<f", f))[0])


class LeNet5(nn.Module):
    def __init__(self, in_channels, out_classes) -> None:
        super().__init__()
        self.in_channels = in_channels
        self.out_classes = out_classes

        self.layers = nn.Sequential(
            OrderedDict(
                [
                    ("conv1", nn.Conv2d(self.in_channels, 6, 5)),
                    ("relu1", nn.ReLU()),
                    ("pool1", nn.MaxPool2d(2)),
                    ("conv2", nn.Conv2d(6, 16, 5)),
                    ("relu2", nn.ReLU()),
                    ("pool2", nn.MaxPool2d(2)),
                    ("flatten", nn.Flatten()),
                    ("fc1", nn.Linear(16 * 4 * 4, 120)),
                    ("relu3", nn.ReLU()),
                    ("fc2", nn.Linear(120, 84)),
                    ("relu4", nn.ReLU()),
                    ("fc3", nn.Linear(84, self.out_classes)),
                    ("softmax", nn.Softmax(dim=1)),
                ]
            )
        )

    def forward(self, x):
        return self.layers(x)

    def export_c(self, header, source):
        with open(header, "w") as fh:
            fh.write("#pragma once\n")
            fh.write("#include <stdint.h>\n")

            fh.write(f"#define NN_Lenet_Input_Channels {self.in_channels}\n")
            fh.write(f"#define NN_Lenet_Output_Classes {self.out_classes}\n")

            for name, param in self.named_parameters():
                fh.write(f"// {name}, {param.shape}\n")
                fh.write(
                    f"extern const uint32_t NN_Lenet_Param_{name.replace('.', '_')}_Size;\n"
                )
                fh.write(f"// {name}, {param.shape}\n")
                fh.write(
                    f"extern const uint32_t NN_Lenet_Param_{name.replace('.', '_')}_Data[];\n"
                )

        with open(source, "w") as fs:
            fs.write(f'#include "{os.path.basename(header)}"\n')
            for name, param in self.named_parameters():
                fs.write(
                    f"const uint32_t NN_Lenet_Param_{name.replace('.', '_')}_Size = {param.numel()};\n"
                )
                fs.write(
                    f"const uint32_t NN_Lenet_Param_{name.replace('.', '_')}_Data[] = {{"
                )
                fs.write(", ".join([float_to_hex(x.item()) for x in param.flatten()]))
                fs.write("};\n")


# Define data transformations and download MNIST dataset
transform = transforms.Compose(
    [
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,)),  # Normalize to range [-1, 1]
    ]
)

train_dataset = MNIST(root="./data", train=True, transform=transform, download=True)
test_dataset = MNIST(root="./data", train=False, transform=transform, download=True)

# Set hyperparameters
batch_size = 64
learning_rate = 0.001
num_epochs = 10

# Create data loaders
train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

# Initialize the LeNet-5 model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = LeNet5(1, 10).to(device)

# Define loss function and optimizer
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# Training loop
for epoch in tqdm(range(num_epochs)):
    model.train()
    train_loss = 0.0
    for images, labels in train_loader:
        images, labels = images.to(device), labels.to(device)

        # Zero the gradients
        optimizer.zero_grad()

        # Forward pass
        outputs = model(images)
        loss = criterion(outputs, labels)

        # Backward pass and optimization
        loss.backward()
        optimizer.step()

        train_loss += loss.item()

    avg_train_loss = train_loss / len(train_loader)
    print(f"Epoch [{epoch + 1}/{num_epochs}], Avg. Train Loss: {avg_train_loss:.4f}")

# Evaluation
model.eval()
correct = 0
total = 0

with torch.no_grad():
    for images, labels in test_loader:
        images, labels = images.to(device), labels.to(device)
        outputs = model(images)
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

accuracy = 100 * correct / total
print(f"Test Accuracy: {accuracy:.2f}%")

# Save the model
torch.save(model.state_dict(), "lenet5.pth")

# Export the model to C
model.export_c("Inc/lenet_params.h", "Src/lenet_params.c")
