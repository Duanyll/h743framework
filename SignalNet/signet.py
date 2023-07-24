import torch
import torch.nn as nn
import torch.nn.functional as F

from collections import OrderedDict
import struct
import os.path
import tqdm

def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0])

# 1d convolutional neural network for signal classification
class SignalNet(nn.Module):
    def __init__(self, input_channels, output_classes):
        super(SignalNet, self).__init__()

        self.input_samples = 143
        self.input_channels = input_channels
        self.output_classes = output_classes

        self.layers = nn.Sequential(
            OrderedDict(
                [
                    # cin x 143
                    ("conv1", nn.Conv1d(self.input_channels, 8, 5, stride=2)),
                    # 8 x 70
                    ("relu1", nn.ReLU()),
                    ("bn1", nn.BatchNorm1d(8)),
                    ("pool1", nn.AvgPool1d(2)),
                    # 8 x 35
                    ("conv2", nn.Conv1d(8, 16, 5, stride=2)),
                    # 16 x 16
                    ("relu2", nn.ReLU()),
                    ("bn2", nn.BatchNorm1d(16)),
                    ("pool2", nn.AvgPool1d(2)),
                    # 16 x 8
                    ("flatten", nn.Flatten()),
                    ("fc1", nn.Linear(16 * 8, 16)),
                    ("relu4", nn.ReLU()),
                    ("fc2", nn.Linear(16, self.output_classes)),
                    ("softmax", nn.Softmax(dim=1))
                ]
            )
        )

    def forward(self, x):
        return self.layers(x)
    
    def fuse_bn(self):
        for m in self.modules():
            if type(m) == nn.BatchNorm1d:
                m.weight.data = m.weight.data / torch.sqrt(m.running_var + 1e-5)
                m.bias.data = m.bias.data - m.running_mean * m.weight.data
                m.running_mean = torch.zeros_like(m.running_mean)
                m.running_var = torch.ones_like(m.running_var)

    def export_c(self, header, source):
        with open(header, "w") as fh:
            fh.write("#pragma once\n")
            fh.write("#include <stdint.h>\n")

            fh.write(f"#define NN_Signet_Input_Samples {self.input_samples}\n")
            fh.write(f"#define NN_Signet_Input_Channels {self.input_channels}\n")
            fh.write(f"#define NN_Signet_Output_Classes {self.output_classes}\n")

            for name, param in self.named_parameters():
                fh.write(f"// {name}, {param.shape}\n")
                fh.write(f"extern const uint32_t NN_Signet_Param_{name.replace('.', '_')}_Size;\n")
                fh.write(f"// {name}, {param.shape}\n")
                fh.write(f"extern const uint32_t NN_Signet_Param_{name.replace('.', '_')}_Data[];\n")

        with open(source, "w") as fs:
            fs.write(f"#include \"{os.path.basename(header)}\"\n")
            for name, param in self.named_parameters():
                fs.write(f"const uint32_t NN_Signet_Param_{name.replace('.', '_')}_Size = {param.numel()};\n")
                fs.write(f"const uint32_t NN_Signet_Param_{name.replace('.', '_')}_Data[] = {{")
                fs.write(", ".join([float_to_hex(x.item()) for x in param.flatten()]))
                fs.write("};\n")
                
def train(model, dataset):
    # optimizer = torch.optim.SGD(model.parameters(), lr=0.01, momentum=0.9)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    loss_fn = torch.nn.CrossEntropyLoss()
    train_loader = torch.utils.data.DataLoader(dataset, batch_size=32, shuffle=True)
    for epoch in tqdm(range(10)):
        correct_count = 0
        for i, (waveform, label) in enumerate(train_loader):
            pred = model(waveform)
            loss = loss_fn(pred, label)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            pred = torch.argmax(pred, dim=1)
            correct_count += torch.sum(pred == label)
        print(f"Epoch {epoch}, accuracy {correct_count / len(dataset)}")
        torch.save(model.state_dict(), f"model_{epoch}.pt")

def confusion_matrix(model, test_loader):
    model.eval()
    confusion_matrix = torch.zeros(3, 3)
    correct_count = 0
    for i, (waveform, label) in enumerate(test_loader):
        pred = model(waveform)
        pred = torch.argmax(pred, dim=1)
        for j in range(len(label)):
            confusion_matrix[label[j], pred[j]] += 1
        correct_count += torch.sum(pred == label)
    return confusion_matrix, correct_count / len(test_loader.dataset)


if __name__ == "__main__":
    model = SignalNet(2, 3)
    waveform = torch.load("waveform.pt")
    waveform = torch.cat((waveform, torch.zeros((waveform.shape[0], 2, 5))), dim=2)
    label = torch.load("label.pt")
    dataset = torch.utils.data.TensorDataset(waveform, label)
    train(model, dataset)
    test_loader = torch.utils.data.DataLoader(dataset, batch_size=32, shuffle=False)
    model.fuse_bn()
    print(confusion_matrix(model, test_loader))
    model.export_c("Inc/signet_params.h", "Src/signet_params.c")
