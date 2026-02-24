import os
import random
import math
from PIL import Image
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
import torchvision.transforms.functional as TF
from torchmetrics.image import StructuralSimilarityIndexMeasure
import torchvision.models as models

NOAA_DIR = "./noaa" # outdated reference path, contained my raw images
AA_DIR = "./aa" # outdated reference path, contained my fully supersampled images (aka the reference)
SECTION_SIZE = 512  # train on full image
BATCH_SIZE = 1
LEARNING_RATE = 1e-4
NUM_EPOCHS = 40


class PerceptualLoss(nn.Module):
    def __init__(self):
        super().__init__()
        vgg = models.vgg16(weights=models.VGG16_Weights.DEFAULT).features
        self.blocks = nn.ModuleList(
            [
                vgg[:4],  # relu1_2
                vgg[4:9],  # relu2_2
                vgg[9:16],  # relu3_3
            ]
        )
        for p in self.parameters():
            p.requires_grad = False

    def forward(self, pred, target):
        loss = 0.0
        x, y = pred, target
        for block in self.blocks:
            x, y = block(x), block(y)
            loss += nn.functional.l1_loss(x, y)
        return loss


def edge_loss(pred, target):
    sobel_x = torch.tensor([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=torch.float32)
    sobel_y = sobel_x.T
    kernel_x = sobel_x.view(1, 1, 3, 3).repeat(3, 1, 1, 1).to(pred.device)
    kernel_y = sobel_y.view(1, 1, 3, 3).repeat(3, 1, 1, 1).to(pred.device)

    pred_ex = nn.functional.conv2d(pred, kernel_x, padding=1, groups=3)
    pred_ey = nn.functional.conv2d(pred, kernel_y, padding=1, groups=3)
    tgt_ex = nn.functional.conv2d(target, kernel_x, padding=1, groups=3)
    tgt_ey = nn.functional.conv2d(target, kernel_y, padding=1, groups=3)

    return nn.functional.l1_loss(pred_ex, tgt_ex) + nn.functional.l1_loss(
        pred_ey, tgt_ey
    )


class RayTraceDataset(Dataset):
    def __init__(self, noaa_dir, aa_dir):
        self.noaa_dir = noaa_dir
        self.aa_dir = aa_dir
        self.filenames = [
            f
            for f in os.listdir(noaa_dir)
            if f.endswith(".png") and os.path.exists(os.path.join(aa_dir, f))
        ]
        print(f"found {len(self.filenames)} valid image pairs")

    def __len__(self):
        return len(self.filenames)

    def __getitem__(self, idx):
        filename = self.filenames[idx]
        input_img = Image.open(os.path.join(self.noaa_dir, filename)).convert("RGB")
        target_img = Image.open(os.path.join(self.aa_dir, filename)).convert("RGB")

        # w, h = input_img.size
        # if w < SECTION_SIZE or h < SECTION_SIZE:
        #     input_img = TF.resize(
        #         input_img, (max(w, SECTION_SIZE), max(h, SECTION_SIZE))
        #     )
        #     target_img = TF.resize(
        #         target_img, (max(w, SECTION_SIZE), max(h, SECTION_SIZE))
        #     )
        #     w, h = input_img.size

        # best_patch, best_target, best_var = None, None, -1
        # for _ in range(4):
        #     i = random.randint(0, h - SECTION_SIZE)
        #     j = random.randint(0, w - SECTION_SIZE)
        #     p = TF.crop(input_img, i, j, SECTION_SIZE, SECTION_SIZE)
        #     t = TF.crop(target_img, i, j, SECTION_SIZE, SECTION_SIZE)
        #     var = TF.to_tensor(p).var().item()
        #     if var > best_var:
        #         best_patch, best_target, best_var = p, t, var

        # return TF.to_tensor(best_patch), TF.to_tensor(best_target)
        return TF.to_tensor(input_img), TF.to_tensor(target_img)


class DoubleConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.double_conv = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1),
            nn.InstanceNorm2d(out_channels), 
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1),
            nn.InstanceNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.double_conv(x)


class DeepAA(nn.Module):
    def __init__(self):
        super(DeepAA, self).__init__()

        self.inc = DoubleConv(3, 64)
        self.down1 = nn.Sequential(nn.MaxPool2d(2), DoubleConv(64, 128))
        self.down2 = nn.Sequential(nn.MaxPool2d(2), DoubleConv(128, 256))
        self.down3 = nn.Sequential(nn.MaxPool2d(2), DoubleConv(256, 512))

        self.up1 = nn.ConvTranspose2d(512, 256, kernel_size=2, stride=2)
        self.conv_up1 = DoubleConv(512, 256)

        self.up2 = nn.ConvTranspose2d(256, 128, kernel_size=2, stride=2)
        self.conv_up2 = DoubleConv(256, 128)

        self.up3 = nn.ConvTranspose2d(128, 64, kernel_size=2, stride=2)
        self.conv_up3 = DoubleConv(128, 64)

        self.outc = nn.Conv2d(64, 3, kernel_size=1)

    def forward(self, x):
        inp = x
        x1 = self.inc(x)
        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x4 = self.down3(x3)

        x = self.up1(x4)
        x = self.conv_up1(torch.cat([x3, x], dim=1))
        x = self.up2(x)
        x = self.conv_up2(torch.cat([x2, x], dim=1))
        x = self.up3(x)
        x = self.conv_up3(torch.cat([x1, x], dim=1))

        # residual = torch.tanh(self.outc(x)) * 0.1
        residual = self.outc(x)  # test no cap
        return torch.clamp(inp + residual, 0.0, 1.0)


def calculate_psnr(mse):
    if mse == 0:
        return 100
    return 10 * math.log10(1.0 / mse)


def main():
    # MPS for Apple Silicon, CUDA for Nvidia, fallback to CPU
    if torch.backends.mps.is_available():
        device = torch.device("mps")
    elif torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")
    print(f"training on: {device}")

    ssim_metric = StructuralSimilarityIndexMeasure(data_range=1.0).to(device)
    dataset = RayTraceDataset(NOAA_DIR, AA_DIR)

    total_count = len(dataset)
    train_count = int(0.8 * total_count)
    val_count = int(0.1 * total_count)
    test_count = total_count - train_count - val_count
    train_data, val_data, test_data = random_split(
        dataset,
        [train_count, val_count, test_count],
        generator=torch.Generator().manual_seed(42),
    )

    train_loader = DataLoader(train_data, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_data, batch_size=BATCH_SIZE, shuffle=False)
    test_loader = DataLoader(test_data, batch_size=1, shuffle=False)

    model = DeepAA().to(device)
    perceptual_loss = PerceptualLoss().to(device)

    criterion_train = nn.L1Loss()
    criterion_metric = nn.MSELoss()

    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, "max", patience=3, factor=0.5
    )

    best_val_ssim = 0.0

    print("\ntraining\n")
    for epoch in range(NUM_EPOCHS):
        model.train()
        train_loss = 0.0
        for inputs, targets in train_loader:
            inputs, targets = inputs.to(device), targets.to(device)
            optimizer.zero_grad()
            outputs = model(inputs)

            l1 = criterion_train(outputs, targets)
            ssim_loss = 1 - ssim_metric(outputs, targets)
            perc = perceptual_loss(outputs, targets)
            eloss = edge_loss(outputs, targets)
            loss = l1 + 0.2 * perc + 0.5 * ssim_loss + 0.3 * eloss

            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        model.eval()
        val_l1_loss = 0.0
        val_mse_loss = 0.0
        val_ssim_sum = 0.0

        with torch.no_grad():
            for inputs, targets in val_loader:
                inputs, targets = inputs.to(device), targets.to(device)
                outputs = model(inputs)

                val_l1_loss += criterion_train(outputs, targets).item()
                val_mse_loss += criterion_metric(outputs, targets).item()
                val_ssim_sum += ssim_metric(outputs, targets).item()

        avg_train_loss = train_loss / len(train_loader)

        avg_val_mse = val_mse_loss / len(val_loader)
        val_psnr = calculate_psnr(avg_val_mse)
        avg_val_ssim = val_ssim_sum / len(val_loader)

        scheduler.step(avg_val_ssim)

        print(
            f"epoch {epoch+1}/{NUM_EPOCHS} | train loss: {avg_train_loss:.5f} | PSNR: {val_psnr:.2f} | SSIM: {avg_val_ssim:.4f}"
        )

        if avg_val_ssim > best_val_ssim:
            best_val_ssim = avg_val_ssim
            dummy_input = torch.randn(1, 3, 512, 512, device=device)
            torch.onnx.export(
                model,
                dummy_input,
                "neural_aa_best.onnx",
                input_names=["input"],
                output_names=["output"],
                dynamic_axes={
                    "input": {0: "batch", 2: "h", 3: "w"},
                    "output": {0: "batch", 2: "h", 3: "w"},
                },
            )
            print(f"  new best model (SSIM: {best_val_ssim:.4f})")

    print("\nevaluation\n")
    model.eval()
    test_mse_sum = 0.0
    test_ssim_sum = 0.0

    with torch.no_grad():
        for inputs, targets in test_loader:
            inputs, targets = inputs.to(device), targets.to(device)
            outputs = model(inputs)
            test_mse_sum += criterion_metric(outputs, targets).item()
            test_ssim_sum += ssim_metric(outputs, targets).item()

    avg_test_mse = test_mse_sum / len(test_loader)
    avg_test_ssim = test_ssim_sum / len(test_loader)
    final_psnr = calculate_psnr(avg_test_mse)

    print(f"results:")
    print(f"  MSE:  {avg_test_mse:.6f}")
    print(f"  PSNR: {final_psnr:.2f} dB")
    print(f"  SSIM: {avg_test_ssim:.4f}")


if __name__ == "__main__":
    main()
