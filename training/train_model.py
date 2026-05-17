"""
train_model.py — Train a lightweight CNN on real hand-drawn Greek letter images
                 and export as .onnx for use in perception_node.py.
 
Input contract:
    Tensor shape : [1, 1, 64, 64]  float32  values in [0, 1]
    Preprocessing: greyscale -> gaussian blur -> adaptive threshold ->
                   largest contour crop -> resize 64x64
 
Output contract:
    Tensor shape : [1, 10]  float32  logits (apply softmax at inference)
 
Usage:
    python3 train_model.py
    python3 train_model.py --dataset_dir dataset_real --epochs 40 --out greek_letters.onnx
"""
 
from __future__ import annotations
 
import argparse
import os
import time
from pathlib import Path
 
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as T
from PIL import Image
from torch.utils.data import DataLoader, Dataset
 
# LABELS must be alphabetically sorted — matches folder order on disk
LABELS = sorted([
    "alpha", "beta", "delta", "eta", "gamma",
    "lambda", "mu", "psi", "rho", "tau",
])
NUM_CLASSES = len(LABELS)
IMG_SIZE    = 64
 
 
def preprocess_to_tensor(bgr: np.ndarray) -> np.ndarray:
    """
    BGR image -> 64x64 float32 numpy array [0,1].
    Must match inference pipeline in test_model.py exactly.
    """
    grey   = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    grey   = cv2.GaussianBlur(grey, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(
        grey, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blockSize=11, C=2,
    )
    cnts, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnts:
        largest = max(cnts, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        crop = thresh[y:y + h, x:x + w]
        if crop.size > 0:
            thresh = crop
    resized = cv2.resize(thresh, (IMG_SIZE, IMG_SIZE),
                         interpolation=cv2.INTER_AREA)
    return resized.astype(np.float32) / 255.0
 
 
class GreekLetterDataset(Dataset):
    def __init__(self, root: str, split: str, augment: bool = False) -> None:
        self.samples: list[tuple[str, int]] = []
        self.augment = augment
 
        for idx, label in enumerate(LABELS):
            folder = Path(root, split, label)
            if not folder.exists():
                raise FileNotFoundError(f"Missing: {folder}")
            for p in sorted(folder.glob("*.jpg")):
                self.samples.append((str(p), idx))
 
        if not self.samples:
            raise ValueError(f"No images found in {root}/{split}/")
 
        self._aug = T.Compose([
            T.RandomAffine(degrees=20, translate=(0.1, 0.1),
                           scale=(0.85, 1.15), shear=8, fill=0),
            T.RandomPerspective(distortion_scale=0.2, p=0.4, fill=0),
            T.GaussianBlur(kernel_size=3, sigma=(0.1, 1.5)),
        ])
 
    def __len__(self) -> int:
        return len(self.samples)
 
    def __getitem__(self, idx: int) -> tuple[torch.Tensor, int]:
        path, label_idx = self.samples[idx]
        bgr = cv2.imread(path)
        arr = preprocess_to_tensor(bgr)
        pil = Image.fromarray((arr * 255).astype(np.uint8), mode="L")
        if self.augment:
            pil = self._aug(pil)
        arr    = np.array(pil, dtype=np.float32) / 255.0
        tensor = torch.from_numpy(arr).unsqueeze(0)
        return tensor, label_idx
 
 
class GreekLetterCNN(nn.Module):
    def __init__(self, num_classes: int = NUM_CLASSES) -> None:
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, 3, padding=1, bias=False), nn.BatchNorm2d(32), nn.ReLU(True),
            nn.Conv2d(32, 32, 3, padding=1, bias=False), nn.BatchNorm2d(32), nn.ReLU(True),
            nn.MaxPool2d(2), nn.Dropout2d(0.1),
            nn.Conv2d(32, 64, 3, padding=1, bias=False), nn.BatchNorm2d(64), nn.ReLU(True),
            nn.Conv2d(64, 64, 3, padding=1, bias=False), nn.BatchNorm2d(64), nn.ReLU(True),
            nn.MaxPool2d(2), nn.Dropout2d(0.15),
            nn.Conv2d(64, 128, 3, padding=1, bias=False), nn.BatchNorm2d(128), nn.ReLU(True),
            nn.Conv2d(128, 128, 3, padding=1, bias=False), nn.BatchNorm2d(128), nn.ReLU(True),
            nn.MaxPool2d(2), nn.Dropout2d(0.2),
        )
        self.gap = nn.AdaptiveAvgPool2d(1)
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128, 128), nn.ReLU(True), nn.Dropout(0.4),
            nn.Linear(128, num_classes),
        )
 
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.classifier(self.gap(self.features(x)))
 
 
def train(dataset_dir, epochs, batch_size, lr, out_path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")
    print(f"Labels ({len(LABELS)}): {LABELS}")
 
    train_ds = GreekLetterDataset(dataset_dir, "train", augment=True)
    val_ds   = GreekLetterDataset(dataset_dir, "val",   augment=False)
    print(f"Train: {len(train_ds)}  Val: {len(val_ds)}")
 
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,  num_workers=2)
    val_loader   = DataLoader(val_ds,   batch_size=batch_size, shuffle=False, num_workers=2)
 
    model     = GreekLetterCNN(NUM_CLASSES).to(device)
    criterion = nn.CrossEntropyLoss(label_smoothing=0.1)
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
 
    best_val_acc = 0.0
    best_ckpt    = out_path.replace(".onnx", "_best.pt")
 
    print(f"\n{'Epoch':>6}  {'TrainLoss':>10}  {'TrainAcc':>9}  {'ValLoss':>8}  {'ValAcc':>7}  {'Time':>6}")
    print("─" * 60)
 
    for epoch in range(1, epochs + 1):
        t0 = time.time()
        model.train()
        tl, tc, tt = 0.0, 0, 0
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            optimizer.zero_grad()
            logits = model(imgs)
            loss = criterion(logits, labels)
            loss.backward()
            optimizer.step()
            tl += loss.item() * imgs.size(0)
            tc += (logits.argmax(1) == labels).sum().item()
            tt += imgs.size(0)
        scheduler.step()
 
        model.eval()
        vl, vc, vt = 0.0, 0, 0
        with torch.no_grad():
            for imgs, labels in val_loader:
                imgs, labels = imgs.to(device), labels.to(device)
                logits = model(imgs)
                vl += criterion(logits, labels).item() * imgs.size(0)
                vc += (logits.argmax(1) == labels).sum().item()
                vt += imgs.size(0)
 
        v_acc = 100.0 * vc / vt
        print(f"{epoch:>6}  {tl/tt:>10.4f}  {100*tc/tt:>8.1f}%  {vl/vt:>8.4f}  {v_acc:>6.1f}%  {time.time()-t0:>5.1f}s")
 
        if v_acc > best_val_acc:
            best_val_acc = v_acc
            torch.save(model.state_dict(), best_ckpt)
            print(f"          ✓ Best: {best_val_acc:.1f}%")
 
    print(f"\nBest val accuracy: {best_val_acc:.1f}%")
 
    print(f"\nExporting -> {out_path}")
    model.load_state_dict(torch.load(best_ckpt, map_location=device))
    model.eval()
    dummy = torch.zeros(1, 1, IMG_SIZE, IMG_SIZE, device=device)
    torch.onnx.export(model, dummy, out_path,
                      input_names=["input"], output_names=["logits"],
                      opset_version=11)
    import onnx
    onnx.checker.check_model(onnx.load(out_path))
    print(f"ONNX verified. Labels: {LABELS}")
    with open(out_path.replace(".onnx", "_labels.txt"), "w") as f:
        f.write("\n".join(LABELS))
 
 
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset_dir", default="dataset_real")
    ap.add_argument("--epochs",      type=int,   default=40)
    ap.add_argument("--batch_size",  type=int,   default=64)
    ap.add_argument("--lr",          type=float, default=1e-3)
    ap.add_argument("--out",         default="greek_letters.onnx")
    args = ap.parse_args()
    train(args.dataset_dir, args.epochs, args.batch_size, args.lr, args.out)
 
 
if __name__ == "__main__":
    main()
