"""
augment_real_letters.py — Build a training dataset purely from the real
hand-drawn letter images by augmentation. NO font rendering at all.
 
Augmentations applied per sample:
  - Rotation           ±30°
  - Scale              0.75 – 1.20×
  - Perspective warp   mild (simulates camera angle)
  - Translation        ±10% of image size
  - Brightness/contrast variation
  - Gaussian noise
  - Occasional blur
  - Occasional horizontal flip (only for symmetric-ish letters)
 
Output structure (matches what train_model.py expects):
    dataset_real/
        train/
            alpha/  0000.jpg  0001.jpg ...
            beta/   ...
        val/
            alpha/  ...
 
Usage:
    python3 augment_real_letters.py
    python3 augment_real_letters.py --samples 500 --out dataset_real
"""
 
from __future__ import annotations
 
import argparse
import os
import random
from pathlib import Path
 
import cv2
import numpy as np
 
# ---------------------------------------------------------------------------
# Source images — update paths if running from a different directory
# ---------------------------------------------------------------------------
SOURCE_IMAGES: dict[str, str] = {
    "alpha":  "alpha.png",
    "beta":   "beta.png",
    "delta":  "delta.png",
    "gamma":  "gamma.png",
    "lambda": "lambda.png",
    "mu":     "mu.png",
    "eta":    "nu2.png",    # file was named nu2.png
    "psi":    "psi.png",
    "rho":    "rho.png",
    "tau":    "tau.png",
}
 
# Letters that are roughly horizontally symmetric — safe to flip
FLIPPABLE = {"alpha", "delta", "mu", "psi", "tau"}
 
LABELS      = list(SOURCE_IMAGES.keys())
TRAIN_SPLIT = 0.85
IMG_SIZE    = 64      # final output size — matches model input
CANVAS      = 96      # work at higher res before downscaling
 
RNG    = random.Random(42)
NP_RNG = np.random.default_rng(42)
 
 
# ---------------------------------------------------------------------------
# Preprocessing — isolate the letter on a clean white background
# ---------------------------------------------------------------------------
 
def extract_letter(bgr: np.ndarray) -> np.ndarray:
    """
    Given a photo of a letter on white/light paper:
      1. Convert to greyscale
      2. Threshold to get the dark ink strokes
      3. Crop tightly to the ink bounding box
      4. Paste onto a clean white square canvas with padding
 
    Returns a square BGR image (CANVAS × CANVAS) with white background.
    """
    grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
 
    # Adaptive threshold — handles uneven lighting across the paper
    thresh = cv2.adaptiveThreshold(
        grey, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blockSize=15, C=8,
    )
 
    # Remove tiny noise specks
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
 
    # Find bounding box of all ink pixels
    coords = cv2.findNonZero(thresh)
    if coords is None:
        # Fallback: use whole image
        x, y, w, h = 0, 0, bgr.shape[1], bgr.shape[0]
    else:
        x, y, w, h = cv2.boundingRect(coords)
 
    # Add padding (15% of the larger dimension)
    pad = int(max(w, h) * 0.15)
    x0  = max(0, x - pad)
    y0  = max(0, y - pad)
    x1  = min(bgr.shape[1], x + w + pad)
    y1  = min(bgr.shape[0], y + h + pad)
 
    crop = bgr[y0:y1, x0:x1]
 
    # Paste onto a square white canvas
    canvas = np.full((CANVAS, CANVAS, 3), 255, dtype=np.uint8)
    ch, cw = crop.shape[:2]
    scale  = min(CANVAS / cw, CANVAS / ch) * 0.80   # 80% of canvas
    nw     = max(1, int(cw * scale))
    nh     = max(1, int(ch * scale))
    resized = cv2.resize(crop, (nw, nh), interpolation=cv2.INTER_AREA)
 
    # Centre on canvas
    ox = (CANVAS - nw) // 2
    oy = (CANVAS - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = resized
 
    return canvas
 
 
# ---------------------------------------------------------------------------
# Augmentation helpers
# ---------------------------------------------------------------------------
 
def _rotate(img: np.ndarray, max_deg: float = 30.0) -> np.ndarray:
    h, w  = img.shape[:2]
    angle = RNG.uniform(-max_deg, max_deg)
    M     = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
    return cv2.warpAffine(img, M, (w, h),
                          borderMode=cv2.BORDER_CONSTANT,
                          borderValue=(255, 255, 255))
 
 
def _scale(img: np.ndarray, lo: float = 0.75, hi: float = 1.20) -> np.ndarray:
    h, w   = img.shape[:2]
    s      = RNG.uniform(lo, hi)
    nw, nh = int(w * s), int(h * s)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas  = np.full((h, w, 3), 255, dtype=np.uint8)
    y0 = max(0, (h - nh) // 2)
    x0 = max(0, (w - nw) // 2)
    y1 = min(h, y0 + nh)
    x1 = min(w, x0 + nw)
    canvas[y0:y1, x0:x1] = resized[:y1 - y0, :x1 - x0]
    return canvas
 
 
def _translate(img: np.ndarray, max_frac: float = 0.10) -> np.ndarray:
    h, w = img.shape[:2]
    tx   = int(RNG.uniform(-max_frac, max_frac) * w)
    ty   = int(RNG.uniform(-max_frac, max_frac) * h)
    M    = np.float32([[1, 0, tx], [0, 1, ty]])
    return cv2.warpAffine(img, M, (w, h),
                          borderMode=cv2.BORDER_CONSTANT,
                          borderValue=(255, 255, 255))
 
 
def _perspective(img: np.ndarray, strength: float = 0.08) -> np.ndarray:
    h, w = img.shape[:2]
    # Shrink source points inward so warp edges never touch image border
    margin = int(min(h, w) * 0.06)
    d      = int(min(h, w) * strength)
    src = np.float32([
        [margin,     margin],
        [w - margin, margin],
        [w - margin, h - margin],
        [margin,     h - margin],
    ])
    dst = np.float32([
        [margin + RNG.randint(0, d),         margin + RNG.randint(0, d)],
        [w - margin - RNG.randint(0, d),     margin + RNG.randint(0, d)],
        [w - margin - RNG.randint(0, d),     h - margin - RNG.randint(0, d)],
        [margin + RNG.randint(0, d),         h - margin - RNG.randint(0, d)],
    ])
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(img, M, (w, h),
                               borderMode=cv2.BORDER_CONSTANT,
                               borderValue=(255, 255, 255))
 
 
def _brightness_contrast(img: np.ndarray) -> np.ndarray:
    """Vary brightness and contrast to simulate lighting changes."""
    alpha = RNG.uniform(0.80, 1.20)   # contrast
    beta  = RNG.uniform(-20, 20)      # brightness
    out   = np.clip(img.astype(np.float32) * alpha + beta, 0, 255)
    return out.astype(np.uint8)
 
 
def _noise(img: np.ndarray) -> np.ndarray:
    sigma = RNG.uniform(0, 10)
    noise = NP_RNG.normal(0, sigma, img.shape).astype(np.float32)
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
 
 
def _blur(img: np.ndarray) -> np.ndarray:
    if RNG.random() < 0.35:
        k = RNG.choice([3, 5])
        return cv2.GaussianBlur(img, (k, k), 0)
    return img
 
 
def _flip(img: np.ndarray, label: str) -> np.ndarray:
    if label in FLIPPABLE and RNG.random() < 0.35:
        return cv2.flip(img, 1)
    return img
 
 
def _thickness_jitter(img: np.ndarray) -> np.ndarray:
    """
    Slightly erode or dilate to simulate thicker/thinner marker strokes.
    Works on the ink (dark) pixels.
    """
    grey   = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, ink = cv2.threshold(grey, 200, 255, cv2.THRESH_BINARY_INV)
    k_size = RNG.choice([2, 3])
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_size, k_size))
    choice = RNG.random()
    if choice < 0.3:
        ink = cv2.dilate(ink, kernel, iterations=1)   # thicker
    elif choice < 0.6:
        ink = cv2.erode(ink, kernel, iterations=1)    # thinner
 
    # Reconstruct BGR: white bg, dark ink
    out = np.full_like(img, 255)
    out[ink > 0] = [20, 20, 20]
    return out
 
 
def augment_one(img: np.ndarray, label: str) -> np.ndarray:
    """Apply the full augmentation pipeline to one source image."""
    img = _scale(img)
    img = _rotate(img)
    img = _translate(img)
    img = _perspective(img)
    img = _flip(img, label)
    img = _thickness_jitter(img)
    img = _brightness_contrast(img)
    img = _noise(img)
    img = _blur(img)
    img = cv2.resize(img, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_AREA)
    return img
 
 
# ---------------------------------------------------------------------------
# Dataset generation
# ---------------------------------------------------------------------------
 
def generate(out_dir: str, samples_per_class: int, source_dir: str) -> None:
    n_train = int(samples_per_class * TRAIN_SPLIT)
    n_val   = samples_per_class - n_train
 
    for split in ("train", "val"):
        for label in LABELS:
            Path(out_dir, split, label).mkdir(parents=True, exist_ok=True)
 
    missing = []
    for label, fname in SOURCE_IMAGES.items():
        full = os.path.join(source_dir, fname)
        if not os.path.exists(full):
            missing.append(f"  {label}: {full}")
    if missing:
        print("ERROR — missing source images:")
        print("\n".join(missing))
        print("\nCopy the source images to the same folder as this script,")
        print("or pass --source_dir pointing to their location.")
        return
 
    total = 0
    for label, fname in SOURCE_IMAGES.items():
        src_path = os.path.join(source_dir, fname)
        src_bgr  = cv2.imread(src_path)
        if src_bgr is None:
            print(f"ERROR: cannot read {src_path}")
            return
 
        # Extract clean letter on white canvas
        base = extract_letter(src_bgr)
 
        print(f"  {label:8s} → ", end="", flush=True)
 
        for split, n in [("train", n_train), ("val", n_val)]:
            for i in range(n):
                aug  = augment_one(base.copy(), label)
                path = os.path.join(out_dir, split, label, f"{i:04d}.jpg")
                cv2.imwrite(path, aug, [cv2.IMWRITE_JPEG_QUALITY, 92])
                total += 1
 
        print(f"train={n_train}  val={n_val}")
 
    print(f"\nDataset complete: {total} images in '{out_dir}/'")
    print(f"  Per class : {n_train} train / {n_val} val")
    print(f"  Image size: {IMG_SIZE}×{IMG_SIZE} px")
    print(f"\nNext step:")
    print(f"  python3 train_model.py --dataset_dir {out_dir} --epochs 40 --out greek_letters.onnx")
 
 
# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(
        description="Augment real hand-drawn letters into a training dataset.")
    ap.add_argument("--out_dir",  default="dataset_real",
                    help="Output dataset directory (default: dataset_real)")
    ap.add_argument("--samples",  type=int, default=500,
                    help="Augmented images per class (default: 500)")
    ap.add_argument("--source_dir", default=".",
                    help="Directory containing the source .png letter images "
                         "(default: current directory)")
    args = ap.parse_args()
    generate(args.out_dir, args.samples, args.source_dir)
 
 
if __name__ == "__main__":
    main()
