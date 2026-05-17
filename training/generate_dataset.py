"""
generate_dataset.py — Synthetic training dataset for Greek letter classification.
 
Generates augmented images of 10 Greek letters that mimic hand-drawn marker
on white A4 paper, as seen by the OAK-D camera after white-paper crop and
perspective correction.
 
Output structure:
    dataset/
        train/
            alpha/   *.jpg
            beta/    *.jpg
            ...
        val/
            alpha/   *.jpg
            ...
 
Usage:
    python3 generate_dataset.py
    python3 generate_dataset.py --out_dir my_dataset --samples_per_class 300
"""
 
from __future__ import annotations
 
import argparse
import os
import random
from pathlib import Path
 
import cv2
import numpy as np
 
# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
LABELS = ["alpha", "beta", "gamma", "delta", "eta",
          "lambda", "mu", "rho", "tau", "psi"]
 
# Unicode lowercase Greek letters matching each label
UNICODE_CHARS = {
    "alpha":  "α",
    "beta":   "β",
    "gamma":  "γ",
    "delta":  "δ",
    "eta":    "η",
    "lambda": "λ",
    "mu":     "μ",
    "rho":    "ρ",
    "tau":    "τ",
    "psi":    "ψ",
}
 
# PIL/Pillow font paths to try — we prefer fonts that render Greek well.
# freetype fonts on Ubuntu that support Greek:
FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSerif.ttf",
]
 
IMAGE_SIZE   = 128    # square output size (model input will be resized to 64)
CANVAS_SIZE  = 256    # render at higher res, then downscale for quality
TRAIN_SPLIT  = 0.85
 
RNG = random.Random(42)
NP_RNG = np.random.default_rng(42)
 
 
# ---------------------------------------------------------------------------
# Pillow-based renderer (preferred — supports Unicode Greek)
# ---------------------------------------------------------------------------
def _get_pil_fonts():
    """Return list of (PIL.ImageFont, name) for available fonts."""
    try:
        from PIL import ImageFont
    except ImportError:
        return []
    fonts = []
    for path in FONT_CANDIDATES:
        if os.path.exists(path):
            try:
                # Several sizes to vary stroke weight
                for size in [160, 180, 200, 220]:
                    fonts.append((ImageFont.truetype(path, size), path))
            except Exception:
                pass
    return fonts
 
 
def render_with_pil(char: str, fonts) -> np.ndarray | None:
    """Render a Unicode character using Pillow. Returns BGR numpy or None."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return None
    if not fonts:
        return None
 
    font, _ = RNG.choice(fonts)
    img_pil = Image.new("RGB", (CANVAS_SIZE, CANVAS_SIZE), color=(255, 255, 255))
    draw    = ImageDraw.Draw(img_pil)
 
    # Measure text and centre it
    bbox = draw.textbbox((0, 0), char, font=font)
    tw   = bbox[2] - bbox[0]
    th   = bbox[3] - bbox[1]
    x    = (CANVAS_SIZE - tw) // 2 - bbox[0]
    y    = (CANVAS_SIZE - th) // 2 - bbox[1]
 
    # Slightly randomise position
    x += RNG.randint(-15, 15)
    y += RNG.randint(-15, 15)
 
    # Draw with near-black colour (marker pen variation)
    ink = tuple(RNG.randint(0, 40) for _ in range(3))
    draw.text((x, y), char, font=font, fill=ink)
 
    bgr = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
    return bgr
 
 
# ---------------------------------------------------------------------------
# Augmentation helpers
# ---------------------------------------------------------------------------
 
def _random_perspective(img: np.ndarray, strength: float = 0.12) -> np.ndarray:
    """Apply a mild random perspective warp."""
    h, w = img.shape[:2]
    d    = int(min(h, w) * strength)
    src  = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst  = np.float32([
        [RNG.randint(0, d),     RNG.randint(0, d)],
        [w - RNG.randint(0, d), RNG.randint(0, d)],
        [w - RNG.randint(0, d), h - RNG.randint(0, d)],
        [RNG.randint(0, d),     h - RNG.randint(0, d)],
    ])
    M   = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(img, M, (w, h),
                               borderMode=cv2.BORDER_CONSTANT,
                               borderValue=(255, 255, 255))
 
 
def _random_rotation(img: np.ndarray, max_deg: float = 25.0) -> np.ndarray:
    """Rotate by a random angle."""
    h, w   = img.shape[:2]
    angle  = RNG.uniform(-max_deg, max_deg)
    M      = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
    return cv2.warpAffine(img, M, (w, h),
                          borderMode=cv2.BORDER_CONSTANT,
                          borderValue=(255, 255, 255))
 
 
def _random_scale(img: np.ndarray,
                  lo: float = 0.75, hi: float = 1.15) -> np.ndarray:
    """Scale the letter relative to the canvas."""
    h, w  = img.shape[:2]
    scale = RNG.uniform(lo, hi)
    new_w = int(w * scale)
    new_h = int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    # Paste onto white canvas, centred
    canvas = np.full((h, w, 3), 255, dtype=np.uint8)
    y0     = max(0, (h - new_h) // 2)
    x0     = max(0, (w - new_w) // 2)
    y1     = min(h, y0 + new_h)
    x1     = min(w, x0 + new_w)
    canvas[y0:y1, x0:x1] = resized[:y1 - y0, :x1 - x0]
    return canvas
 
 
def _add_noise(img: np.ndarray) -> np.ndarray:
    """Add Gaussian noise and optional blur to simulate camera/print quality."""
    out = img.astype(np.float32)
    sigma = RNG.uniform(0, 8)
    noise = NP_RNG.normal(0, sigma, out.shape).astype(np.float32)
    out   = np.clip(out + noise, 0, 255).astype(np.uint8)
    if RNG.random() < 0.4:
        k = RNG.choice([3, 5])
        out = cv2.GaussianBlur(out, (k, k), 0)
    return out
 
 
def _add_background_texture(img: np.ndarray) -> np.ndarray:
    """
    Simulate the paper-on-bucket look: slight off-white background tint,
    occasional shadow gradient, and mild paper texture.
    """
    h, w = img.shape[:2]
    out  = img.copy().astype(np.float32)
 
    # Off-white tint
    tint = RNG.uniform(-15, 10)
    out  = np.clip(out + tint, 0, 255)
 
    # Shadow gradient (25% chance)
    if RNG.random() < 0.25:
        direction = RNG.choice(["h", "v"])
        strength  = RNG.uniform(0, 40)
        if direction == "h":
            grad = np.linspace(0, strength, w, dtype=np.float32)
            grad = np.tile(grad, (h, 1))
        else:
            grad = np.linspace(0, strength, h, dtype=np.float32)
            grad = np.tile(grad, (w, 1)).T
        grad = grad[:, :, np.newaxis]
        out  = np.clip(out - grad, 0, 255)
 
    # Ink colour variation: make strokes slightly brown/grey instead of pure black
    grey = cv2.cvtColor(out.astype(np.uint8), cv2.COLOR_BGR2GRAY)
    dark_mask = grey < 80
    if dark_mask.any():
        shift = np.array([RNG.randint(-10, 10),
                          RNG.randint(-10, 10),
                          RNG.randint(-10, 10)], dtype=np.float32)
        out[dark_mask] = np.clip(out[dark_mask] + shift, 0, 60)
 
    return out.astype(np.uint8)
 
 
def augment(img: np.ndarray) -> np.ndarray:
    """Apply the full augmentation pipeline to one rendered image."""
    img = _random_scale(img)
    img = _random_rotation(img)
    img = _random_perspective(img)
    img = _add_background_texture(img)
    img = _add_noise(img)
    img = cv2.resize(img, (IMAGE_SIZE, IMAGE_SIZE),
                     interpolation=cv2.INTER_AREA)
    return img
 
 
# ---------------------------------------------------------------------------
# Dataset generation
# ---------------------------------------------------------------------------
 
def generate_dataset(out_dir: str, samples_per_class: int) -> None:
    pil_fonts = _get_pil_fonts()
    if not pil_fonts:
        raise RuntimeError(
            "No suitable fonts found. Install fonts:\n"
            "  sudo apt install fonts-dejavu fonts-freefont-ttf\n"
            "and re-run."
        )
    print(f"Found {len(pil_fonts)} font variants.")
 
    n_train = int(samples_per_class * TRAIN_SPLIT)
    n_val   = samples_per_class - n_train
 
    for split, n in [("train", n_train), ("val", n_val)]:
        for label in LABELS:
            Path(out_dir, split, label).mkdir(parents=True, exist_ok=True)
 
    total = 0
    for label in LABELS:
        char = UNICODE_CHARS[label]
        print(f"  Generating '{label}' ({char}) ...", end=" ", flush=True)
 
        for split, n in [("train", n_train), ("val", n_val)]:
            for i in range(n):
                rendered = render_with_pil(char, pil_fonts)
                if rendered is None:
                    raise RuntimeError("PIL rendering failed.")
                aug  = augment(rendered)
                path = os.path.join(out_dir, split, label, f"{label}_{i:04d}.jpg")
                cv2.imwrite(path, aug, [cv2.IMWRITE_JPEG_QUALITY, 92])
                total += 1
 
        print(f"train={n_train}, val={n_val}")
 
    print(f"\nDataset complete: {total} images in '{out_dir}/'")
    print(f"  Classes : {LABELS}")
    print(f"  Per class: {n_train} train / {n_val} val")
    print(f"  Image size: {IMAGE_SIZE}×{IMAGE_SIZE} px")
 
 
# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description="Generate synthetic Greek letter dataset.")
    ap.add_argument("--out_dir",           default="dataset",
                    help="Output directory (default: dataset)")
    ap.add_argument("--samples_per_class", type=int, default=400,
                    help="Total images per class (default: 400)")
    args = ap.parse_args()
    generate_dataset(args.out_dir, args.samples_per_class)
 
 
if __name__ == "__main__":
    main()
