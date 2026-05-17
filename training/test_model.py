"""
test_model.py — Evaluate the trained Greek letter ONNX model.

Tests:
  1. Run inference on the full validation set and print per-class accuracy
     and a confusion matrix.
  2. Optionally test on a real image from the OAK-D camera by specifying
     --image path/to/image.jpg  (the script will auto-detect and crop the
     white paper region before classifying).

Usage:
    # Evaluate on validation set
    python3 test_model.py

    # Evaluate on a specific image file
    python3 test_model.py --image test_photo.jpg

    # Use a different model or dataset
    python3 test_model.py --model greek_letters.onnx --dataset dataset
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

LABELS = ["alpha", "beta", "delta", "eta", "gamma", "lambda", "mu", "psi", "rho", "tau"]
IMG_SIZE = 64


# ---------------------------------------------------------------------------
# Preprocessing — must exactly match perception_node pipeline
# ---------------------------------------------------------------------------

def detect_paper(bgr: np.ndarray) -> np.ndarray | None:
    """
    Detect the white paper square on the bucket and return a
    perspective-corrected crop. Returns None if not found.
    """
    h, w  = bgr.shape[:2]
    grey  = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(grey, 190, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=3)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  kernel, iterations=2)
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_cnt  = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 0.03 * h * w:
            continue
        peri   = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        if len(approx) == 4 and area > best_area:
            best_area = area
            best_cnt  = approx
        elif best_cnt is None and area > best_area:
            hull = cv2.convexHull(cnt)
            hull_approx = cv2.approxPolyDP(
                hull, 0.03 * cv2.arcLength(hull, True), True)
            if len(hull_approx) == 4:
                best_area = area
                best_cnt  = hull_approx

    if best_cnt is None:
        return None

    pts  = best_cnt.reshape(4, 2).astype(np.float32)
    s    = pts.sum(axis=1)
    d    = np.diff(pts, axis=1).ravel()
    rect = np.float32([
        pts[np.argmin(s)],
        pts[np.argmin(d)],
        pts[np.argmax(s)],
        pts[np.argmax(d)],
    ])
    ow = int(max(np.linalg.norm(rect[0] - rect[1]),
                 np.linalg.norm(rect[2] - rect[3])))
    oh = int(max(np.linalg.norm(rect[0] - rect[3]),
                 np.linalg.norm(rect[1] - rect[2])))
    ow, oh = max(ow, 64), max(oh, 64)
    dst = np.float32([[0, 0], [ow, 0], [ow, oh], [0, oh]])
    M   = cv2.getPerspectiveTransform(rect, dst)
    return cv2.warpPerspective(bgr, M, (ow, oh))


def preprocess_letter(bgr: np.ndarray) -> np.ndarray | None:
    """
    Isolate ink strokes from a paper crop → [1, 1, 64, 64] float32 tensor.

    Pipeline (MNIST-style):
      1. Greyscale + Gaussian blur
      2. Adaptive threshold inverted  (ink=255, background=0)
      3. Largest contour → tight bounding box crop
      4. Resize 64×64, normalise [0, 1]

    Returns None if no contour found.
    """
    grey   = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    grey   = cv2.GaussianBlur(grey, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(
        grey, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blockSize=11, C=2,
    )
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest     = max(contours, key=cv2.contourArea)
    x, y, w, h  = cv2.boundingRect(largest)
    letter_crop = thresh[y:y + h, x:x + w]
    if letter_crop.size == 0:
        return None
    resized = cv2.resize(letter_crop, (64, 64), interpolation=cv2.INTER_AREA)
    tensor  = resized.astype(np.float32) / 255.0
    return tensor[np.newaxis, np.newaxis, :, :]   # [1, 1, 64, 64]


def softmax(x: np.ndarray) -> np.ndarray:
    e = np.exp(x - x.max())
    return e / e.sum()


def infer(
    session: ort.InferenceSession,
    bgr: np.ndarray,
    skip_paper: bool = False,
) -> tuple[str, float]:
    """
    Full pipeline: paper detection -> ink isolation -> inference.
    skip_paper=True  — already a clean letter crop, skip paper detection.
    skip_paper=False — full scene image, run paper detection first.
    """
    if skip_paper:
        src = bgr
    else:
        paper = detect_paper(bgr)
        src   = paper if paper is not None else bgr

    tensor = preprocess_letter(src)
    if tensor is None:
        return "unknown", 0.0

    input_name = session.get_inputs()[0].name
    logits     = session.run(None, {input_name: tensor})[0][0]
    probs      = softmax(logits)
    idx        = int(np.argmax(probs))
    return LABELS[idx], float(probs[idx])


# ---------------------------------------------------------------------------
# Validation set evaluation
# ---------------------------------------------------------------------------

def evaluate_val_set(session: ort.InferenceSession, dataset_dir: str) -> None:
    print(f"\n{'─'*55}")
    print("Validation set evaluation")
    print(f"{'─'*55}")

    confusion = np.zeros((len(LABELS), len(LABELS)), dtype=int)
    per_class_correct = {l: 0 for l in LABELS}
    per_class_total   = {l: 0 for l in LABELS}

    for true_idx, label in enumerate(LABELS):
        folder = Path(dataset_dir, "val", label)
        if not folder.exists():
            print(f"  WARNING: missing val folder: {folder}")
            continue
        imgs = list(folder.glob("*.jpg"))
        if not imgs:
            continue
        for img_path in imgs:
            bgr  = cv2.imread(str(img_path))
            pred, conf = infer(session, bgr, skip_paper=True)
            pred_idx   = LABELS.index(pred)
            confusion[true_idx][pred_idx] += 1
            per_class_total[label]   += 1
            if pred == label:
                per_class_correct[label] += 1

    # Per-class accuracy
    print(f"\n{'Label':<10}  {'Correct':>8}  {'Total':>6}  {'Accuracy':>9}")
    print(f"{'─'*40}")
    total_correct = 0
    total_count   = 0
    for label in LABELS:
        c = per_class_correct[label]
        t = per_class_total[label]
        acc = 100.0 * c / t if t > 0 else 0.0
        print(f"{label:<10}  {c:>8}  {t:>6}  {acc:>8.1f}%")
        total_correct += c
        total_count   += t

    overall = 100.0 * total_correct / total_count if total_count > 0 else 0.0
    print(f"{'─'*40}")
    print(f"{'OVERALL':<10}  {total_correct:>8}  {total_count:>6}  {overall:>8.1f}%")

    # Confusion matrix
    print(f"\nConfusion matrix (rows=true, cols=predicted):")
    header = f"{'':10}" + "".join(f"{l[:4]:>6}" for l in LABELS)
    print(header)
    for i, label in enumerate(LABELS):
        row = f"{label:<10}" + "".join(f"{confusion[i][j]:>6}" for j in range(len(LABELS)))
        print(row)

    # Most confused pairs
    print("\nMost confused pairs:")
    pairs = []
    for i in range(len(LABELS)):
        for j in range(len(LABELS)):
            if i != j and confusion[i][j] > 0:
                pairs.append((confusion[i][j], LABELS[i], LABELS[j]))
    for count, true_l, pred_l in sorted(pairs, reverse=True)[:5]:
        print(f"  {true_l} → {pred_l}: {count} times")


# ---------------------------------------------------------------------------
# Single image test
# ---------------------------------------------------------------------------

def test_image(
    session: ort.InferenceSession,
    image_path: str,
    skip_paper: bool = False,
) -> None:
    print(f"\nTesting image: {image_path}")
    bgr = cv2.imread(image_path)
    if bgr is None:
        print(f"ERROR: Cannot read {image_path}")
        return

    # Step 1 — paper detection (skipped for clean letter crops)
    if skip_paper:
        src = bgr
        print("  Skipping paper detection (--skip_paper flag set).")
    else:
        paper = detect_paper(bgr)
        if paper is not None:
            print("  White paper detected — using perspective-corrected crop.")
            cv2.imwrite(image_path.replace(".jpg", "_paper.jpg")
                        .replace(".png", "_paper.jpg"), paper)
        else:
            print("  No paper region found — classifying full image.")
        src = paper if paper is not None else bgr

    # Step 2 — show ink isolation
    tensor = preprocess_letter(src)
    if tensor is not None:
        ink_vis = (tensor[0, 0] * 255).astype(np.uint8)
        ink_vis = cv2.resize(ink_vis, (128, 128), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(image_path.replace(".jpg", "_ink.jpg")
                    .replace(".png", "_ink.jpg"), ink_vis)
        print("  Ink isolation saved as _ink.jpg")
    else:
        print("  WARNING: no ink contour found.")

    label, confidence = infer(session, bgr, skip_paper=skip_paper)
    print(f"  Prediction : {label}")
    print(f"  Confidence : {confidence*100:.1f}%")

    # Annotate original image
    annotated = bgr.copy()
    if paper is not None:
        grey  = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(grey, 190, 255, cv2.THRESH_BINARY)
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            largest = max(cnts, key=cv2.contourArea)
            cv2.drawContours(annotated, [largest], -1, (0, 255, 0), 3)

    colour = (0, 200, 0) if confidence > 0.7 else (0, 165, 255)
    cv2.putText(annotated, f"{label} ({confidence*100:.0f}%)",
                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, colour, 3)
    out_path = image_path.replace(".jpg", "_result.jpg").replace(".png", "_result.jpg")
    cv2.imwrite(out_path, annotated)
    print(f"  Annotated image saved: {out_path}")


# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(
        description="Evaluate Greek letter ONNX model.")
    ap.add_argument("--model",   default="greek_letters.onnx",
                    help="ONNX model path (default: greek_letters.onnx)")
    ap.add_argument("--dataset", default="dataset",
                    help="Dataset directory (default: dataset)")
    ap.add_argument("--image",   default="",
                    help="Optional: path to a real image to test")
    ap.add_argument("--skip_paper", action="store_true",
                    help="Skip paper detection (use for clean letter crops, "
                         "not full robot camera frames)")
    args = ap.parse_args()

    if not os.path.exists(args.model):
        print(f"ERROR: Model not found: {args.model}")
        print("Run train_model.py first.")
        return

    print(f"Loading model: {args.model}")
    session = ort.InferenceSession(
        args.model, providers=["CPUExecutionProvider"])
    print(f"Model loaded. Input: {session.get_inputs()[0].shape}")

    if args.image:
        test_image(session, args.image, skip_paper=args.skip_paper)
    else:
        evaluate_val_set(session, args.dataset)


if __name__ == "__main__":
    main()