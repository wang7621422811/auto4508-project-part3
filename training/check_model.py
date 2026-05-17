import torch, cv2, numpy as np
from train_model import GreekLetterCNN

LABELS = ['alpha', 'beta', 'delta', 'eta', 'gamma', 'lambda', 'mu', 'psi', 'rho', 'tau']
model = GreekLetterCNN(10)
model.load_state_dict(torch.load('greek_letters_best.pt', map_location='cpu'))
model.eval()

for label in LABELS:
    img_path = f'dataset_real/val/{label}/0000.jpg'
    bgr = cv2.imread(img_path)
    grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    grey = cv2.GaussianBlur(grey, (5,5), 0)
    thresh = cv2.adaptiveThreshold(grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 11, 2)
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest = max(cnts, key=cv2.contourArea)
    x,y,w,h = cv2.boundingRect(largest)
    crop = thresh[y:y+h, x:x+w]
    resized = cv2.resize(crop, (64,64)).astype(np.float32)/255.0
    tensor = torch.from_numpy(resized[np.newaxis, np.newaxis])
    with torch.no_grad():
        probs = torch.softmax(model(tensor), dim=1)[0].numpy()
    pred = LABELS[np.argmax(probs)]
    print(f'{label:10s} -> {pred:10s} ({100*probs.max():.1f}%)')
