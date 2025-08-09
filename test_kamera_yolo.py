import cv2
from ultralytics import YOLO
import time

# --- PENGATURAN ---
# Ganti dengan path ke file bobot .pt Anda
# MODEL_PATH = 'yolov8n.pt' # Gunakan yolov8n.pt (model nano) sebagai contoh
MODEL_PATH = 'best.pt'

# Ganti 0 dengan ID kamera lain jika Anda memiliki lebih dari satu kamera
# Contoh: 1, 2, atau path ke file video seperti 'video.mp4'
CAMERA_ID = 0

print("Pastikan Anda sudah menginstal opencv-python dan ultralytics:")
print("pip install opencv-python ultralytics")
print("----------------------------------------------------------")

# Muat model YOLO
try:
    model = YOLO(MODEL_PATH)
    print(f"Model YOLO '{MODEL_PATH}' berhasil dimuat.")
except Exception as e:
    print(f"Gagal memuat model YOLO: {e}")
    exit(1)

# Buka koneksi ke kamera
cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    print(f"Error: Tidak bisa membuka kamera dengan ID {CAMERA_ID}.")
    exit(1)

print("Kamera berhasil dibuka. Memulai deteksi...")
print("Tekan 'q' pada jendela video untuk keluar.")

# Loop untuk memproses setiap frame dari kamera
try:
    while True:
        # Baca frame dari kamera
        success, frame = cap.read()
        if not success:
            print("Gagal membaca frame dari kamera. Mengakhiri loop.")
            break

        # Jalankan deteksi YOLO pada frame
        results = model(frame)

        # Gambar hasil deteksi pada frame
        annotated_frame = results[0].plot()
        
        # Tampilkan frame yang sudah dianotasi
        cv2.imshow("Deteksi YOLOv8", annotated_frame)

        # Cek jika pengguna menekan tombol 'q' untuk keluar
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"Terjadi error saat pemrosesan video: {e}")

finally:
    # Lepaskan sumber daya
    cap.release()
    cv2.destroyAllWindows()
    print("Sumber daya kamera dan jendela tampilan dilepaskan.")
