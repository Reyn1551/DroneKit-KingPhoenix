from dronekit import connect
import time

# --- GANTI DENGAN KONEKSI DRONE FISIK ANDA ---
# Contoh untuk Linux: '/dev/ttyACM0' atau '/dev/ttyUSB0'
# Contoh untuk Windows: 'COM3'
CONNECTION_STRING = '/dev/ttyACM0'

print(f"Menyambungkan ke drone di: {CONNECTION_STRING}")
try:
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
except Exception as e:
    print(f"Gagal terhubung ke drone: {e}")
    exit(1)

print("Drone terhubung. Memulai tes Lidar...")
print("Tekan Ctrl+C untuk berhenti.")

try:
    while True:
        # vehicle.rangefinder.distance adalah jarak dalam meter
        jarak_lidar = vehicle.rangefinder.distance
        
        if jarak_lidar is not None:
            print(f"Jarak Lidar: {jarak_lidar:.2f} meter")
        else:
            print("Tidak ada data Lidar diterima (None). Periksa koneksi/konfigurasi sensor.")
            
        time.sleep(1) # Tunggu 1 detik sebelum pembacaan berikutnya

except KeyboardInterrupt:
    print("\nTes dihentikan oleh pengguna.")

finally:
    print("Menutup koneksi.")
    vehicle.close()
