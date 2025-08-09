from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil 
import time
import math
import random # Digunakan untuk simulasi deteksi objek

# === PENGATURAN & VARIABEL ===
# Ganti dengan koneksi riil, misal '/dev/ttyACM0' untuk koneksi serial via USB
# 'udp:127.0.0.1:14550' digunakan untuk koneksi ke simulator (SITL)
CONNECTION_STRING = 'udp:127.0.0.1:14550'
TAKEOFF_ALTITUDE = 2  # Ketinggian takeoff awal dalam meter
TARGET_WAYPOINT = LocationGlobalRelative(-7.833018, 110.384288, TAKEOFF_ALTITUDE) # Contoh waypoint
TARGET_WAYPOINT2 = LocationGlobalRelative(-7.833000, 110.384634, TAKEOFF_ALTITUDE) # Contoh waypoint
LANDING_SPEED = 0.3 # Kecepatan turun saat landing presisi (m/s)

# Pengaturan untuk Kamera & Koreksi Posisi
CAM_WIDTH_PX = 640  # Lebar frame kamera dalam piksel
CAM_HEIGHT_PX = 480 # Tinggi frame kamera dalam piksel
CENTER_X = CAM_WIDTH_PX / 2
CENTER_Y = CAM_HEIGHT_PX / 2
KP_GAIN = 0.005 # Proportional gain untuk koreksi posisi (bisa di-tuning)


# === KONEKSI KE DRONE ===
print(f"Menyambungkan ke drone di: {CONNECTION_STRING}")
try:
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
except Exception as e:
    print(f"Gagal terhubung ke drone: {e}")
    exit(1)

print("Drone terhubung!")

# === FUNGSI-FUNGSI UTAMA ===

def arm_and_takeoff(target_altitude):
    """
    Mengaktifkan motor (arm) dan lepas landas ke ketinggian target.
    Menggunakan altimeter utama (Barometer/GPS) karena ini perilaku default simple_takeoff.
    """
    print("Mengecek status drone sebelum takeoff...")
    while not vehicle.is_armable:
        print(" Menunggu drone siap untuk di-arm...")
        time.sleep(1)

    print("Mengganti mode ke GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    
    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Menunggu arming...")
        time.sleep(1)

    print(f"Takeoff menuju {target_altitude} meter!")
    vehicle.simple_takeoff(target_altitude)

    # Tunggu hingga drone mencapai ketinggian yang cukup
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f" Ketinggian (GPS/Baro): {current_altitude:.2f}m")
        if current_altitude >= target_altitude * 0.95:
            print("Ketinggian takeoff tercapai")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """
    Menghitung jarak antara dua titik LocationGlobalRelative dalam meter.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def detect_object_and_get_center():
    """
    !!! FUNGSI PLACEHOLDER / SIMULASI !!!
    Di sinilah integrasi dengan model Computer Vision (CV) dilakukan.
    Fungsi ini harusnya:
    1. Mengambil frame dari kamera.
    2. Menjalankan deteksi objek (misal: landasan/marker).
    3. Jika objek terdeteksi, kembalikan (True, center_x, center_y).
    4. Jika tidak, kembalikan (False, None, None).
    
    Untuk simulasi, kita buat seolah-olah objek selalu terdeteksi dengan sedikit noise.
    """
    is_detected = True # Asumsikan selalu terdeteksi untuk simulasi
    if is_detected:
        # Simulasikan posisi target dengan sedikit pergeseran acak dari pusat
        simulated_x = CENTER_X + random.uniform(-50, 50)
        simulated_y = CENTER_Y + random.uniform(-50, 50)
        return (True, simulated_x, simulated_y)
    return (False, None, None)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Mengirim perintah kecepatan dalam frame NED (North-East-Down).
    velocity_x: kecepatan ke Utara (m/s)
    velocity_y: kecepatan ke Timur (m/s)
    velocity_z: kecepatan ke Bawah (m/s)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (hanya velocity yang aktif)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)
    vehicle.send_mavlink(msg)

def precision_land():
    """
    Fungsi utama untuk melakukan pendaratan presisi.
    """
    print("Memulai prosedur pendaratan presisi...")
    
    while True:
        # Baca ketinggian dari LIDAR
        # current_lidar_distance = vehicle.rangefinder.distance
        current_lidar_distance = vehicle.location.global_relative_frame.alt
        
        # Jika Lidar tidak tersedia atau error, batalkan
        if current_lidar_distance is None:
            print("ERROR: Data Lidar tidak tersedia. Membatalkan landing presisi.")
            break
        
        print(f"Ketinggian (Lidar): {current_lidar_distance:.2f}m")

        # Kondisi untuk menyelesaikan pendaratan
        if current_lidar_distance < 0.2: # Target 20 cm
            print("Target pendaratan tercapai. Menghentikan motor.")
            send_ned_velocity(0, 0, 0) # Hentikan semua gerakan
            vehicle.armed = False
            while vehicle.armed:
                time.sleep(1)
            break
            
        # 1. Deteksi objek
        detected, target_x, target_y = detect_object_and_get_center()

        if detected:
            # 2. Hitung error dari pusat frame
            error_x = target_x - CENTER_X
            error_y = target_y - CENTER_Y
            print(f"Target terdeteksi. Error X:{error_x:.0f}px, Y:{error_y:.0f}px")

            # 3. Hitung kecepatan koreksi berdasarkan error (simple P-controller)
            # Ingat: error_y (atas/bawah di frame) -> velocity_x (maju/mundur drone)
            #         error_x (kiri/kanan di frame) -> velocity_y (kiri/kanan drone)
            vel_x = KP_GAIN * error_y 
            vel_y = KP_GAIN * error_x

            # Batasi kecepatan maksimum untuk keamanan
            vel_x = max(min(vel_x, 0.5), -0.5)
            vel_y = max(min(vel_y, 0.5), -0.5)
            
            # 4. Kirim perintah kecepatan (koreksi posisi + turun perlahan)
            print(f"Mengirim V_N:{vel_x:.2f} m/s, V_E:{vel_y:.2f} m/s, V_D:{LANDING_SPEED:.2f} m/s")
            send_ned_velocity(vel_x, vel_y, LANDING_SPEED)
        else:
            # Jika target hilang, drone hanya akan turun perlahan di tempat
            print("Target tidak terdeteksi. Turun vertikal perlahan.")
            send_ned_velocity(0, 0, LANDING_SPEED)

        time.sleep(0.1) # Loop pendaratan berjalan setiap 100ms


# === MAIN EXECUTION SCRIPT ===
if __name__ == '__main__':
    try:
        # 1. Arm dan Takeoff
        arm_and_takeoff(TAKEOFF_ALTITUDE)

        # 2. Terbang ke Waypoint
        print(f"Terbang menuju waypoint di {TARGET_WAYPOINT.lat}, {TARGET_WAYPOINT.lon}")
        vehicle.simple_goto(TARGET_WAYPOINT)
       

        # Tunggu hingga drone mendekati waypoint
        while True:
            distance_to_target = get_distance_metres(vehicle.location.global_relative_frame, TARGET_WAYPOINT)
            print(f"Jarak ke waypoint: {distance_to_target:.2f}m")
            if distance_to_target < 1.0: # Anggap sudah sampai jika jarak < 1 meter
                print("Waypoint tercapai.")
                break
            time.sleep(1)

        print(f"Terbang menuju waypoint di {TARGET_WAYPOINT2.lat}, {TARGET_WAYPOINT2.lon}")
        vehicle.simple_goto(TARGET_WAYPOINT2)
        while True:
            distance_to_target = get_distance_metres(vehicle.location.global_relative_frame, TARGET_WAYPOINT2)
            print(f"Jarak ke waypoint: {distance_to_target:.2f}m")
            if distance_to_target < 1.0: # Anggap sudah sampai jika jarak < 1 meter
                print("Waypoint tercapai.")
                break
            time.sleep(1)
        
        # Beri jeda sejenak sebelum memulai landing
        print("Menstabilkan posisi sebelum landing presisi...")
        time.sleep(2)
        
        # 3. Mulai Pendaratan Presisi
        precision_land()

    except Exception as e:
        print(f"Terjadi error pada misi: {e}")

    finally:
        # Pastikan drone mendarat dan koneksi ditutup
        print("Misi selesai atau dibatalkan. Mendaratkan drone...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        print("Menutup koneksi.")
        vehicle.close()