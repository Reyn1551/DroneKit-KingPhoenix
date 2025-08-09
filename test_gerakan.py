from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Gunakan koneksi ke simulator
CONNECTION_STRING = 'udp:127.0.0.1:14550'

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """Mengirim perintah kecepatan untuk durasi tertentu."""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    
    print(f"Mengirim kecepatan V_N:{velocity_x}, V_E:{velocity_y}, V_D:{velocity_z} selama {duration} detik")
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


print(f"Menyambungkan ke drone di: {CONNECTION_STRING}")
vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=90)

try:
    print("Memastikan FRAME_CLASS diatur untuk SITL...")
    vehicle.parameters['FRAME_CLASS'] = 1
    time.sleep(1)

    print("Memulai prosedur arming...")
    while not vehicle.is_armable:
        print(" Menunggu drone siap untuk di-arm...")
        time.sleep(1)
    
    print("Drone siap. Mengatur mode GUIDED.")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Menunggu konfirmasi arming...")
        time.sleep(1)

    print("BERHASIL ARM. Takeoff ke 5 meter.")
    vehicle.simple_takeoff(5)

    # Tunggu hingga ketinggian tercapai
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Ketinggian: {alt:.2f}m")
        if alt >= 5 * 0.95:
            break
        time.sleep(1)
    print("Ketinggian takeoff tercapai.")

    # Tes 1: Terbang maju (Utara) selama 3 detik
    send_ned_velocity(vehicle, 1, 0, 0, 3) # V_N=1m/s

    # Tes 2: Berhenti sejenak (hover)
    print("Berhenti sejenak (hover) selama 3 detik")
    send_ned_velocity(vehicle, 0, 0, 0, 3)

    # Tes 3: Terbang ke kanan (Timur) selama 3 detik
    send_ned_velocity(vehicle, 0, 1, 0, 3) # V_E=1m/s

    print("Tes gerakan selesai. Mendarat...")
    vehicle.mode = VehicleMode("LAND")

    # Tunggu hingga mendarat
    while vehicle.armed:
        print(f"Menunggu disarm... Ketinggian: {vehicle.location.global_relative_frame.alt:.2f}m")
        time.sleep(1)
    
    print("Drone telah mendarat dan disarm.")

except Exception as e:
    print(f"Terjadi error: {e}")

finally:
    print("Menutup koneksi.")
    vehicle.close()
