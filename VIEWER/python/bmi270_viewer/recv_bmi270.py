import serial

PORT = "COM9" # 実際のポート名に変更 (例: "COM3", "COM6" など)
BAUDRATE = 115200 # nRF5340 側の UART 設定に合わせる
# ==============================

def main():
    print(f"Open {PORT} @ {BAUDRATE}bps ...")
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            
            # seq,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps
            try:
                parts = line.split(",")
                if len(parts) != 7:
                    print(f"[WARN] unexpected format: {line}")
                    continue

                seq = int(parts[0])
                ax_mg = int(parts[1])
                ay_mg = int(parts[2])
                az_mg = int(parts[3])
                gx_mdps = int(parts[4])
                gy_mdps = int(parts[5])
                gz_mdps = int(parts[6])

                # g, dps に戻したければ /1000.0
                ax_g = ax_mg / 1000.0
                ay_g = ay_mg / 1000.0
                az_g = az_mg / 1000.0
                gx_dps = gx_mdps / 1000.0
                gy_dps = gy_mdps / 1000.0
                gz_dps = gz_mdps / 1000.0

                print(
                    f"seq={seq} | "
                    f"acc[g]=({ax_g:.3f}, {ay_g:.3f}, {az_g:.3f}) "
                    f"gyro[dps]=({gx_dps:.3f}, {gy_dps:.3f}, {gz_dps:.3f})"
                )

            except Exception as e:
                print(f"[ERR] parse error: {e} | raw=&apos;{line}&apos;")

if __name__ == "__main__":
    main()
