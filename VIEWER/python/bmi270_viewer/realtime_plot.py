import serial
from collections import deque
import matplotlib.pyplot as plt

# ==== 設定（ここだけ環境に合わせて変更） ====
PORT = "COM9"      # ← 実際のCOMポートに変更
BAUDRATE = 115200  # ← TAG側UARTのボーレートに合わせる
SAMPLE_RATE = 100.0
DURATION_SEC = 3.0
WINDOW = int(SAMPLE_RATE * DURATION_SEC)      # グラフに表示する最新サンプル数
UPDATE_EVERY = 5   # 何サンプルごとにグラフ更新するか
# =========================================


def parse_line(line: str):
    """
    seq,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps
    をパースして (seq, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps) にして返す
    """
    parts = line.split(",")
    if len(parts) != 7:
        raise ValueError(f"unexpected format: {line}")

    seq = int(parts[0])
    ax_mg = int(parts[1])
    ay_mg = int(parts[2])
    az_mg = int(parts[3])
    gx_mdps = int(parts[4])
    gy_mdps = int(parts[5])
    gz_mdps = int(parts[6])

    # mg → g, mdps → dps
    ax_g = ax_mg / 1000.0
    ay_g = ay_mg / 1000.0
    az_g = az_mg / 1000.0
    gx_dps = gx_mdps / 1000.0
    gy_dps = gy_mdps / 1000.0
    gz_dps = gz_mdps / 1000.0

    return seq, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps


def main():
    print(f"Open {PORT} @ {BAUDRATE}bps ...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)

    # データバッファ（最新 WINDOW 個だけ保持）
    seq_buf = deque(maxlen=WINDOW)
    ax_buf = deque(maxlen=WINDOW)
    ay_buf = deque(maxlen=WINDOW)
    az_buf = deque(maxlen=WINDOW)
    gx_buf = deque(maxlen=WINDOW)
    gy_buf = deque(maxlen=WINDOW)
    gz_buf = deque(maxlen=WINDOW)

    # ====== プロット初期化 ======
    plt.ion()  # 対話モード

    # 4段構成：
    # 1段目: 加速度グラフ
    # 2段目: 加速度の数値表示
    # 3段目: ジャイログラフ
    # 4段目: ジャイロの数値表示
    fig, axes = plt.subplots(
        4, 1,
        sharex=True,
        gridspec_kw={"height_ratios": [3, 1, 3, 1]}
    )

    ax_acc, ax_acc_val, ax_gyr, ax_gyr_val = axes

    # --- 加速度グラフ ---
    line_ax, = ax_acc.plot([], [], label="Ax [g]")
    line_ay, = ax_acc.plot([], [], label="Ay [g]")
    line_az, = ax_acc.plot([], [], label="Az [g]")
    ax_acc.set_ylabel("Accelerometer [g]")
    ax_acc.legend(loc="upper right")
    ax_acc.grid(True)

    # --- 加速度の数値表示用 ---
    ax_acc_val.axis("off")  # 軸は消す
    acc_text = ax_acc_val.text(
        0.01, 0.5,
        "",
        transform=ax_acc_val.transAxes,
        va="center",
        fontsize=10
    )

    # --- ジャイログラフ ---
    line_gx, = ax_gyr.plot([], [], label="Gx [dps]")
    line_gy, = ax_gyr.plot([], [], label="Gy [dps]")
    line_gz, = ax_gyr.plot([], [], label="Gz [dps]")
    ax_gyr.set_ylabel("Gyro [dps]")
    ax_gyr.legend(loc="upper right")
    ax_gyr.grid(True)

    # --- ジャイロの数値表示用 ---
    ax_gyr_val.axis("off")
    gyr_text = ax_gyr_val.text(
        0.01, 0.5,
        "",
        transform=ax_gyr_val.transAxes,
        va="center",
        fontsize=10
    )

    # X軸ラベルは一番下にだけ出す
    ax_gyr_val.set_xlabel("Time [s] (last 3 sec)")

    fig.tight_layout()
    plt.show()

    sample_count = 0

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            try:
                seq, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps = parse_line(
                    line)
            except Exception as e:
                print(f"[WARN] {e}")
                continue

            # バッファに追加
            seq_buf.append(seq)
            ax_buf.append(ax_g)
            ay_buf.append(ay_g)
            az_buf.append(az_g)
            gx_buf.append(gx_dps)
            gy_buf.append(gy_dps)
            gz_buf.append(gz_dps)

            sample_count += 1

            # 一定サンプルごとにグラフ更新

            if sample_count % UPDATE_EVERY == 0:
                n = len(ax_buf)
                if n >= 2:
                    # 時間軸 [秒] を作成：0 ～ 約3秒
                    xs = [i / SAMPLE_RATE for i in range(n)]

                    # グラフにデータをセット
                    line_ax.set_data(xs, list(ax_buf))
                    line_ay.set_data(xs, list(ay_buf))
                    line_az.set_data(xs, list(az_buf))

                    line_gx.set_data(xs, list(gx_buf))
                    line_gy.set_data(xs, list(gy_buf))
                    line_gz.set_data(xs, list(gz_buf))

                    # X軸は 0〜DURATION_SEC に固定
                    ax_acc.set_xlim(0.0, DURATION_SEC)
                    ax_gyr.set_xlim(0.0, DURATION_SEC)

                    # Y軸は固定（必要に応じて調整）
                    # 加速度：±3g
                    ax_acc.set_ylim(-10.0, 10.0)
                    # ジャイロ：±600 dps
                    ax_gyr.set_ylim(-6.0, 6.0)

                    # 数値表示を更新（最新値をそのまま使う）
                    acc_text.set_text(
                        f"Ax={ax_g:.3f} g, Ay={ay_g:.3f} g, Az={az_g:.3f} g"
                    )
                    gyr_text.set_text(
                        f"Gx={gx_dps:.1f} dps, Gy={gy_dps:.1f} dps, Gz={gz_dps:.1f} dps"
                    )

                    fig.canvas.draw()
                    fig.canvas.flush_events()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        ser.close()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
