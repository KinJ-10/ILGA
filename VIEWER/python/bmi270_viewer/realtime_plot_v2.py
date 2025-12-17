import serial
from collections import deque
import matplotlib.pyplot as plt

# ==== 設定（ここだけ環境に合わせて変更） ====
PORT = "COM9"      # ← 実際のCOMポートに変更
BAUDRATE = 115200  # ← TAG側UARTのボーレートに合わせる

SAMPLE_RATE = 100.0       # サンプリング周波数 [Hz]
DURATION_SEC = 3.0        # 画面に出す時間幅 [秒]
WINDOW = int(SAMPLE_RATE * DURATION_SEC)  # 保持するサンプル数（約3秒分）

UPDATE_EVERY = 5   # 何サンプルごとにグラフ更新するか
G_CONST = 9.80665  # 1g あたりの加速度 [m/s^2]
# =========================================


def parse_line(line: str):
    """
    1行をパースして、加速度を m/s^2 と g の両方で返す。
    フォーマット:
        seq,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw

    前提:
        ax_raw などは "milli (m/s^2)" として送られているものとする。
        → 1000 で割ると m/s^2 になる。
        ジャイロについては "milli (deg/s)" と仮定し、1000 で割って deg/s にする。
    """
    parts = line.split(",")
    if len(parts) != 7:
        raise ValueError(f"unexpected format: {line}")

    seq = int(parts[0])

    # 加速度（milli m/s^2 → m/s^2 → g）
    ax_raw = int(parts[1])
    ay_raw = int(parts[2])
    az_raw = int(parts[3])

    ax_ms2 = ax_raw / 1000.0
    ay_ms2 = ay_raw / 1000.0
    az_ms2 = az_raw / 1000.0

    ax_g = ax_ms2 / G_CONST
    ay_g = ay_ms2 / G_CONST
    az_g = az_ms2 / G_CONST

    # ジャイロ（milli deg/s → deg/s と仮定）
    gx_raw = int(parts[4])
    gy_raw = int(parts[5])
    gz_raw = int(parts[6])

    gx_dps = gx_raw / 1000.0
    gy_dps = gy_raw / 1000.0
    gz_dps = gz_raw / 1000.0

    return (
        seq,
        ax_g, ay_g, az_g,
        ax_ms2, ay_ms2, az_ms2,
        gx_dps, gy_dps, gz_dps,
    )


def main():
    print(f"Open {PORT} @ {BAUDRATE}bps ...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)

    # データバッファ（最新 WINDOW 個だけ保持）
    seq_buf = deque(maxlen=WINDOW)
    ax_buf = deque(maxlen=WINDOW)  # [g]
    ay_buf = deque(maxlen=WINDOW)  # [g]
    az_buf = deque(maxlen=WINDOW)  # [g]
    gx_buf = deque(maxlen=WINDOW)  # [deg/s]
    gy_buf = deque(maxlen=WINDOW)  # [deg/s]
    gz_buf = deque(maxlen=WINDOW)  # [deg/s]

    # ====== プロット初期化 ======
    plt.ion()  # 対話モード

    # 4段構成：
    # 1段目: 加速度グラフ
    # 2段目: 加速度の数値表示 (g と m/s^2)
    # 3段目: ジャイログラフ
    # 4段目: ジャイロの数値表示 (deg/s)
    fig, axes = plt.subplots(
        4, 1,
        sharex=True,
        gridspec_kw={"height_ratios": [3, 1, 3, 1]}
    )

    ax_acc, ax_acc_val, ax_gyr, ax_gyr_val = axes

    # --- 加速度グラフ (単位: g) ---
    line_ax, = ax_acc.plot([], [], label="Ax [g]")
    line_ay, = ax_acc.plot([], [], label="Ay [g]")
    line_az, = ax_acc.plot([], [], label="Az [g]")
    ax_acc.set_ylabel("Accelerometer [g]")
    ax_acc.legend(loc="upper right")
    ax_acc.grid(True)

    # --- 加速度の数値表示用 (g と m/s^2) ---
    ax_acc_val.axis("off")  # 軸は消す
    acc_text = ax_acc_val.text(
        0.01, 0.5,
        "",
        transform=ax_acc_val.transAxes,
        va="center",
        fontsize=10
    )

    # --- ジャイログラフ (単位: deg/s) ---
    line_gx, = ax_gyr.plot([], [], label="Gx [deg/s]")
    line_gy, = ax_gyr.plot([], [], label="Gy [deg/s]")
    line_gz, = ax_gyr.plot([], [], label="Gz [deg/s]")
    ax_gyr.set_ylabel("Gyro [deg/s]")
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

    # 最新値の保持（数値表示用）
    last_ax_g = last_ay_g = last_az_g = 0.0
    last_ax_ms2 = last_ay_ms2 = last_az_ms2 = 0.0
    last_gx_dps = last_gy_dps = last_gz_dps = 0.0

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            try:
                (
                    seq,
                    ax_g, ay_g, az_g,
                    ax_ms2, ay_ms2, az_ms2,
                    gx_dps, gy_dps, gz_dps,
                ) = parse_line(line)
            except Exception as e:
                print(f"[WARN] {e}")
                continue

            # バッファに追加（グラフ用は g / deg/s）
            seq_buf.append(seq)
            ax_buf.append(ax_g)
            ay_buf.append(ay_g)
            az_buf.append(az_g)
            gx_buf.append(gx_dps)
            gy_buf.append(gy_dps)
            gz_buf.append(gz_dps)

            # 最新値を更新（数値表示用）
            last_ax_g, last_ay_g, last_az_g = ax_g, ay_g, az_g
            last_ax_ms2, last_ay_ms2, last_az_ms2 = ax_ms2, ay_ms2, az_ms2
            last_gx_dps, last_gy_dps, last_gz_dps = gx_dps, gy_dps, gz_dps

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

                    # Y軸は g ベースで固定（必要に応じて調整）
                    ax_acc.set_ylim(-3.0, 3.0)        # 加速度：±3 g
                    ax_gyr.set_ylim(-200.0, 200.0)    # ジャイロ：±200 deg/s

                    # 数値表示を更新
                    acc_text.set_text(
                        f"Ax={last_ax_g:.3f} g ,       Ay={last_ay_g:.3f} g ,       Az={last_az_g:.3f} g \n"
                        f"Ax={last_ax_ms2:.2f} m/s^2, Ay={last_ay_ms2:.2f} m/s^2, Az={last_az_ms2:.2f} m/s^2"
                    )
                    gyr_text.set_text(
                        f"Gx={last_gx_dps:.1f} deg/s, "
                        f"Gy={last_gy_dps:.1f} deg/s, "
                        f"Gz={last_gz_dps:.1f} deg/s"
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
