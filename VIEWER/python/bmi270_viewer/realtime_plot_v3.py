import serial
from collections import deque
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

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


def classify_orientation(ax_g: float, ay_g: float, az_g: float) -> str:
    """
    現在の加速度から、おおざっぱな向きをラベル化する。
    ここでは「どの軸方向に重力が強くかかっているか」で判定する。
    """
    mag = math.sqrt(ax_g ** 2 + ay_g ** 2 + az_g ** 2)

    # 重力がほとんどかかっていない → 自由落下や強い振動など
    if mag < 0.2:
        return f"|a|={mag:.2f} g (low accel / free-fall?)"

    # どの成分が一番大きいか
    comps = {
        "+X": ax_g,
        "-X": -ax_g,
        "+Y": ay_g,
        "-Y": -ay_g,
        "+Z": az_g,
        "-Z": -az_g,
    }
    main_dir = max(comps, key=lambda k: abs(comps[k]))
    main_val = comps[main_dir]

    # ある程度以上支配的な軸がない場合は「斜め」扱い
    if abs(main_val) < 0.5:
        return f"|a|={mag:.2f} g, tilted (no dominant axis)"

    # 軸方向からざっくりした向きをコメント
    notes = {
        "+Z": "Z+ up (仮に '表面が上' とみなす)",
        "-Z": "Z- up (仮に '裏面が上' とみなす)",
        "+X": "X+ up (X+側が上向き)",
        "-X": "X- up (X-側が上向き)",
        "+Y": "Y+ up (Y+側が上向き)",
        "-Y": "Y- up (Y-側が上向き)",
    }
    note = notes.get(main_dir, "")

    return f"|a|={mag:.2f} g, main={main_dir} ({note})"


# ==== 3Dボード描画用ユーティリティ ====

def board_vertices_local():
    """
    ローカル座標系でボード（SparkFunボードっぽい長方形）の頂点を定義。
    中心を原点とし、XY平面に寝ていて +Z が表面側という想定。
    実寸ではなく、見やすい比率にしている。
    """
    width = 2.0   # X方向
    height = 1.2  # Y方向
    thickness = 0.1  # Z方向（薄い板）

    w = width / 2.0
    h = height / 2.0
    t = thickness / 2.0

    # 8頂点 (x, y, z)
    v = [
        [-w, -h, -t],
        [-w,  h, -t],
        [w,  h, -t],
        [w, -h, -t],
        [-w, -h,  t],
        [-w,  h,  t],
        [w,  h,  t],
        [w, -h,  t],
    ]
    return v


def rotate_vertices(vertices, roll, pitch):
    """
    ロール・ピッチ角（ラジアン）を使ってボード頂点を回転。
    ヨー角は 0 と仮定（絶対方位は加速度だけではわからないため）。
    回転順序は R = Ry(pitch) * Rx(roll) とする。
    """
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)

    # X軸回転（ロール）
    Rx = [
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr],
    ]

    # Y軸回転（ピッチ）
    Ry = [
        [cp, 0, sp],
        [0,  1, 0],
        [-sp, 0, cp],
    ]

    # 行列の積 Ry * Rx
    R = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            R[i][j] = sum(Ry[i][k] * Rx[k][j] for k in range(3))

    # 頂点を回転
    rotated = []
    for x, y, z in vertices:
        xr = R[0][0]*x + R[0][1]*y + R[0][2]*z
        yr = R[1][0]*x + R[1][1]*y + R[1][2]*z
        zr = R[2][0]*x + R[2][1]*y + R[2][2]*z
        rotated.append([xr, yr, zr])

    return rotated


def accel_to_roll_pitch(ax_g: float, ay_g: float, az_g: float):
    """
    加速度（g）から簡易的なロール・ピッチ角を計算。
    - ロール: X軸まわり
    - ピッチ: Y軸まわり
    ヨー角は不定なので 0 とみなす。
    """
    # 加速度方向ベクトルの長さが小さすぎるときは 0 にしておく
    mag = math.sqrt(ax_g**2 + ay_g**2 + az_g**2)
    if mag < 0.1:
        return 0.0, 0.0

    # よく使われる簡易式（静止 or 低加速前提）
    roll = math.atan2(ay_g, az_g)
    pitch = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2))
    return roll, pitch


def setup_time_series_fig():
    """時系列グラフ用の Figure を準備して返す。"""
    plt.ion()

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
    ax_acc_val.axis("off")
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

    ax_gyr_val.set_xlabel("Time [s] (last 3 sec)")

    fig.tight_layout()
    fig.show()

    return (
        fig,
        ax_acc, ax_acc_val, ax_gyr, ax_gyr_val,
        line_ax, line_ay, line_az,
        line_gx, line_gy, line_gz,
        acc_text, gyr_text,
    )


def setup_3d_fig():
    """加速度ベクトル用の 3D Figure を準備して返す。"""
    fig3d = plt.figure()
    ax3d = fig3d.add_subplot(111, projection="3d")  # type: ignore

    ax3d.set_xlabel("Ax [g]")
    ax3d.set_ylabel("Ay [g]")
    ax3d.set_zlabel("Az [g]")

    axis_lim = 1.5  # ±1.5 g 程度を想定
    ax3d.set_xlim(-axis_lim, axis_lim)
    ax3d.set_ylim(-axis_lim, axis_lim)
    ax3d.set_zlim(-axis_lim, axis_lim)

    ax3d.set_title("Board orientation (3D)")

    fig3d.tight_layout()
    fig3d.show()

    return fig3d, ax3d


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

    # 図の準備（時系列 & 3D）
    (
        fig,
        ax_acc, ax_acc_val, ax_gyr, ax_gyr_val,
        line_ax, line_ay, line_az,
        line_gx, line_gy, line_gz,
        acc_text, gyr_text,
    ) = setup_time_series_fig()

    fig3d, ax3d = setup_3d_fig()
    board_local = board_vertices_local()

    sample_count = 0

    # 最新値の保持（数値表示 & 3D 用）
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

            # 最新値を更新（数値表示用 & 3D 表示用）
            last_ax_g, last_ay_g, last_az_g = ax_g, ay_g, az_g
            last_ax_ms2, last_ay_ms2, last_az_ms2 = ax_ms2, ay_ms2, az_ms2
            last_gx_dps, last_gy_dps, last_gz_dps = gx_dps, gy_dps, gz_dps

            sample_count += 1

            # 一定サンプルごとにグラフ更新
            if sample_count % UPDATE_EVERY == 0:
                n = len(ax_buf)
                if n >= 2:
                    # ========= 時系列グラフ更新 =========
                    xs = [i / SAMPLE_RATE for i in range(n)]  # 時間軸 [sec]

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

                    # Y軸は g / deg/s ベースで固定
                    ax_acc.set_ylim(-3.0, 3.0)        # 加速度：±3 g
                    ax_gyr.set_ylim(-200.0, 200.0)    # ジャイロ：±200 deg/s 程度に調整

                    # 数値表示を更新（g と m/s^2）
                    acc_text.set_text(
                        f"Ax={last_ax_g:.3f} g, Ay={last_ay_g:.3f} g, Az={last_az_g:.3f} g\n"
                        f"Ax={last_ax_ms2:.2f} m/s^2, Ay={last_ay_ms2:.2f} m/s^2, Az={last_az_ms2:.2f} m/s^2"
                    )
                    gyr_text.set_text(
                        f"Gx={last_gx_dps:.1f} deg/s, "
                        f"Gy={last_gy_dps:.1f} deg/s, "
                        f"Gz={last_gz_dps:.1f} deg/s"
                    )

                    fig.canvas.draw()
                    fig.canvas.flush_events()

                    # ========= 3D 表示更新（ボードオブジェクト） =========
                    ax3d.cla()
                    ax3d.set_xlabel("Ax [g]")
                    ax3d.set_ylabel("Ay [g]")
                    ax3d.set_zlabel("Az [g]")
                    axis_lim = 1.5
                    ax3d.set_xlim(-axis_lim, axis_lim)
                    ax3d.set_ylim(-axis_lim, axis_lim)
                    ax3d.set_zlim(-axis_lim, axis_lim)

                    orient_label = classify_orientation(
                        last_ax_g, last_ay_g, last_az_g)
                    ax3d.set_title(f"Board orientation (3D)\n{orient_label}")

                    # 加速度からロール・ピッチを求めてボードを回転
                    roll, pitch = accel_to_roll_pitch(
                        last_ax_g, last_ay_g, last_az_g)
                    verts_world = rotate_vertices(board_local, roll, pitch)

                    # 8頂点 → 面（ポリゴン）定義
                    # インデックス: 0..3 = bottom, 4..7 = top
                    faces_idx = [
                        [0, 1, 2, 3],  # 底面
                        [4, 5, 6, 7],  # 上面
                        [0, 1, 5, 4],  # 側面
                        [1, 2, 6, 5],
                        [2, 3, 7, 6],
                        [3, 0, 4, 7],
                    ]
                    faces = [[verts_world[i] for i in face]
                             for face in faces_idx]

                    poly = Poly3DCollection(faces, alpha=0.5)
                    poly.set_edgecolor("k")  # 枠線だけ黒
                    ax3d.add_collection3d(poly)

                    fig3d.canvas.draw()
                    fig3d.canvas.flush_events()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        ser.close()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
