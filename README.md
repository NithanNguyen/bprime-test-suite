# Test Suite: Isaac Sim 4.1 ↔ ROS 2 Foxy (container)

Bộ kiểm thử cấu hình mô phỏng cho đề tài robot Object Goal Navigation hỗ trợ người cao tuổi. Câu hỏi cần trả lời:

> Trên một máy cụ thể, Isaac Sim 4.1 (dùng thư viện ROS 2 Foxy **nội bộ**) có trao đổi đủ dữ liệu với một container ROS 2 Foxy không, kể cả khi robot nằm trong một căn hộ ảo?

Dữ liệu cần trao đổi: đồng hồ mô phỏng, lệnh vận tốc, camera RGB, LiDAR 2D (RPLIDAR S2E), odometry và TF.

## 1. Kiến trúc

```
Host Ubuntu + Docker + NVIDIA Container Toolkit
├─ Container Isaac Sim 4.1 (nvcr.io/nvidia/isaac-sim:4.1.0, headless)
│    bridge dùng foxy/lib nội bộ  → phát /clock /scan /odom /tf /bprime/rgb, nhận /cmd_vel
└─ Container ROS 2 Foxy (osrf/ros:foxy-desktop) → pipeline của đề tài, use_sim_time:=true
   ↕ Fast DDS chỉ UDP (config/fastdds_udp.xml) · --network host · ROS_DOMAIN_ID=42
```

Foxy nói chuyện với Foxy, nên không có cầu nối chéo phiên bản. Bộ test dùng robot **Jetbot** (vi sai) thay cho robot thật của đề tài. **Không** kiểm tra: Nav2/SLAM, mô hình robot thật, phần cứng thật (Jetson, LiDAR).

## 2. Trạng thái

| Máy | T0–T6 (hạ tầng) | T7 Office | T7 căn hộ GRScenes |
|---|---|---|---|
| Laptop: i7-12700H, RTX 3050 Laptop **4 GB**, Ubuntu 20.04, driver 570 | PASS | PASS | **Không chạy được: thiếu VRAM** (mục 7.3) |
| PC: RTX 3060 **12 GB**, Ubuntu 22.04, driver 580 | chưa chạy | chưa chạy | chưa chạy |

**Kết luận hiện tại:** laptop dùng cho T0–T6, Office và phát triển node ROS 2. Mô phỏng trong căn hộ GRScenes phải chạy trên PC; Isaac Sim 4.1 yêu cầu tối thiểu 8 GB VRAM [R1].

**Bước tiếp theo (PC):** mục 4.1 → T0–T6 → T7 Office → mục 6 (GRScenes) → T7 cho các cảnh ứng viên, giữ 2 cảnh PASS.

## 3. Yêu cầu

- Ubuntu 20.04 hoặc 22.04; GPU NVIDIA RTX; driver ≥ 535.129.03 [R1].
- **VRAM ≥ 8 GB** cho cảnh GRScenes. Với GPU < 8 GB, `isaac t7` in cảnh báo trước khi chạy cảnh không phải Office.
- Docker Engine và NVIDIA Container Toolkit (`install-docker`, `install-toolkit` cài giúp nếu chưa có).
- Ổ trống: ≥ 30 GB cho hai image (17,4 GB + 4,2 GB, đo bằng `docker images`) và cache; thêm dung lượng cho GRScenes (mục 6).
- Mạng tới `nvcr.io`, Docker Hub, máy chủ asset S3 của NVIDIA và Hugging Face.
- Mỗi lúc chỉ chạy **một** container Isaac Sim.

## 4. Cách dùng

### 4.1 Chuẩn bị (mỗi máy một lần)

```bash
git clone https://github.com/NithanNguyen/bprime-test-suite.git && cd bprime-test-suite
./bprime.sh preflight      # chỉ đọc thông tin máy
./bprime.sh gpu-check      # T0 (máy chưa có Docker: install-docker → đăng xuất/đăng nhập → install-toolkit)
./bprime.sh pull           # T1a: tải image, ghi digest. Lỗi 'unauthorized': docker login nvcr.io
./bprime.sh inspect        # T2: image có thư viện Foxy nội bộ
```

### 4.2 Chạy test

Mở hai terminal ở thư mục repo. Terminal B tự chờ terminal A sẵn sàng, nên có thể chạy ngay sau A.

| Test | Terminal A (Isaac) | Terminal B (Foxy) |
|---|---|---|
| Base: BACKEND, T3–T5 | `./bprime.sh isaac base` | `./bprime.sh foxy base` |
| T6: LiDAR, robot, odom, TF | `./bprime.sh isaac t6` | `./bprime.sh foxy t6` |
| T7 Office | `./bprime.sh isaac t7 --scene office --add-colliders --floor-z 0 --duration 180` | `./bprime.sh foxy t7` |
| T7 căn hộ | `./bprime.sh isaac t7 --scene <ID>` | `./bprime.sh foxy t7` |

- Bấm **Ctrl+C** ở terminal A khi xong một test. Lần đầu chạy trên mỗi máy phải biên dịch shader (vài phút); lần đầu mở một cảnh GRScenes có thể mất 15–20 phút [R6].
- `<ID>` là ID thật của cảnh, ví dụ `MV7J6NIKTKJZ2AABAAAAADQ8` (thư mục `<ID>_usd`).
- Sau mỗi lần chạy: `./bprime.sh collect` (đóng gói log vào `~/bprime_report_<host>_<time>.tar.gz`). Mỗi test ghi đè log của lần trước. `./bprime.sh stop` xóa các container test.

### 4.3 Tùy chọn chính

| Lệnh | Tùy chọn |
|---|---|
| `isaac base` / `t6` | `--duration S` (300), `--no-camera`, `--lidar-config NAME` (`RPLIDAR_S2E`), `--livestream` |
| `isaac t7` | `--scene office\|<ID>\|<file.usd>`, `--variant navigation\|raw`, `--duration S`, `--spawn X Y`, `--floor-z Z`, `--search-region XMIN YMIN XMAX YMAX`, `--add-colliders`, `--ground-plane`, `--hide-hdr`, `--add-dome-light`, `--no-camera`, `--cam-res WxH` (640x480), `--livestream` |
| `foxy t7` | `--no-camera` |
| `shell isaac\|foxy` | Shell trong container, môi trường đã thiết lập |

### 4.4 Xem mô phỏng (livestream WebRTC)

Thêm `--livestream --duration 1800` vào lệnh của terminal A. Khi log in `simulation playing`, mở bằng Chrome/Chromium `http://127.0.0.1:8211/streaming/webrtc-client?server=127.0.0.1` và bấm nút Play đỏ [R2]. Chỉ dùng để **xem**: render UI làm mô phỏng chậm hơn (T6 trên laptop: RTF 0,47–0,59 → 0,39), và mỗi instance chỉ phục vụ một người xem.

Lái robot bằng tay từ host (cùng domain và transport UDP với container):

```bash
source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=~/bprime-test-suite/config/fastdds_udp.xml
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 5. Danh mục test và tiêu chí đạt

### 5.1 Hạ tầng (T0–T6)

| ID | Kiểm tra | Log | Đạt khi |
|---|---|---|---|
| T0 / T1a / T2 | GPU trong container / tải image / có thư viện Foxy nội bộ | `01_*`, `02_*`, `10_*` | GPU được liệt kê / có digest / `T2 PASS` |
| BACKEND | Isaac nạp backend **Foxy**, không phải Humble | `20_isaac_base.log` | `BACKEND PASS` |
| T3 / T4 | `/clock` Isaac → Foxy / `/cmd_vel` Foxy → Isaac | `21_*`, `20_*` | Timestamp tăng đều 1/60 s / `TWIST PASS` |
| T5 | Camera `/bprime/rgb` + `camera_info` | `21_foxy_base.log` | Tần số > 0; 1280×720 |
| T6a–c | `/scan`, `/odom`, `/tf` | `31_foxy_t6.log` | ≥ 80 % tia hợp lệ; đủ `odom→base_link→lidar_link` |
| T6d / T6e | Chạy thẳng 0,15 m/s / quay 0,5 rad/s | `31_foxy_t6.log` | Tỉ số đo/kỳ vọng 0,8–1,2 (0,5–1,5 = WARN) |

B′ **dùng được** trên một máy khi T0, T1a, T2, BACKEND, T3, T4 và T6a–T6e đều PASS. Nếu BACKEND FAIL, các kết quả DDS sau đó không còn giá trị cho B′.

T6d/T6e tính kỳ vọng theo thời gian **mô phỏng** (`/clock`), nên không phụ thuộc tốc độ máy. `/odom` là vị trí thật của chassis (`IsaacComputeOdometry`), nên phép đo kiểm tra trọn chuỗi Foxy → bridge → bộ điều khiển vi sai → vật lý → `/odom`.

### 5.2 Robot trong cảnh (T7)

| ID | Kiểm tra | Đạt khi |
|---|---|---|
| T7a LOAD | Mở và nạp xong cảnh | PASS khi đơn vị là mét, trục Z lên. Các cảnh GRScenes đã thống kê dùng cm (`metersPerUnit=0.01`) nên báo WARN; script quy đổi đơn vị, T7b/T7h xác nhận tỉ lệ |
| T7f SEMANTICS | Có nhãn ngữ nghĩa; xuất `t7_objects_<cảnh>.json` (ground truth cho Success/SPL) | Có ít nhất một vật thể có nhãn (Office: N/A) |
| SPAWN | Tìm ô sàn trống trong nhà bằng raycast PhysX | Tìm được ô, hoặc dùng `--spawn X Y` |
| T7b STABLE | Robot đứng yên trên sàn, 2–5 s mô phỏng | Chassis cao −0,05 đến 0,5 m so với sàn, dao động ≤ 5 cm, trôi ngang ≤ 0,2 m |
| T7c | `/scan`, `/tf` trong cảnh | ≥ 80 % tia hợp lệ; đủ khung TF |
| T7d VRAM | Đỉnh VRAM toàn GPU (host lấy mẫu 1 Hz) | ≤ 90 % VRAM **và** log không có `ERROR_OUT_OF_DEVICE_MEMORY` |
| T7e RTF | Thời gian mô phỏng / thời gian thực | Chỉ tham khảo |
| T7g CAMERA | `/bprime/rgb` | Tần số > 0; độ sáng trung bình 15–240; độ lệch chuẩn ≥ 5 |
| T7h | Quay 0,5 rad/s; chạy thẳng 0,15 m/s nếu phía trước trống ≥ 0,8 m | Tỉ số 0,8–1,2 |

**Một cảnh dùng được** khi T7a, SPAWN, T7b, T7c, T7d, T7g và T7h ROTATE đều PASS (riêng T7a: WARN do đơn vị cm được chấp nhận). Các ngưỡng T7 là đề xuất của bộ test, không lấy từ tài liệu NVIDIA.

## 6. GRScenes-100: chọn và giải nén căn hộ

Bộ cảnh nhà của GRScenes-100 [R4] gồm 69 căn hộ, giấy phép CC BY-NC-SA 4.0. Trước khi tải phải điền User Agreement (link trong README của InternUtopia [R5]).

### 6.1 Số liệu đo được

| Đại lượng | Giá trị |
|---|---|
| Archive tải về (3 phần `home_scenes.zip/.z01/.z02`) | 50,95 GB (8,0 + 21,5 + 21,5) |
| Sau giải nén toàn bộ | **110,96 GB**: `models/` 91,38 · `Materials/` 19,32 · `scenes/` 0,26 |
| Model mà **một** cảnh tham chiếu | 0,57–2,45 GB |
| Mỗi thư mục `scenes/<ID>_usd/` | 3 file USD nhị phân (USDC): `start_result_{raw,navigation,interaction}.usd`, `metadata.json` (danh sách model), symlink `models → ../../models`, `Materials → ../../Materials` |

Cộng dung lượng model của từng cảnh được 91,92 GB, chỉ hơn hợp của chúng (91,37 GB) 0,6 %. Nghĩa là các cảnh gần như không dùng chung model, nên **chỉ cần giải nén các cảnh sẽ dùng**: `Materials/` cộng 8 cảnh tốn ≈ 32 GB, so với 111 GB nếu giải nén hết.

### 6.2 Quy trình

```bash
# 1) Tải (≈ 51 GB). Chạy trong tmux; nếu bị ngắt, chạy lại đúng lệnh để tải tiếp
sudo apt install -y zip unzip python3-venv tmux
python3 -m venv ~/hfenv && ~/hfenv/bin/pip install -U huggingface_hub
~/hfenv/bin/python - <<'PY'
from huggingface_hub import snapshot_download
snapshot_download(repo_id="InternRobotics/GRScenes", repo_type="dataset",
                  allow_patterns=["scenes/GRScenes-100/home_scenes.*"],
                  local_dir="/data/datasets/GRScenes")      # thay bằng ổ của bạn
PY

# 2) Gộp archive chia nhỏ, kiểm tra toàn vẹn, rồi mới xóa 3 phần gốc
cd /data/datasets/GRScenes/scenes/GRScenes-100
zip -s 0 home_scenes.zip --out home_scenes_full.zip
unzip -tq home_scenes_full.zip                   # phải in "No errors detected"
rm home_scenes.zip home_scenes.z01 home_scenes.z02

# 3) Xếp hạng 69 cảnh (chỉ đọc zip, không giải nén)
cd ~/bprime-test-suite
./bprime.sh grscenes-select plan --zip /data/datasets/GRScenes/scenes/GRScenes-100/home_scenes_full.zip

# 4) Giải nén Materials/ + các cảnh đã chọn + đúng các model chúng dùng (có kiểm tra ổ trống)
#    <ID1> <ID2> ...: thay bằng ID thật trong bảng của bước 3 (ví dụ mục 6.3)
./bprime.sh grscenes-select extract --zip /data/datasets/GRScenes/scenes/GRScenes-100/home_scenes_full.zip \
    --dest /data/datasets/GRScenes/home --ids <ID1> <ID2> ...
echo 'export GRSCENES_ROOT=/data/datasets/GRScenes/home/home_scenes' >> ~/.bashrc && source ~/.bashrc

# 5) Diện tích sàn và loại vật thể (đọc USD trong container Isaac, không mô phỏng)
./bprime.sh grscenes-inventory --ids <ID1> <ID2> ...
```

Giải thích:
- **Bước 2:** `unzip` không đọc trực tiếp archive chia nhỏ, nên phải gộp bằng `zip -s 0` [R8].
- **Bước 3:** `plan` đọc `metadata.json` của từng cảnh ngay trong zip. Với mỗi cảnh, công cụ in dung lượng model, số model và số loại vật thể, cùng các loại "đồ cá nhân" mà cảnh có. Danh sách mặc định gồm 20 loại (cup, bottle, book, shoe, …); đây là đề xuất của bộ test, sửa bằng `--targets`. Kết quả ghi vào `~/bprime_logs/grscenes_plan.csv`.
- **Bước 4:** `extract` dùng `unzip` để giữ nguyên symlink, bỏ qua file đã có, và kiểm tra mọi model được tham chiếu đều mở được qua symlink. Công cụ báo lỗi nếu ID không tồn tại. Giữ `home_scenes_full.zip` cho đến khi chốt xong cảnh, để còn giải nén thêm.
- `GRSCENES_ROOT` là thư mục **chứa** `Materials/ models/ scenes/`. Bộ test mount nó chỉ đọc tại `/grscenes` và đặt `MDL_SYSTEM_PATH=/grscenes/Materials` [R4].

### 6.3 Tám cảnh ứng viên (đã thống kê)

| Cảnh | Sàn* (m²) | Hộp bao sàn (m) | Model | GB model | Đồ cá nhân (/20) |
|---|---|---|---|---|---|
| MV7J6NIKTKJZ2AABAAAAADQ8 | 107,3 | 17,0 × 7,0 | 522 | 1,40 | 13 |
| MVUHLWYKTKJ5EAABAAAAAAQ8 | 114,1 | 11,5 × 12,3 | 753 | 1,09 | 13 |
| MV7J6NIKTKJZ2AABAAAAAEA8 | 116,9 | 13,8 × 10,5 | 644 | 1,61 | 12 |
| MVUHLWYKTKJ5EAABAAAAACI8 | 140,6 | 14,3 × 11,7 | 1125 | 1,71 | 13 |
| MVUHL5YKTKJ5EAABAAAAAAA8 | 140,6 | 14,3 × 11,7 | 1020 | 1,96 | 12 |
| MVUHL5YKTKJ5EAABAAAAAAQ8 | 115,5 | 13,2 × 12,7 | 1009 | 1,62 | 12 |
| MVUHLWYKTKJ5EAABAAAAADQ8 | 109,8 | 15,2 × 12,3 | 1224 | 1,10 | 11 |
| MV7J6NIKTKJZ2AABAAAAADA8 | 206,4 | 17,2 × 12,0 | 1105 | 2,45 | 13 |

\* Ước lượng của `grscenes-inventory`: hợp các mặt nằm ngang của mesh `ground`, raster 0,1 m.

- Cả 8 cảnh đều có `metersPerUnit=0.01`, trục Z lên.
- Hai cảnh `…ACI8` và `MVUHL5…AAA8` có cùng diện tích và cùng hộp bao. Nhiều khả năng đây là cùng một mặt bằng với cách bày đồ khác nhau (chưa kiểm chứng), nên không chọn cả hai làm "2 căn hộ khác nhau".
- **Thứ tự T7 đề xuất trên PC:** `…ADQ8` (ít model nhất), `MVUHLWY…AAQ8`, `…AEA8`; dự phòng `…ACI8` (nhiều đồ nhất). Đây là đề xuất, dựa trên diện tích, số model và số loại đồ cá nhân.
- Nhãn không đồng nhất giữa các cảnh, phải gộp khi tính Success/SPL: `sofa_chair`/`sofachair`, `tv_stand`/`tvstand`, `coffee_maker`/`coffeemaker`, `shoecabinet`/`shoe_cabinet`, `door_<UUID>`.

## 7. Kết quả đo trên laptop (19–20/10/2026)

### 7.1 T0–T6: PASS toàn bộ

| Mục | Kết quả |
|---|---|
| BACKEND | Nạp `libomni.isaac.ros2_bridge.foxy.so` và `foxy/lib/*`, không nạp Humble |
| T3 / T4 | `/clock` tăng đúng 16,667 ms mỗi bước / `TWIST PASS lin=(0.3,0,0) ang=(0,0,0.5)` |
| T5 | RGB ≈ 14 Hz ở 1280×720; fx = 3054,16, cx = 640, cy = 360 |
| T6a | 3200 tia, bước 0,1125°, tầm 0,05–30 m, chu kỳ 0,1 s (khớp datasheet S2E [R7]); 89 % tia hợp lệ |
| T6d / T6e / T6f | Tỉ số 1,12 / 1,03; khoảng cách tới tường giảm 0,488 m so với 0,485 m theo odom |
| RTF | 0,47–0,59 (28–35 bước/s). Mọi node ROS phải đặt `use_sim_time:=true` |

### 7.2 T7 Office: PASS toàn bộ

Lệnh: `--scene office --add-colliders --floor-z 0`. Cả hai tùy chọn đều **bắt buộc**: Office chỉ có một collider dạng Plane, và có một bề mặt ở z = −3,3 m nằm dưới sàn thật.

| T7a | SPAWN | T7b | T7c | T7g | T7h quay / thẳng | T7d | T7e RTF |
|---|---|---|---|---|---|---|---|
| 488,7 s lần đầu, 16–47 s các lần sau | (0,30; 17,30), z = 0 | z = 0,041 m, trôi 0,002 m | 86 %, TF đủ | 640×480, 9,2 Hz | 1,01 / 1,13 | 2821 MiB (69 %) | 0,24 |

### 7.3 T7 GRScenes: không khả thi trên GPU 4 GB

Cảnh `MV7J6NIKTKJZ2AABAAAAADQ8` (ít model nhất trong 8 ứng viên). VRAM nền trước khi chạy là 280–384 MiB (Xorg, gnome-shell; PRIME = nvidia).

| Lần chạy | Kết quả |
|---|---|
| Mặc định | T7a WARN: nạp 100,2 s, 13 757 prim, 4 223 mesh, 516 prim có collider. Sau đó 6355 lỗi `ERROR_OUT_OF_DEVICE_MEMORY` khi cấp phát bộ nhớ cho hình học và cấu trúc tăng tốc tia (`AccelStruct creation failed`) → `GPU crash`. Đỉnh 3775 MiB / 4096 MiB |
| `--no-camera --hide-hdr` | Vẫn 6373 lỗi out-of-memory; PhysX báo không đủ VRAM và chuyển sang CPU; `SPAWN FAIL: no collider hit`. Đỉnh 3778 MiB (92 %) |

**Kết luận (độ tin cậy cao):** chỉ riêng hình học của cảnh cộng RTX LiDAR đã vượt 4 GB. Bỏ camera và HDR không khắc phục được, và VRAM nền (≈ 0,3 GB) quá nhỏ để giải thích mức thiếu hụt. Kết quả này phù hợp với mức tối thiểu 8 GB của NVIDIA [R1].

Hai điểm còn mở, cần kiểm tra lại trên PC:
- `SPAWN FAIL` là hệ quả của việc hết VRAM hay do sàn trong file `navigation` không có collider? Mười collider đầu tiên được in ra đều là tủ.
- Một số mesh có kích thước bất thường (model `pan` dài 261 m), làm vùng tìm spawn rộng 36 × 36 m trong khi sàn chỉ khoảng 17 × 7 m.

Bộ test đã được sửa sau lần chạy này: T7d nay báo **FAIL** khi log có lỗi out-of-memory, vì đỉnh VRAM lấy mẫu không thể vượt dung lượng card và từng hiển thị là "WARN 92 %".

## 8. Hạn chế đã biết

- ≈ 11 % tia `/scan` có giá trị −1,0 dù `/point_cloud` có điểm ở hướng đó; chưa rõ nguyên nhân. Có điểm đo ở ≈ 0,08 m, nhiều khả năng LiDAR quét trúng thân robot (chưa kiểm chứng).
- Jetbot tự quay ≈ −1° mỗi 10 s khi lệnh bằng 0 (T6). Trong T7 Office, yaw trôi ≈ −0,2°/s sau khi dừng; nghi do ma sát bánh caster (PhysX cảnh báo `improvePatchFriction`), chưa kiểm chứng.
- LiDAR `SICK_microscan3_*` cho `angle_increment` sai dấu; không dùng cho đề tài.
- Scan mô phỏng theo quy ước ROS (ngược chiều kim đồng hồ, REP 103 [R7]); S2E thật quay theo chiều kim đồng hồ, cần đối chiếu với driver `rplidar_ros`.
- Camera T7 chưa có TF `camera_link`. Thuật toán tìm spawn mới chỉ chạy thành công trên Office.

## 9. Xử lý sự cố

Mỗi lần chỉ đổi một thứ, rồi chạy lại.

| Triệu chứng | Cách xử lý |
|---|---|
| `docker: permission denied` | Đăng xuất/đăng nhập lại sau `install-docker` |
| `bprime.sh must run on the host` | Đang ở trong shell container (`root@...:/isaac-sim#`): gõ `exit` |
| `container bprime-isaac already exists` | `./bprime.sh stop` |
| Terminal B chờ mãi | A đã in `simulation playing` chưa? Cả hai bên phải in `ROS_DOMAIN_ID=42`; `ip -br addr` phải có card UP ngoài `lo` |
| Terminal B chỉ thấy `clock=True` | Phía Isaac in `ROBOT SKIPPED` (SPAWN FAIL): dừng cả hai bên, sửa SPAWN trước |
| `ERROR_OUT_OF_DEVICE_MEMORY`, `GPU crash`, T7d FAIL | GPU không đủ VRAM cho cảnh: chạy trên GPU ≥ 8 GB (mục 7.3). `--cam-res 424x240`, `--no-camera` chỉ giúp khi vượt ít |
| `SPAWN FAIL: no collider hit` | Office: `--add-colliders`. GRScenes: dùng `--variant navigation` (mặc định); nếu vẫn lỗi, thử `--add-colliders` hoặc `grscenes-prep <ID>` |
| `SPAWN FAIL`, vùng tìm rộng bất thường | `--floor-z Z` và/hoặc `--search-region XMIN YMIN XMAX YMAX`; hoặc chọn điểm bằng `--livestream` rồi `--spawn X Y` |
| T7b FAIL `moved X m ... without a command` | Robot bị đặt bên trong/dưới hình học và bị PhysX đẩy ra: chỉ định `--floor-z` |
| Lỗi `DayMaterial.mdl`, nạp rất lâu | `--hide-hdr` (tắt `HDR_Sphere` trong phiên chạy, theo InternUtopia issue #30 [R5]) |
| T7g ảnh tối | `--add-dome-light` |
| Lỗi vật lý với file `navigation` có sẵn | `./bprime.sh grscenes-prep <ID>`: tạo lại bằng `preprocess.py -n` của InternUtopia (cần clone vào `~/InternUtopia`); bản gốc lưu thành `.shipped.usd` |
| Không mở được trang livestream | `ss -ltnp \| grep 8211` trên host. Không có dòng nào: extension WebRTC chưa chạy (xem log terminal A). Có dòng: thử Chromium, đóng tab xem khác; màn hình đen lần đầu thì chờ 1–2 phút |
| `grscenes-inventory` báo `0 scene folders` | ID sai hoặc là chữ giữ chỗ: dùng ID trong bảng `grscenes-select plan` |
| `python3: can't open file '/bprime/...'` | Container Foxy cũ mount bản repo khác: `docker rm -f bprime-foxy` |
| `BACKEND UNKNOWN/FAIL` | Tìm `ros2_bridge` trong log Kit ở `~/docker/isaac-sim-4.1/logs` |

## 10. Ghi chú kỹ thuật T7

- **Mở cảnh:** `open_stage()` của `omni.isaac.core`, giống `play_scene.py` của InternUtopia; dùng physics scene có sẵn trong file và tắt UJITSO collision cooking. `World(stage_units_in_meters=metersPerUnit)`; mọi vị trí tính theo mét rồi nhân với `1/metersPerUnit`.
- **Spawn** (`common/t7_lib.py`): lưới 0,5 m trên hộp bao của các mesh ≤ 50 m. Một ô hợp lệ khi: tia xuống chạm mặt sàn (độ cao phổ biến nhất); tia lên chạm trần trong 1,5–5 m (trong nhà, không nằm dưới bàn/giường); tia ngang theo 16 hướng, ở hai độ cao 0,10 m và 0,40 m, không chạm vật nào trong 0,35 m. Chọn ô có nhiều ô lân cận hợp lệ nhất. Test đơn vị: `python3 common/test_t7_lib.py`.
- **Camera:** gắn trên chassis (+0,06 m phía trước, +0,12 m lên trên), HFOV 70°, 640×480. Quaternion (w, x, y, z) = (0,5; 0,5; −0,5; −0,5) đổi hướng nhìn −Z của camera USD thành +X của robot.
- **Ground truth:** prim cao nhất mang thuộc tính `semantic:*:params:semanticData`; nhãn GRScenes có dạng `<loại>/<model>`. Tâm và kích thước hộp bao được ghi theo mét.

## 11. Cấu trúc và gỡ bỏ

```
bprime.sh             lệnh duy nhất (./bprime.sh help)
config/               bprime.env (image, domain, thư mục, GRSCENES_ROOT) · fastdds_udp.xml
isaac/                chạy trong container Isaac: test_base/t6/t7.py, grscenes_inventory.py, bprime_app.py
foxy/                 chạy trong container Foxy: check_base.sh, check_t6/t7.py, diag_scan.py
common/               thư viện thuần Python cho T7 (spawn, diện tích sàn, thống kê ảnh) + test đơn vị
host/                 chạy trên host: grscenes_select.py (plan/extract GRScenes)
```

Repo được mount **chỉ đọc** vào cả hai container tại `/bprime`; log ghi ra `~/bprime_logs`. Bộ test không cài hay sửa ROS trên host. Gỡ bỏ:

```bash
./bprime.sh stop && docker image rm nvcr.io/nvidia/isaac-sim:4.1.0 osrf/ros:foxy-desktop
sudo rm -rf ~/docker/isaac-sim-4.1 ~/bprime_logs      # file do container tạo thuộc root
```

## 12. Tài liệu tham khảo

- [R1] Isaac Sim 4.1 – [Requirements](https://docs.isaacsim.omniverse.nvidia.com/4.1.0/installation/requirements.html) (GPU tối thiểu RTX 3070, 8 GB VRAM; driver 535.129.03) · [ROS and ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/4.1.0/installation/install_ros.html) (thư viện Foxy nội bộ, profile Fast DDS)
- [R2] Isaac Sim 4.2 – [Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/manual_livestream_clients.html)
- [R3] Isaac Sim 4.2 – [Container Installation](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/install_container.html), [ROS 2 RTX Lidar](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html), [TF & Odometry](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/ros2_tutorials/tutorial_ros2_tf.html), [Jetbot controller](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/core_api_tutorials/tutorial_core_adding_controller.html) · Isaac Sim 4.1 [Environment Assets](https://docs.isaacsim.omniverse.nvidia.com/4.1.0/features/environment_setup/assets/usd_assets_environments.html)
- [R4] [GRScenes dataset card](https://huggingface.co/datasets/InternRobotics/GRScenes) (cấu trúc thư mục, `MDL_SYSTEM_PATH`, giấy phép)
- [R5] [InternUtopia](https://github.com/InternRobotics/InternUtopia) (User Agreement, `toolkits/grscenes_scripts`) · [issue #30](https://github.com/InternRobotics/InternUtopia/issues/30)
- [R6] [Dive into GRScenes-100](https://internrobotics.github.io/user_guide/internutopia/get_started/dive-into-grscenes-100.html) (thời gian nạp lần đầu)
- [R7] [RPLIDAR S2E datasheet v1.8](https://stgaws.robu.in/wp-content/uploads/2023/03/SLAMTEC_rplidar_datasheet_S2E_v1.8_en.pdf) · [REP 103](https://www.ros.org/reps/rep-0103.html)
- [R8] [Info-ZIP `zip`](https://linux.die.net/man/1/zip) (`-s 0 --out` gộp archive chia nhỏ) · [`unzip`](https://linux.die.net/man/1/unzip) · [huggingface_hub download](https://huggingface.co/docs/huggingface_hub/en/guides/download)
- [Docker Engine – Ubuntu](https://docs.docker.com/engine/install/ubuntu/) · [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
