# B′ Test Suite: Isaac Sim 4.1 ↔ ROS 2 Foxy (container)

Bộ kiểm thử xác nhận cấu hình mô phỏng **B′** cho đề tài robot Object Goal Navigation hỗ trợ người cao tuổi:

> **Isaac Sim 4.1** chạy trong container, dùng thư viện **ROS 2 Foxy nội bộ** của Isaac
> ⇄ **ROS 2 Foxy** chạy trong một container riêng.
> Hai container trao đổi dữ liệu qua Fast DDS (chỉ dùng UDP), trong một DDS domain riêng.

Bộ test trả lời một câu hỏi: **trên một máy cụ thể, chuỗi Isaac Sim 4.1 (Foxy nội bộ) ↔ Foxy có trao đổi đủ các loại dữ liệu mà đề tài cần không?** Các loại dữ liệu đó gồm: đồng hồ mô phỏng, lệnh vận tốc, camera RGB, LiDAR 2D (RPLIDAR S2E), odometry và TF.

---

## 0. Phương án
```bash
PC (Ubuntu 22.04.5 · RTX 3060 12 GB · driver 580 · Docker + NVIDIA Container Toolkit)
├─ Container Isaac Sim 4.1  (nvcr.io/nvidia/isaac-sim:4.1.0, headless)
│    └─ bridge dùng thư viện Foxy nội bộ (foxy/lib, BACKEND = Foxy, không Humble)
│       phát: /clock  /scan (RPLIDAR S2E)  /point_cloud  /odom  /tf  camera RGB
│       nhận: /cmd_vel → robot vi sai
├─ Container ros:foxy       (osrf/ros:foxy-desktop)
│    └─ pipeline của bạn (cùng launch file với Jetson), use_sim_time:=true
└─ Container Isaac Sim 5.1  (đã có sẵn; tắt khi chạy B′, cache tách riêng)

      ↕ Fast DDS chỉ UDP (fastdds_udp.xml) · --network host · ROS_DOMAIN_ID=42
      → Foxy ↔ Foxy, không có cầu chéo phiên bản · không cần dual-boot

Vận hành:  ./bprime.sh isaac base|t6   ⇄   ./bprime.sh foxy base|t6
Xem cảnh:  thêm --livestream → Chrome http://127.0.0.1:8211/streaming/webrtc-client?server=127.0.0.1
Trạng thái: laptop (host 20.04) PASS T0–T6 · PC (host 22.04, driver 580) chờ chạy kiểm chứng
```

## 1. Phạm vi

| Có kiểm tra | Không kiểm tra |
|---|---|
| Image 4.1 có tải được và có thư viện Foxy nội bộ không | Hiệu năng của cảnh lớn, ví dụ GRScenes |
| Tiến trình Isaac nạp **đúng** backend Foxy, không ngầm chuyển sang Humble | Nav2, slam_toolbox (sẽ là bước tích hợp sau) |
| DDS giữa hai container theo cả hai chiều | Mô hình robot thật của đề tài (bộ test dùng Jetbot thay thế) |
| `/clock`, `/cmd_vel`, camera RGB + `camera_info` | Hiệu năng và độ đúng khi bật livestream (`--livestream` chỉ để **xem**, mục 4.5) |
| RTX LiDAR RPLIDAR S2E → `/scan`, `/point_cloud` | Phần cứng thật (Jetson, LiDAR thật) |
| Robot vi sai: `/cmd_vel` → chuyển động → `/odom`, `/tf` | |

## 2. Yêu cầu

- Ubuntu 20.04 hoặc 22.04, GPU NVIDIA RTX. Image 4.1 được xây trên Ubuntu 22.04. Host 20.04 đã kiểm chứng (mục 6); host 22.04 chưa kiểm chứng.
- Driver NVIDIA ≥ 535.129.03. Đây là bản Isaac Sim 4.1 khuyến nghị; các bản mới hơn phải chạy thử mới biết.
- Docker Engine và NVIDIA Container Toolkit. Lệnh `install-docker` và `install-toolkit` sẽ cài nếu máy chưa có.
- Tài khoản NGC (`docker login nvcr.io`) nếu khi tải image bị báo `unauthorized`.
- Nên còn ≥ 50 GB ổ trống cho image và cache (image Isaac Sim rất lớn; xem dung lượng thực bằng `docker images` sau khi tải). Mạng phải truy cập được `nvcr.io`, Docker Hub và máy chủ asset S3 của NVIDIA.
- Không chạy container Isaac Sim nào khác trong lúc test, để tránh tranh VRAM.

## 3. Cấu trúc

```
bprime-test-suite/
├── README.md               Tài liệu này
├── bprime.sh               Lệnh duy nhất cần dùng (./bprime.sh help)
├── config/
│   ├── bprime.env          Image, tên container, DDS domain, thư mục cache/log
│   └── fastdds_udp.xml     Profile Fast DDS chỉ dùng UDP (theo tài liệu Isaac Sim 4.1)
├── isaac/                  Chạy bên trong container Isaac Sim
│   ├── env.sh              Thêm thư viện Foxy nội bộ vào LD_LIBRARY_PATH (một lần)
│   ├── inspect.sh          T2: kiểm tra nội dung image
│   ├── bprime_app.py       Khởi động Kit: headless (mặc định) hoặc headless + livestream WebRTC
│   ├── test_base.py        Graph /clock, /cmd_vel, camera; kiểm tra backend
│   └── test_t6.py          Cảnh Jetbot + RTX LiDAR + odom/TF
└── foxy/                   Chạy bên trong container Foxy
    ├── check_base.sh       T3, T4, T5
    ├── check_t6.py         T6a–T6f (đo bằng số liệu)
    └── diag_scan.py        Chẩn đoán /scan và /point_cloud
```

Thư mục bộ test được mount vào cả hai container ở chế độ **chỉ đọc**, tại `/bprime`. Mọi log được ghi ra host, mặc định ở `~/bprime_logs`.

## 4. Cách dùng

### 4.1 Chuẩn bị (mỗi máy làm một lần)

```bash
tar -xzf bprime-test-suite.tar.gz && cd bprime-test-suite
./bprime.sh preflight            # chỉ đọc thông tin máy, không thay đổi gì
```

| Máy | Các bước |
|---|---|
| **Chưa có Docker** (ví dụ laptop Ubuntu 20.04) | `./bprime.sh install-docker` → đăng xuất rồi đăng nhập lại → `./bprime.sh install-toolkit` (đã gồm T0) |
| **Đã có Docker + NVIDIA Toolkit** (ví dụ PC đang chạy Isaac Sim 5.1) | `./bprime.sh gpu-check` (T0) |

Sau đó, ở cả hai loại máy:

```bash
./bprime.sh pull                 # T1a: tải image và ghi lại digest
./bprime.sh inspect              # T2: kiểm tra image có thư viện Foxy nội bộ
```

### 4.2 Chạy test

Cần mở hai terminal, cùng ở thư mục bộ test. Bên Foxy **tự chờ** bên Isaac sẵn sàng (tối đa 300 s), nên không phải canh thời điểm. Bạn có thể bắt đầu terminal B ngay sau terminal A.

| Bước | Terminal A (Isaac) | Terminal B (Foxy) |
|---|---|---|
| Base: T1b, BACKEND, T3–T5 | `./bprime.sh isaac base` | `./bprime.sh foxy base` |
| T6: LiDAR, robot, odom, TF | `./bprime.sh isaac t6` | `./bprime.sh foxy t6` |

- Chạy xong một bước thì bấm **Ctrl+C** ở terminal A, rồi mới chuyển sang bước tiếp theo. Mỗi lúc chỉ chạy được một container Isaac.
- Muốn chẩn đoán thêm LiDAR (không bắt buộc): khi `isaac t6` đang chạy, chạy `./bprime.sh foxy scan` ở terminal B.
- Lần chạy Isaac đầu tiên trên mỗi máy phải biên dịch shader, nên có thể mất vài phút trước khi log in `simulation playing`.

### 4.3 Kết thúc

```bash
./bprime.sh collect              # đóng gói log: ~/bprime_report_<host>_<thời gian>.tar.gz
./bprime.sh stop                 # dừng và xóa các container test
```

### 4.4 Tùy chọn

| Lệnh | Tùy chọn |
|---|---|
| `isaac base` | `--duration S` (mặc định 300), `--no-camera`, `--no-assets`, `--livestream` |
| `isaac t6` | `--duration S` (mặc định 300), `--lidar-config NAME` (mặc định `RPLIDAR_S2E`), `--no-pointcloud`, `--livestream` |
| `isaac stream` | Không có tùy chọn. Mở app Isaac Sim đầy đủ (stage trống) qua `runheadless.webrtc.sh` |
| `shell isaac` / `shell foxy` | Mở shell tương tác trong container, môi trường đã được thiết lập sẵn |

### 4.5 Xem mô phỏng bằng livestream (WebRTC)

Container Isaac Sim chỉ chạy headless; cách xem được NVIDIA hỗ trợ là livestream. Vì container dùng `--network=host`, cổng 8211 mở thẳng trên máy chạy test, không cần mở port thêm.

1. Cài Chrome hoặc Chromium trên máy xem (tài liệu NVIDIA khuyến nghị; WebRTC có thể không chạy với Firefox):
   `sudo snap install chromium`
2. Terminal A, **trên host** (dấu nhắc `user@máy:~/bprime-test-suite$`, không phải `root@...:/isaac-sim#`):

   | Muốn xem | Lệnh |
   |---|---|
   | Cảnh T6 (Jetbot + LiDAR) | `./bprime.sh isaac t6 --livestream --duration 1800` |
   | Cảnh base (camera, /clock) | `./bprime.sh isaac base --livestream --duration 1800` |
   | App trống, để thử stream | `./bprime.sh isaac stream` |

3. Khi log in `simulation playing` (hoặc app nạp xong với `isaac stream`), mở trong Chrome:
   `http://127.0.0.1:8211/streaming/webrtc-client?server=127.0.0.1` rồi bấm **nút Play đỏ**.
   Xem từ máy khác trong LAN: thay `127.0.0.1` bằng IP của máy chạy Isaac (chưa kiểm chứng).
4. Terminal B: `./bprime.sh foxy t6` để robot chạy thẳng rồi quay tại chỗ. Hoặc lái tay từ host:
   ```bash
   source /opt/ros/foxy/setup.bash
   export ROS_DOMAIN_ID=42
   export FASTRTPS_DEFAULT_PROFILES_FILE=~/bprime-test-suite/config/fastdds_udp.xml   # cùng transport UDP với container
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

Lưu ý:
- **Chỉ dùng để xem.** Render UI và mã hóa video làm mô phỏng chậm hơn: trên laptop RTX 3050 4 GB, tỉ lệ thời gian thực giảm từ 0,47–0,59 xuống khoảng 0,39 (mục 6). Lần chạy dùng để kết luận PASS/FAIL hoặc đo hiệu năng phải chạy **không** có `--livestream`.
- Mỗi instance chỉ phục vụ **một** người xem; tab thứ hai sẽ báo lỗi.
- Viewport mặc định là camera **Perspective** nhìn chéo nên lưới sàn trông như hình thoi và đường thẳng có thể trông xiên. Chọn **Perspective → Top** để nhìn từ trên xuống, không còn phối cảnh.
- Cấu hình livestream (`isaac/bprime_app.py`) chép từ ví dụ có sẵn trong image `/isaac-sim/standalone_examples/api/omni.isaac.kit/livestream.py`, chỉ đổi sang extension WebRTC `omni.services.streamclient.webrtc` và cửa sổ 1280×720 để tiết kiệm VRAM.

## 5. Danh mục test và tiêu chí đạt

| ID | Kiểm tra | Xem kết quả ở | Đạt khi |
|---|---|---|---|
| T0 | GPU hiện ra trong container | `01_gpu_check.log` | `nvidia-smi` liệt kê GPU |
| T1a | Tải được image 4.1 | `02_image_digests.txt` | Có digest của cả hai image |
| T2 | Image có thư viện Foxy nội bộ | `10_inspect.log` | `T2 PASS` |
| T1b / BACKEND | Isaac khởi động, nạp **backend Foxy** | `20_isaac_base.log` | `BACKEND PASS` (Foxy được nạp, Humble không) |
| GRAPH | Tạo được graph `/clock`, `/cmd_vel` | `20_isaac_base.log` | `GRAPH PASS` |
| T3 | Isaac → Foxy | `21_foxy_base.log` | Thấy `/clock` với đúng kiểu message; timestamp tăng đều 1/60 s |
| T4 | Foxy → Isaac | `20_isaac_base.log` | `TWIST PASS ... lin=(0.3, 0.0, 0.0) ang=(0.0, 0.0, 0.5)` |
| T5 | Camera | `21_foxy_base.log` | `/bprime/rgb` có tần số > 0; `camera_info` 1280×720 |
| ROBOT / LIDAR / ODOM_TF / DRIVE | Dựng cảnh T6 | `30_isaac_t6.log` | Các dòng `PASS`, kèm `config='RPLIDAR_S2E'` |
| T6a | `/scan` | `31_foxy_t6.log` | ≥ 80% tia hợp lệ (robot đứng trong hộp kín) |
| T6b | `/odom` | `31_foxy_t6.log` | Frame `odom` → `base_link` |
| T6c | `/tf` | `31_foxy_t6.log` | Có đủ `odom→base_link` và `base_link→lidar_link` |
| T6d | Chạy thẳng v = 0,15 m/s | `31_foxy_t6.log` | Tỉ số đo/kỳ vọng trong 0,8–1,2 (0,5–1,5 = WARN) |
| T6e | Quay tại chỗ ω = 0,5 rad/s | `31_foxy_t6.log` | Tỉ số trong 0,8–1,2 (0,5–1,5 = WARN) |
| T6f | LiDAR so với odom (tham khảo) | `31_foxy_t6.log` | Khoảng cách tới tường phía trước giảm ≈ quãng đường odom |

Ghi chú về cách đo:
- T6d và T6e tính thời gian theo `/clock`, nên kết quả không phụ thuộc máy mô phỏng nhanh hay chậm. Lệnh được phát trong 6 s **thời gian thực**, nhưng giá trị kỳ vọng tính theo thời gian **mô phỏng** đã trôi: ví dụ ω = 0,5 rad/s × `sim_time_active` 2,33 s = 1,165 rad ≈ 66,8°, không phải 0,5 × 6 = 172°.
- `/odom` trong bộ test là vị trí thật của chassis trong mô phỏng (node `IsaacComputeOdometry`), không phải odometry tính từ encoder. Vì vậy T6d và T6e kiểm tra cả đường đi của lệnh: Foxy → bridge → bộ điều khiển vi sai → mô phỏng vật lý → `/odom` → Foxy.

**Cách kết luận cho một máy:**
- B′ **dùng được** khi T0, T1a, T2, BACKEND, GRAPH, T3, T4 và T6a–T6e đều PASS.
- Nếu **BACKEND FAIL** (Humble được nạp): mọi kết quả DDS sau đó **không còn giá trị cho B′**, vì thứ được kiểm thử khi đó là giao tiếp chéo phiên bản.
- Nếu **T1a hoặc T2 FAIL**: B′ không khả thi trên bất kỳ máy nào.

## 6. Kết quả tham chiếu

Kết quả trên laptop i7-12700H, RTX 3050 Laptop 4 GB, Ubuntu 20.04, driver 570.133.07, chạy ngày 19/10/2026. **Tất cả đều PASS.**

| Mục | Kết quả |
|---|---|
| BACKEND | Foxy được nạp (`libomni.isaac.ros2_bridge.foxy.so`, `foxy/lib/*`), Humble không được nạp |
| T3 | `/clock` tăng đúng 16,667 ms mỗi bước (1/60 s mô phỏng) |
| T4 | `TWIST PASS lin=(0.3, 0.0, 0.0) ang=(0.0, 0.0, 0.5)` |
| T5 | RGB khoảng 14 Hz (1280×720); `camera_info` fx = 3054,16, cx = 640, cy = 360 |
| T6a | 3200 tia, bước 0,1125°, tầm đo 0,05–30 m, chu kỳ quét 0,1 s, 89% tia hợp lệ. Cả bốn thông số khớp datasheet S2E |
| T6d / T6e | Tỉ số 1,12 / 1,03 |
| T6f | Khoảng cách tới tường trước giảm 0,488 m; odom ghi nhận 0,485 m |
| Hiệu năng | Khoảng 28–35 bước/s, tức mô phỏng chạy bằng **0,47–0,59 lần thời gian thực**. Các node ROS bắt buộc đặt `use_sim_time:=true` |
| T6 có `--livestream` (cùng ngày) | T6a–T6e vẫn PASS: 89% tia hợp lệ; T6d 0,395 m / 0,353 m (tỉ số 1,12); T6e 69,2° / 66,8° (tỉ số 1,03). Mô phỏng chậm còn **≈ 0,39 lần thời gian thực** (2,35 s mô phỏng trong 6 s thực). Mới có một lần chạy |

## 7. Hạn chế đã biết

- **Khoảng 11% tia `/scan` có giá trị −1,0,** dù `/point_cloud` vẫn có điểm ở các hướng đó. Chưa rõ nguyên nhân. Các thuật toán thường bỏ qua giá trị nhỏ hơn `range_min`.
- **Có điểm đo ở khoảng 0,08 m,** nhiều khả năng LiDAR quét trúng thân robot (chưa kiểm chứng). Với robot thật, cần tăng `range_min` hoặc lọc các tia này.
- **Jetbot tự quay khoảng −1° mỗi 10 s** khi lệnh bằng 0. Hiện tượng này gắn với mô hình Jetbot, không phải lỗi của bridge.
- **Cấu hình LiDAR `SICK_microscan3_*`** (máy quét an toàn 275°) cho `angle_increment` sai dấu và gần như không có tia hợp lệ. Không dùng cấu hình này cho đề tài.
- **`--livestream` làm chậm mô phỏng** (mục 4.5, mục 6). Ngoài một lần chạy T6 ở mục 6, chưa kiểm chứng độ đúng của các test khác khi bật livestream.
- **Chiều quét:** S2E thật quay theo chiều kim đồng hồ, còn ROS quy ước ngược lại. Scan mô phỏng đã theo đúng quy ước ROS (REP 103). Khi có LiDAR thật, cần đối chiếu với driver `rplidar_ros`.

## 8. Xử lý sự cố

Mỗi lần chỉ thay đổi một thứ, rồi chạy lại.

| Triệu chứng | Kiểm tra / cách sửa |
|---|---|
| `docker: permission denied` | Chưa đăng xuất/đăng nhập lại sau `install-docker` |
| `bash: ./bprime.sh: No such file or directory`, hoặc `bprime.sh must run on the host` | Bạn đang ở trong shell của container (`root@...:/isaac-sim#`). Gõ `exit` rồi chạy lại trên host |
| `python3: can't open file '/bprime/foxy/check_t6.py'` | Container `bprime-foxy` cũ còn mount một bản suite cũ (thư mục khác, hoặc đã xóa rồi clone lại). Bản `bprime.sh` hiện tại tự tạo lại container; với bản cũ, chạy `docker rm -f bprime-foxy` (không dùng `stop`, vì `stop` tắt luôn Isaac) |
| Chrome không vào được trang livestream | `ss -ltnp \| grep 8211` trên host. Không có dòng nào: extension chưa chạy, tìm `streamclient.webrtc` trong log terminal A. Có dòng: thử Chromium, tắt extension chặn quảng cáo, đóng tab xem khác |
| Livestream màn hình đen | Lần đầu có thể đang biên dịch shader; chờ 1–2 phút |
| `pull` báo `unauthorized` | `docker login nvcr.io`: username là `$oauthtoken`, password là NGC API key |
| `container bprime-isaac already exists` | Còn một test Isaac đang chạy hoặc bị treo → `./bprime.sh stop` |
| Bên Foxy chờ hết 300 s mà không thấy gì | (a) Terminal A đã in `simulation playing` chưa? (b) Cả hai bên đều phải in `ROS_DOMAIN_ID=42`. (c) `ip -br addr` phải có card mạng đang UP ngoài `lo` |
| Isaac bị kill hoặc treo khi khởi động | Thiếu RAM/VRAM: đóng ứng dụng khác, dừng các container Isaac khác, thử `--no-camera`. Theo dõi bằng `nvidia-smi -l 1` |
| `BACKEND UNKNOWN/FAIL` | Mở log Kit trong `${ISAAC_CACHE_ROOT}/logs`, tìm `ros2_bridge` |
| Một bước `... FAIL(graph)` | Log có dòng `ATTRS` liệt kê tên thuộc tính thật của node. Gửi log để sửa script |
| `/scan` ít tia hợp lệ | Chạy `./bprime.sh foxy scan` để so `/scan` với `/point_cloud`; thử `--lidar-config SICK_tim781` để đối chứng |

## 9. Gỡ bỏ

```bash
./bprime.sh stop
docker image rm nvcr.io/nvidia/isaac-sim:4.1.0 osrf/ros:foxy-desktop
sudo rm -rf ~/docker/isaac-sim-4.1 ~/bprime_logs        # file do container tạo thuộc user root
```

Bộ test không cài và không sửa ROS/Gazebo trên host. Gỡ Docker và NVIDIA Toolkit là tùy bạn.

## 10. Tài liệu tham khảo

- Isaac Sim 4.1 – [ROS and ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/4.1.0/installation/install_ros.html): thư viện Foxy nội bộ, `LD_LIBRARY_PATH`, profile Fast DDS
- Isaac Sim 4.2 – [Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/manual_livestream_clients.html): URL WebRTC, khuyến nghị Chrome, mỗi instance một client
- Isaac Sim 4.2 – [Container Installation](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/install_container.html), [ROS 2 RTX Lidar](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html), [ROS 2 TF & Odometry](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/ros2_tutorials/tutorial_ros2_tf.html), [Adding a Controller (Jetbot)](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/core_api_tutorials/tutorial_core_adding_controller.html)
- [Docker Engine – Install on Ubuntu](https://docs.docker.com/engine/install/ubuntu/) · [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [SLAMTEC RPLIDAR S2E datasheet v1.8](https://stgaws.robu.in/wp-content/uploads/2023/03/SLAMTEC_rplidar_datasheet_S2E_v1.8_en.pdf) · [REP 103](https://www.ros.org/reps/rep-0103.html)
