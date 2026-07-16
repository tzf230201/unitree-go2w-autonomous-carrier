# OM6DOF ROS 2 System

Stack ROS 2 Humble untuk manipulator OM6DOF: lengan 6-DOF turunan
OpenMANIPULATOR-X, gripper prismatic, driver Dynamixel, ros2_control, MoveIt,
teleop Go2W, dan aplikasi pick-and-place.

Sistem ini mandiri dan tidak lagi bergantung pada
`open_manipulator_friends_ros2`.

## Arsitektur

Hanya `om6dof_bringup` yang boleh membuka U2D2 dan bus Dynamixel.
`om6dof_controller` menjadi satu-satunya node yang mengubah perintah enam
sumbu menjadi target posisi joint untuk `forward_position_controller`.

```text
Go2W remote                 Program streaming lain
     |                               |
     v                               v
om6dof_teleop ----> /om6dof/operation_mode
               `---> /om6dof/control_cmd
                               |
                               v
                      om6dof_controller
                     - mode dan ownership
                     - integrasi joint
                     - Cartesian velocity IK
                     - cylindrical velocity IK
                     - limit, watchdog, collision check
                               |
                               v
                 forward_position_controller ---+
                                                  |
MoveIt ---> arm_controller -----------------------+--> ros2_control
                                                        |
                                             dynamixel_hardware_interface
                                                        |
                                                       U2D2
```

`arm_controller` dan `forward_position_controller` menggunakan enam command
interface yang sama. Karena itu keduanya eksklusif dan tidak pernah boleh aktif
bersamaan:

- `arm_controller` digunakan oleh MoveIt dan autonomous trajectory.
- `forward_position_controller` digunakan untuk JOINT, CARTESIAN, dan
  CYLINDRICAL streaming control.
- `om6dof_controller` melakukan pergantian keduanya melalui
  `/controller_manager/switch_controller`.
- Gripper memakai command interface tersendiri dan tetap aktif pada kedua
  kondisi tersebut.

`om6dof_controller` adalah supervisor berbasis ROS 2 yang memakai
ros2_control. Node ini bukan plugin controller real-time.

## Quick start

Periksa terlebih dahulu apakah systemd sudah menjalankan full stack:

```bash
systemctl is-active om6dof-hardware.service
```

Jika hasilnya `active`, jangan menjalankan hardware kedua. Cukup gunakan stack
yang sudah hidup:

```bash
ros2 control list_controllers
```

Jika service tidak aktif dan workspace sudah di-build, jalankan manual:

```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
ros2 launch om6dof_teleop full_stack.launch.py
```

Sistem mulai pada AUTONOMOUS. Tekan F3 untuk mengambil remote ownership dan
masuk READY, Select untuk mengganti JOINT/CARTESIAN/CYLINDRICAL, dan F3 lagi
untuk mengembalikan ownership ke MoveIt.

## Struktur package

| Package/folder | Fungsi |
|---|---|
| [`om6dof_description`](om6dof_description/) | URDF/Xacro, mesh, joint/link, dan model visual |
| [`om6dof_bringup`](om6dof_bringup/) | Pemilik hardware, ros2_control node, dan konfigurasi controller |
| [`om6dof_controller`](om6dof_controller/) | Konversi JOINT/CARTESIAN/CYLINDRICAL, IK, limit, watchdog, READY/STARTUP, dan controller switching |
| [`om6dof_teleop`](om6dof_teleop/) | Adapter input remote Go2W ke dua topic command publik |
| [`om6dof_moveit_config`](om6dof_moveit_config/) | SRDF, kinematics, planning, controller mapping, dan RViz MoveIt |
| [`om6dof_pick_and_place`](om6dof_pick_and_place/) | AprilTag, MoveIt pick-and-place, tracking, dan calibration tools |
| `dynamixel_hardware_interface` | Plugin ros2_control Dynamixel yang sudah membawa patch lokal |
| `dynamixel_interfaces` | Message dan service ROS untuk Dynamixel |
| `DynamixelSDK` | SDK Dynamixel yang di-vendor di dalam source tree |

## Controller ros2_control

Konfigurasi berada di
[`om6dof_bringup/config/controllers.yaml`](om6dof_bringup/config/controllers.yaml).

| Controller | Plugin | Kondisi awal |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | active |
| `arm_controller` | `JointTrajectoryController` | active |
| `forward_position_controller` | `ForwardCommandController` | inactive |
| `gripper_controller` | `GripperActionController` | active |

## Topic inti

### API command OM6DOF

Ini adalah interface utama untuk manual/jog streaming yang boleh dipakai
teleop maupun program lain, tetapi hanya satu command source boleh aktif pada
satu waktu. MoveIt memakai action trajectory, bukan API streaming ini.

| Topic | Type | Arah | Keterangan |
|---|---|---|---|
| `/om6dof/operation_mode` | `std_msgs/msg/String` | command source -> controller | Meminta mode atau ownership |
| `/om6dof/control_cmd` | `std_msgs/msg/Float64MultiArray` | command source -> controller | Enam nilai velocity sesuai mode aktif |
| `/om6dof/operation_mode/state` | `std_msgs/msg/String` | controller -> client | Mode yang benar-benar diterima controller |
| `/om6dof/remote_enabled/state` | `std_msgs/msg/Bool` | controller -> client | `true` jika forward controller benar-benar memiliki arm |

QoS command:

- `operation_mode`: reliable, volatile, keep-last 1.
- `control_cmd`: best-effort, volatile, keep-last 1.
- Kedua state topic: reliable, transient-local, keep-last 1.

State topic bersifat transient-local sehingga client yang baru berjalan tetap
menerima state terakhir.

### Topic feedback dan internal

| Topic | Type | Arah | Keterangan |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | ros2_control -> semua client | Posisi/velocity arm dan gripper; effort gripper berisi Present Current |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | ros2_control -> diagnostic client | State seluruh hardware interface dalam bentuk dinamis |
| `/forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | controller -> ros2_control | Internal: enam target posisi joint; jangan publish langsung |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | trajectory client -> ros2_control | Interface topic low-level trajectory controller |
| `/arm_controller/state` | `control_msgs/msg/JointTrajectoryControllerState` | arm controller -> client | State trajectory controller |
| `/arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | arm controller -> client | State controller untuk kompatibilitas ROS 2 control |
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher -> controller manager | URDF hasil render Xacro |
| `/tf` | `tf2_msgs/msg/TFMessage` | robot_state_publisher -> graph | Transform dinamis robot |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher -> graph | Transform statis robot |
| `/dynamixel_hardware_interface/dxl_state` | `dynamixel_interfaces/msg/DynamixelState` | driver -> diagnostic client | Status komunikasi/hardware Dynamixel; hanya hardware asli |

Hanya `om6dof_controller` yang boleh menjadi publisher pada
`/forward_position_controller/commands`.

### Topic teleop Go2W dan gripper

| Topic | Type | Arah | Keterangan |
|---|---|---|---|
| `/lowstate` | `unitree_go/msg/LowState` | Go2W -> teleop | Sumber utama payload remote 40-byte dan state Go2W |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | Go2W/web -> teleop | Fallback event remote dan input web monitor |
| `/om6dof_teleop/gripper_cmd` | `std_msgs/msg/String` | client -> teleop | `open` atau `close` |
| `/om6dof_teleop/gripper_state` | `std_msgs/msg/String` | teleop -> client | Status OPEN/CLOSED/HOLDING serta posisi dan arus |

### Topic aplikasi pick-and-place opsional

Topic ini hanya muncul ketika node aplikasinya dijalankan.

| Topic | Type | Keterangan |
|---|---|---|
| `/apriltag/pose` | `geometry_msgs/msg/PoseStamped` | Pose AprilTag hasil detector |
| `/apriltag/debug_image` | `sensor_msgs/msg/Image` | Citra detector dengan overlay debug |
| `/tag_markers` | `visualization_msgs/msg/MarkerArray` | Visualisasi tag dan waypoint pick di RViz |
| `/direct_reach` | `std_msgs/msg/String` | Status reachability direct-pick |
| `/qr_follow/markers` | `visualization_msgs/msg/MarkerArray` | Visualisasi target QR follower |
| `/coord_debug_markers` | `visualization_msgs/msg/MarkerArray` | Visualisasi coordinate-debug |

Detail application-specific tersedia di
[`om6dof_pick_and_place/README.md`](om6dof_pick_and_place/README.md).

## Action dan service penting

| Nama | Type | Keterangan |
|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Eksekusi trajectory arm; dipakai MoveIt |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Command gripper standar ros2_control |
| `/move_action` | `moveit_msgs/action/MoveGroup` | Planning dan execution request ke MoveGroup |
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | Membaca state controller |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | Internal ownership switch oleh `om6dof_controller` |
| `/dynamixel_hardware_interface/get_dxl_data` | `dynamixel_interfaces/srv/GetDataFromDxl` | Membaca item control table; hardware asli |
| `/dynamixel_hardware_interface/set_dxl_data` | `dynamixel_interfaces/srv/SetDataToDxl` | Menulis item control table; hardware asli |
| `/dynamixel_hardware_interface/reboot_dxl` | `dynamixel_interfaces/srv/RebootDxl` | Reboot Dynamixel; hardware asli |
| `/dynamixel_hardware_interface/set_dxl_torque` | `std_srvs/srv/SetBool` | Mengubah torque Dynamixel; hardware asli |

Gunakan `/om6dof/operation_mode` untuk mengganti ownership. Jangan memanggil
`switch_controller` secara manual pada operasi normal karena
`om6dof_controller` juga menyimpan state, command anchor, dan watchdog.

Beberapa service aplikasi yang tersedia ketika package pick-and-place aktif:

- `/run_pick_place` dan `/snapshot_waypoint`.
- `/run_tag_pick`, `/run_search`, `/run_front_approach`, dan
  `/tag_pick_status`.
- `/run_direct_pick`, `/direct_approach`, `/direct_go_origin`,
  `/direct_go_ready`, `/direct_track`, dan `/direct_stop`.
- `/qr_follow/start`, `/qr_follow/stop`, dan `/qr_follow/status`.

## Kontrak operation mode

`/om6dof/operation_mode` menerima nilai berikut, tanpa membedakan huruf besar
dan kecil:

| Nilai | Perilaku |
|---|---|
| `AUTONOMOUS` | Nonaktifkan forward controller dan kembalikan ownership ke `arm_controller` untuk MoveIt |
| `JOINT` | Ambil ownership streaming; jika sebelumnya autonomous, lakukan controller switch lalu bergerak ke READY |
| `CARTESIAN` | Interpretasikan enam nilai sebagai linear dan angular velocity end-effector |
| `CYLINDRICAL` | Interpretasikan tiga nilai pertama sebagai radius, theta, dan Z velocity |
| `READY` | Jika perlu ambil streaming ownership, lalu bergerak menuju ready pose; setelah selesai kembali ke JOINT |
| `STARTUP` | Dengan streaming ownership aktif, bergerak ke pose feedback pertama yang ditangkap saat controller mulai; setelah selesai kembali ke JOINT |

`CARTESIAN` dan `CYLINDRICAL` hanya diterima setelah streaming ownership aktif.
Dari kondisi autonomous, kirim `JOINT` terlebih dahulu dan tunggu READY/JOINT.

Command nonnol ketika READY atau STARTUP sedang bergerak akan menghentikan
gerakan pose tersebut dan memberikan kontrol langsung ke JOINT mode. Command
nol tidak membatalkan gerakan pose.

## Kontrak control command

`/om6dof/control_cmd` harus selalu berisi tepat enam angka finite. Nilainya
adalah velocity, bukan target posisi absolut.

| Mode | `data` | Unit/frame |
|---|---|---|
| `JOINT` | `[q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot]` | rad/s |
| `CARTESIAN` | `[vx, vy, vz, wx, wy, wz]` | XYZ m/s pada frame `world`; angular rad/s pada tool frame |
| `CYLINDRICAL` | `[radius_dot, theta_dot, z_dot, wx, wy, wz]` | m/s, rad/s, m/s; angular pada tool frame |

Publish command secara kontinu pada 20-50 Hz. Watchdog controller akan
menganggap stream kedaluwarsa setelah 0,3 detik, menahan feedback terkini, dan
menanam ulang anchor IK. Karena itu command lama tidak akan terus menggerakkan
arm setelah publisher berhenti.

Jika `/joint_states` kedaluwarsa lebih dari 1 detik, controller berhenti
menghasilkan target baru. Origin cylindrical default adalah XY `[0.012, 0.0]`,
dan ready pose default joint1..joint6 adalah
`[0.0, -0.6806, 1.3613, 0.0, 0.8901, 0.0]`. Seluruh nilai dapat diubah melalui
[`om6dof_controller/config/controller.yaml`](om6dof_controller/config/controller.yaml).

Beberapa publisher pada `/om6dof/control_cmd` bersifat last-writer-wins. Nilai
dari teleop dan autonomous streamer tidak dijumlahkan seperti `cmd_vel`. Jika
nanti keduanya harus aktif bersamaan, tambahkan command mux dengan topic input
yang terpisah untuk setiap sumber.

Secara khusus, `om6dof_teleop` menerbitkan command pada 50 Hz, termasuk command
nol ketika joystick netral, selama remote ownership aktif. Karena itu program
eksternal tidak boleh menerbitkan `/om6dof/control_cmd` bersamaan dengan
teleop. Command keduanya akan saling menimpa.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh

colcon build --packages-up-to \
  om6dof_teleop om6dof_moveit_config om6dof_pick_and_place \
  --symlink-install

source install/setup.bash
```

Setelah source berubah, build ulang sebelum me-restart service yang sedang
menjalankan robot.

## Menjalankan sistem

### Full stack pada hardware asli

Perintah berikut menjalankan tepat satu hardware owner, controller converter,
dan adapter Go2W:

```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash

ros2 launch om6dof_teleop full_stack.launch.py
```

Argument yang tersedia:

```bash
ros2 launch om6dof_teleop full_stack.launch.py \
  port_name:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0 \
  baud_rate:=1000000 \
  joint_velocity:=0.5 \
  remote_enabled_on_start:=false
```

Default `remote_enabled_on_start` adalah `false`, sehingga sistem mulai pada
AUTONOMOUS dengan `arm_controller` aktif. F3 mengambil ownership dan membawa
arm ke READY.

Untuk langsung mengambil streaming ownership dan masuk READY saat stack mulai:

```bash
ros2 launch om6dof_teleop full_stack.launch.py \
  remote_enabled_on_start:=true
```

Gunakan pilihan ini hanya ketika area gerak arm aman karena robot dapat mulai
bergerak setelah feedback dan controller manager siap.

### Menjalankan setiap layer secara terpisah

Terminal 1, hardware owner:

```bash
ros2 launch om6dof_bringup hardware.launch.py
```

Terminal 2, command converter:

```bash
ros2 launch om6dof_controller controller.launch.py
```

Terminal 3, adapter remote opsional:

```bash
ros2 launch om6dof_teleop teleop.launch.py
```

Jangan menjalankan `hardware.launch.py` jika service systemd sudah memiliki
U2D2.

### Fake hardware

Tanpa RViz:

```bash
ros2 launch om6dof_teleop full_stack.launch.py \
  use_fake_hardware:=true
```

Dengan RViz:

```bash
ros2 launch om6dof_teleop sim.launch.py
```

Untuk pengujian yang benar-benar terpisah dari graph robot asli, gunakan
`ROS_DOMAIN_ID` lain pada semua terminal pengujian:

```bash
export ROS_DOMAIN_ID=179
ros2 launch om6dof_teleop full_stack.launch.py \
  use_fake_hardware:=true
```

### systemd

Unit service disediakan oleh `om6dof_teleop`:

```bash
sudo cp \
  ~/ros2_ws/install/om6dof_teleop/share/om6dof_teleop/systemd/om6dof-hardware.service \
  /etc/systemd/system/om6dof-hardware.service

sudo systemctl daemon-reload
sudo systemctl enable om6dof-hardware.service
sudo systemctl restart om6dof-hardware.service
```

Periksa service dan log:

```bash
systemctl status om6dof-hardware.service
journalctl -u om6dof-hardware.service -f
```

Jangan restart service ketika arm atau lingkungan berada pada kondisi yang
tidak aman.

### Restart full stack dari Go2W Monitor

Go2W Monitor berjalan pada service terpisah, sehingga dashboard tetap hidup
ketika `om6dof-hardware.service` dimulai ulang. Buka:

```text
http://<ip-go2w>:8080
```

Tombol **Restart OM6DOF stack** memulai ulang satu jalur penuh:
`om6dof_bringup`, `om6dof_controller`, dan `om6dof_teleop`. Proses berjalan di
background; dashboard menunggu maksimal 45 detik sampai menemukan MainPID
systemd yang baru, node runtime kembali, broadcaster dan gripper aktif, serta
tepat satu dari `arm_controller` atau `forward_position_controller` aktif.

Web monitor berjalan tanpa privilege. Setelah package dibangun, perbarui unit
monitor dan pasang izin minimal berikut satu kali:

```bash
sudo install -o root -g root -m 0644 \
  ~/ros2_ws/install/application_web_monitor/share/application_web_monitor/systemd/om6dof-web-monitor.service \
  /etc/systemd/system/om6dof-web-monitor.service
sudo install -o root -g root -m 0440 \
  ~/ros2_ws/install/application_web_monitor/share/application_web_monitor/sudoers/om6dof-web-monitor \
  /etc/sudoers.d/om6dof-web-monitor
sudo visudo -cf /etc/sudoers.d/om6dof-web-monitor
sudo systemctl daemon-reload
sudo systemctl enable om6dof-web-monitor.service
sudo systemctl restart om6dof-web-monitor.service
```

Rule tersebut hanya mengizinkan command tetap berikut, bukan shell atau
command `systemctl` lain:

```text
/usr/bin/systemctl --no-block restart om6dof-hardware.service
```

Jika proses `om6dof_controller` keluar, launch full stack sekarang ikut berhenti
dan `Restart=always` pada systemd membangun ulang ketiga layer secara otomatis.
Controller ros2_control yang sekadar berubah menjadi state salah ditandai
`ACTIVE / INCOMPLETE` pada monitor; gunakan tombol restart setelah memastikan
area arm aman.

Restart memutus kontrol dan arm dapat bergerak saat inisialisasi. Dashboard
bind ke `0.0.0.0` tanpa login, sehingga port 8080 hanya boleh tersedia pada LAN
robot tepercaya atau di belakang firewall/VPN.

## Mengontrol arm melalui topic

> **Penting:** contoh pada bagian ini memerlukan hardware +
> `om6dof_controller` tanpa `om6dof_teleop`. Full stack dan
> `om6dof-hardware.service` menjalankan teleop, sehingga publisher eksternal
> akan bersaing dengan stream teleop 50 Hz. Hentikan service pada kondisi arm
> yang aman, lalu jalankan `hardware.launch.py` dan `controller.launch.py`
> seperti bagian "Menjalankan setiap layer secara terpisah". Alternatif
> jangka panjangnya adalah menambahkan command mux.

### 1. Ambil ownership dan masuk READY

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: JOINT}"
```

Periksa state yang telah dikonfirmasi:

```bash
ros2 topic echo --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /om6dof/operation_mode/state

ros2 topic echo --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /om6dof/remote_enabled/state
```

Tunggu state kembali menjadi `JOINT` jika READY harus diselesaikan sebelum
memberikan command berikutnya.

### 2. JOINT mode

Contoh joint1 positif 0,2 rad/s:

```bash
ros2 topic pub --rate 20 \
  --qos-reliability best_effort \
  --qos-durability volatile \
  /om6dof/control_cmd std_msgs/msg/Float64MultiArray \
  "{data: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Hentikan publisher dengan Ctrl+C. Watchdog akan menahan posisi berdasarkan
feedback.

### 3. CARTESIAN mode

Pilih mode:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: CARTESIAN}"
```

Gerakkan end-effector ke +X pada frame `world` dengan 0,02 m/s:

```bash
ros2 topic pub --rate 20 \
  --qos-reliability best_effort \
  --qos-durability volatile \
  /om6dof/control_cmd std_msgs/msg/Float64MultiArray \
  "{data: [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Contoh tool yaw positif 0,2 rad/s:

```bash
ros2 topic pub --rate 20 \
  --qos-reliability best_effort \
  --qos-durability volatile \
  /om6dof/control_cmd std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.2]}"
```

### 4. CYLINDRICAL mode

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: CYLINDRICAL}"
```

Perbesar radius dengan 0,02 m/s:

```bash
ros2 topic pub --rate 20 \
  --qos-reliability best_effort \
  --qos-durability volatile \
  /om6dof/control_cmd std_msgs/msg/Float64MultiArray \
  "{data: [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Putar theta positif dengan 0,2 rad/s tanpa meminta perubahan radius:

```bash
ros2 topic pub --rate 20 \
  --qos-reliability best_effort \
  --qos-durability volatile \
  /om6dof/control_cmd std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.2, 0.0, 0.0, 0.0, 0.0]}"
```

### 5. READY dan STARTUP

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: READY}"

ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: STARTUP}"
```

### 6. Kembalikan ownership ke MoveIt

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: AUTONOMOUS}"
```

Pastikan `arm_controller` aktif sebelum mengeksekusi trajectory:

```bash
ros2 control list_controllers
```

## Menggunakan remote Go2W

`om6dof_teleop` membaca `/lowstate.wireless_remote` sebagai sumber utama dan
`/wirelesscontroller` sebagai fallback. Edge dari kedua sumber di-deduplicate
agar satu penekanan tombol tidak dieksekusi dua kali.

| Input | JOINT | CARTESIAN | CYLINDRICAL |
|---|---|---|---|
| Left / Right | joint1 -/+ | world Y +/- | theta CCW/CW |
| Down / Up | joint2 -/+ | world Z -/+ | Z -/+ |
| A / Y | joint3 -/+ | world X -/+ | radius inward/outward |
| X / B | joint4 -/+ | tool roll +/- | tool roll +/- |
| Right stick `ry` | joint5 analog | tool pitch | tool pitch |
| R1 / R2 | joint6 -/+ | tool yaw +/- | tool yaw +/- |
| L1 / L2 | gripper open/close | gripper open/close | gripper open/close |
| Select | pindah ke CARTESIAN | pindah ke CYLINDRICAL | kembali ke JOINT |
| F3 | remote ON/OFF | remote ON/OFF | remote ON/OFF |
| F1 | STARTUP/READY | kembali ke JOINT lalu pose | kembali ke JOINT lalu pose |

Perilaku tombol utama:

- F3 ON: minta JOINT ownership, switch ke forward controller, lalu READY.
- F3 OFF: kembalikan ownership ke `arm_controller`.
- Select: siklus JOINT -> CARTESIAN -> CYLINDRICAL -> JOINT.
- F1: bergantian menuju STARTUP dan READY.
- Setelah enable atau pergantian mode, teleop menunggu input netral sebelum
  command gerak diteruskan.

## Gripper

Melalui topic adapter teleop:

```bash
ros2 topic pub --once /om6dof_teleop/gripper_cmd \
  std_msgs/msg/String "{data: open}"

ros2 topic pub --once /om6dof_teleop/gripper_cmd \
  std_msgs/msg/String "{data: close}"
```

Atau langsung melalui action standar ros2_control:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.019, max_effort: 0.0}}"
```

Target nominal:

- Open: `0.019 m`.
- Close: `-0.010 m`.

Pantau hasil deteksi gripper:

```bash
ros2 topic echo /om6dof_teleop/gripper_state
```

## MoveIt

Hardware/full stack harus sudah berjalan dan ownership harus AUTONOMOUS:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: AUTONOMOUS}"

ros2 launch om6dof_moveit_config om6dof_moveit.launch.py
```

Jika `om6dof-hardware.service` sudah menjadi hardware owner, wrapper bringup
berikut juga dapat digunakan agar tidak membuka U2D2 kedua:

```bash
ros2 launch om6dof_bringup real.launch.py start_hardware:=false
```

MoveIt menjalankan trajectory melalui
`/arm_controller/follow_joint_trajectory`. Jangan menekan F3 saat trajectory
sedang dieksekusi. Setelah kembali dari remote mode, plan ulang trajectory agar
start state sesuai feedback terbaru.

## Pick-and-place

Contoh launch yang tersedia:

```bash
ros2 launch om6dof_pick_and_place pick_place.launch.py
ros2 launch om6dof_pick_and_place tag_pick_place.launch.py
ros2 launch om6dof_pick_and_place direct_pick.launch.py
ros2 launch om6dof_pick_and_place qr_follower.launch.py
```

Aplikasi ini menggunakan MoveIt/`arm_controller`. Pastikan remote ownership
OFF atau kirim `AUTONOMOUS` sebelum menjalankan sequence.

## Verifikasi

Periksa node penting:

```bash
ros2 node list | grep -E \
  'controller_manager|om6dof_controller|om6dof_teleop|robot_state_publisher'
```

Lihat seluruh interface ROS yang sedang aktif beserta type-nya:

```bash
ros2 topic list --show-types
ros2 action list --show-types
ros2 service list --show-types
```

Periksa controller:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Kondisi awal yang diharapkan:

```text
joint_state_broadcaster      active
arm_controller               active
forward_position_controller inactive
gripper_controller           active
```

Setelah F3 ON atau request JOINT:

```text
joint_state_broadcaster      active
arm_controller               inactive
forward_position_controller active
gripper_controller           active
```

Pastikan hanya controller baru yang menulis final joint command:

```bash
ros2 topic info --verbose /forward_position_controller/commands
```

Publisher count harus `1` dengan node name `om6dof_controller`.

Periksa feedback:

```bash
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states
```

Jalankan test software:

```bash
cd ~/ros2_ws
source install/setup.bash

colcon test --packages-select om6dof_controller om6dof_teleop
colcon test-result --verbose
```

## Troubleshooting

### F3 ditekan tetapi arm tidak bergerak

```bash
ros2 node list | grep om6dof_controller
ros2 topic echo --once --qos-durability transient_local \
  /om6dof/remote_enabled/state
ros2 control list_controllers
journalctl -u om6dof-hardware.service -n 100 --no-pager
```

Pastikan:

- `om6dof_controller` aktif.
- `/joint_states` tersedia dan tidak stale.
- `forward_position_controller` berhasil menjadi active.
- Remote sudah kembali netral setelah F3; teleop memakai neutral latch.

### `/controller_manager/list_controllers` timeout

Artinya controller manager tidak merespons atau proses hardware telah berhenti.
Periksa:

```bash
systemctl status om6dof-hardware.service
journalctl -u om6dof-hardware.service -n 200 --no-pager
ls -l /dev/serial/by-id/
```

### Tombol restart pada Go2W Monitor ditolak

Pesan `Install the scoped web-monitor sudoers rule` berarti izin satu-command
belum terpasang atau package belum dibangun ulang. Periksa:

```bash
sudo visudo -cf /etc/sudoers.d/om6dof-web-monitor
sudo -n \
  /usr/bin/systemctl --no-block restart om6dof-hardware.service
```

Command kedua benar-benar melakukan restart. Jalankan hanya ketika area arm
aman. Jika hanya ingin memeriksa rule tanpa restart, gunakan:

```bash
sudo -n -l
```

### Port U2D2 sibuk

Jangan menjalankan dua `hardware.launch.py`. Jika systemd aktif, gunakan stack
yang sudah berjalan:

```bash
systemctl is-active om6dof-hardware.service
pgrep -af 'ros2_control_node|full_stack.launch'
```

### Mode Cartesian atau cylindrical ditolak

Masuk JOINT terlebih dahulu:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: JOINT}"
```

Tunggu `/om6dof/remote_enabled/state=true`, kemudian minta mode coordinate.

## Patch lokal `dynamixel_hardware_interface`

Copy vendored ini bukan upstream ROBOTIS murni. Patch berikut memperbaiki
`-3001 SYNC_READ_FAIL` yang pernah muncul pada rig ini dan harus dipertahankan
jika dependency di-vendor ulang.

1. `dynamixel_hardware_interface/src/dynamixel/dynamixel.cpp`:
   packet timeout diberi batas minimum 5 ms:

   ```cpp
   port_handler->setPacketTimeout(period_ms < 5.0 ? 5.0 : period_ms);
   ```

2. `dynamixel_hardware_interface/include/dynamixel_hardware_interface/dynamixel/dynamixel.hpp`:
   menambahkan `use_fast_read_protocol_` dan
   `SetUseFastReadProtocol(bool)`.

3. `dynamixel_hardware_interface/src/dynamixel_hardware_interface.cpp`:
   membaca parameter URDF `use_fast_read_protocol` dan meneruskannya ke
   driver.

4. `om6dof_bringup/urdf/om6dof.ros2_control.xacro` mengatur
   `use_fast_read_protocol=0` agar memakai plain SyncRead.

Di luar source tree, U2D2/FTDI juga membutuhkan `latency_timer=1` melalui rule
udev `/etc/udev/rules.d/99-u2d2-latency.rules`. Tanpa pengaturan ini,
`-3001` dapat kembali muncul walaupun packet timeout sudah diperbaiki.
