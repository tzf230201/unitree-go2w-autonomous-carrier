# go2w_remote_monitor

`go2w_remote_monitor` adalah package ROS 2 Python untuk membaca dan menampilkan data remote controller Unitree Go2W dari topic `/lowstate`.

Package ini tidak membuat data remote sendiri. Package ini hanya menjadi subscriber ke topic status robot yang sudah dipublish oleh stack Unitree atau bridge yang menyediakan pesan `unitree_go/msg/LowState`.

## Data Remote Diambil Dari Mana

Sumber data remote pada package ini adalah:

- Topic ROS 2: `/lowstate`
- Tipe pesan: `unitree_go/msg/LowState`
- Field yang dipakai: `wireless_remote`

Definisi resmi field tersebut di dalam `LowState` adalah:

```text
uint8[40] wireless_remote
```

Artinya, data remote masuk sebagai array 40 byte di dalam satu pesan status robot yang lebih besar. Selain `wireless_remote`, pesan `LowState` juga membawa data lain seperti:

- `imu_state`
- `motor_state[20]`
- `bms_state`
- `foot_force`
- `tick`
- `power_v`
- `power_a`

Jadi alur datanya adalah:

1. Robot atau driver Unitree mem-publish `unitree_go/msg/LowState`.
2. Topic default yang dipakai di repo ini adalah `/lowstate`.
3. Node `go2w_remote_monitor` subscribe ke `/lowstate`.
4. Node mengambil `msg.wireless_remote`.
5. Byte di dalam `wireless_remote` di-decode menjadi tombol dan nilai stick.

## Bukti Di Repo Ini

Di repo ini, penggunaan `/lowstate` dan `LowState` juga muncul pada beberapa komponen lain:

- Package monitor ini subscribe ke `/lowstate`.
- `go2w_lio_sam` juga memakai `LowState` sebagai input IMU.
- `go2w_joints_state_and_imu_publisher` menyalin field `wireless_remote` ke struktur low-level lain, yang menunjukkan bahwa data ini memang bagian dari status robot yang datang dari interface Unitree.

Dengan kata lain, sumber data remote bukan berasal dari keyboard PC, joystick Linux biasa, atau kamera. Sumbernya adalah payload remote yang dibawa oleh pesan `LowState` dari sistem Unitree.

## Cara Decode Yang Dipakai Package Ini

Implementasi saat ini mengikuti script `monitor_remote.py` yang sudah ada sebelumnya di root repo.

### Tombol

Package ini membaca bitmap tombol dari:

- `wireless_remote[2]`
- `wireless_remote[3]`

Lalu dua byte itu digabung menjadi integer 16-bit little-endian:

```python
key_value = wr[2] | (wr[3] << 8)
```

Setelah itu, setiap bit dipetakan ke nama tombol:

- bit 0  -> `R1`
- bit 1  -> `L1`
- bit 2  -> `Start`
- bit 3  -> `Select`
- bit 4  -> `R2`
- bit 5  -> `L2`
- bit 6  -> `F1`
- bit 7  -> `F2`
- bit 8  -> `A`
- bit 9  -> `B`
- bit 10 -> `X`
- bit 11 -> `Y`
- bit 12 -> `Up`
- bit 13 -> `Right`
- bit 14 -> `Down`
- bit 15 -> `Left`

### Stick

Untuk monitoring praktis, package ini membaca axis dari byte berikut:

- `lx = wireless_remote[7] - 128`
- `ly = wireless_remote[6] - 128`
- `rx = wireless_remote[11] - 128`
- `ry = wireless_remote[15] - 128`

Nilai `128` diperlakukan sebagai titik tengah. Jadi hasil akhirnya adalah nilai signed sederhana di sekitar nol.

Catatan penting:

Mapping stick di atas mengikuti script yang sudah ada di repo, sehingga cocok untuk monitoring cepat. Namun offset byte untuk axis tetap sebaiknya divalidasi lagi terhadap dokumentasi remote Unitree yang spesifik untuk firmware atau model remote yang sedang dipakai.

## Struktur Package

File utama package ini:

- `go2w_remote_monitor/go2w_remote_monitor/remote_monitor.py`
- `go2w_remote_monitor/launch/remote_monitor.launch.py`
- `go2w_remote_monitor/package.xml`
- `go2w_remote_monitor/setup.py`

## Cara Build

Dari workspace ROS 2:

```bash
cd ~/ros2_ws
colcon build --packages-select go2w_remote_monitor --symlink-install
source install/setup.bash
```

## Cara Menjalankan

Jalankan node langsung:

```bash
ros2 run go2w_remote_monitor remote_monitor
```

Atau lewat launch file:

```bash
ros2 launch go2w_remote_monitor remote_monitor.launch.py
```

## Parameter

Parameter yang tersedia:

- `lowstate_topic` default: `/lowstate`
- `log_stick_on_every_message` default: `false`

Contoh:

```bash
ros2 run go2w_remote_monitor remote_monitor --ros-args -p lowstate_topic:=/lowstate -p log_stick_on_every_message:=true
```

## Output Yang Ditampilkan

Node akan menampilkan:

- tombol yang sedang ditekan
- nilai raw bitmap tombol
- nilai stick hasil decode
- `tick` dari pesan `LowState`

Contoh log:

```text
Buttons: A, R1 (raw=0x0101) Sticks: lx= +12 ly=  -3 rx=  +0 ry= +18 tick=123456
```

## Ketergantungan

Package ini membutuhkan:

- `rclpy`
- `unitree_go`
- ROS 2 Humble atau environment ROS 2 lain yang menyediakan `unitree_go/msg/LowState`

## Ringkasnya

Data remote pada package ini diambil dari field `wireless_remote` milik pesan `unitree_go/msg/LowState` yang dipublish ke topic `/lowstate`. Node ini hanya membaca, mendecode, dan menampilkan data tersebut untuk keperluan monitoring.
