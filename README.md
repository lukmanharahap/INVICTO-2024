# INVICTO-2024

## Robot Vision System

### Deskripsi

Proyek ini menggunakan YOLO untuk mendeteksi bola dan silo dalam sistem robot. Data deteksi kemudian dikirim melalui serial ke STM32F4 untuk pengambilan keputusan robot.

### Fitur

- Deteksi bola dan silo menggunakan kamera utama.
- Deteksi keberadaan bola dalam robot menggunakan kamera internal.
- Penghitungan jarak dan sudut untuk navigasi.
- Komunikasi serial dengan STM32.

### Struktur Proyek

project/
│
├── src/
│ └── main.py
│
├── models/
│ └── ball_silo.pt
│
├── README.md
└── requirements.txt
