# INVICTO-2024

## Robot Vision System

### Deskripsi

Proyek ini menggunakan YOLO untuk mendeteksi bola dan silo dalam sistem robot. Data deteksi kemudian dikirim melalui serial ke STM32F4 untuk pengambilan keputusan robot.

### Fitur

- Deteksi bola dan silo menggunakan kamera utama.
- Deteksi keberadaan bola di dalam robot menggunakan kamera internal.
- Penghitungan jarak dan sudut untuk navigasi.
- Komunikasi serial dengan STM32.

### Cara Menjalankan

1. **Clone repository:**
   ```bash
   git clone -b R2_VISION https://github.com/lukmanharahap/INVICTO-2024.git
   cd INVICTO-2024
   ```
2. **Set up virtual environment:**

   ```bash
   python -m venv R2_VISION
   source R2_VISION/bin/activate
   ```

3. **Install dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

4. **Run the code:**
   ```bash
   python src/main.py
   ```
