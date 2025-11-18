Okej — tu masz **najprostszy i najszybszy sposób** instalacji **TensorFlow Lite C++** na Raspberry Pi 3B+ (bez rekompilacji TensorFlow!).
To metoda używana w praktyce — działa stabilnie i wystarcza do aplikacji w C++ takich jak MoveNet.

---

# ✅ **Sposób 1: Instalacja gotowego pakietu `.deb` (najlepszy / najszybszy)**

Pobierasz **gotowe binaria TensorFlow Lite C++ dla Raspberry Pi** od Q-engineering — są lekkie i przetestowane.

### 1. Pobierz `.deb`:

```bash
wget https://github.com/Qengineering/TensorFlow-Lite-Raspberry-Pi/releases/download/v2.12.0/libtensorflowlite-2.12.0-armhf.deb
```

### 2. Zainstaluj:

```bash
sudo apt install ./libtensorflowlite-2.12.0-armhf.deb
```

### 3. Sprawdź, że pliki są zainstalowane:

Nagłówki:

```
/usr/local/include/tensorflow/lite/
```

Biblioteki:

```
/usr/local/lib/libtensorflow-lite.so
```

Gotowe ✔
Możesz już kompilować:

```bash
g++ -std=c++20 main.cpp -o movenet \
 `pkg-config --cflags --libs opencv4` \
 -I/usr/local/include/tensorflow-lite \
 -L/usr/local/lib -ltensorflow-lite -O2
```

---

# ✅ **Sposób 2: Instalacja z GitHub (skopiowanie nagłówków i biblioteki)**

Jeśli chcesz najnowszą wersję, możesz ściągnąć prekompilowane pliki:

```bash
wget https://github.com/tensorflow/tensorflow/raw/master/tensorflow/lite/tools/make/gen/rpi_armv7l/lib/libtensorflow-lite.a
```

I nagłówki:

```bash
git clone https://github.com/tensorflow/tensorflow.git
cp -r tensorflow/tensorflow/lite /usr/local/include/tensorflow/
```

Ale **nie polecam** tego początkującym — wersja `.deb` jest 10× prostsza.

---

# ❌ **Czego NIE robimy:**

* nie instalujemy TensorFlow pełnego (`pip install tensorflow`) → NIE DZIAŁA na RPi 3B+
* nie budujemy TensorFlow Lite z pełnych źródeł → trwa *4–8 godzin* i często kończy się błędem

---

# 🟢 **Podsumowanie w 10 sekund**

Jeśli chcesz mieć TensorFlow Lite C++ działające *już teraz*, na RPi 3B+:

```bash
wget https://github.com/Qengineering/TensorFlow-Lite-Raspberry-Pi/releases/download/v2.12.0/libtensorflowlite-2.12.0-armhf.deb
sudo apt install ./libtensorflowlite-2.12.0-armhf.deb
```

Gotowe.
Możesz kompilować MoveNet w C++ 🔥

---

Jeśli chcesz, mogę teraz przygotować:

* **CMakeLists.txt**,
* lub skrypt instalacyjny dla całego środowiska (OpenCV + TFLite + model MoveNet).
