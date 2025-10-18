# Dokumentacja narzędzi analitycznych — `test_logger.py` i `dir_classifier.py`

## 1. `test_logger.py` — analiza jakości strumienia IMU

### Cel

`test_logger.py` służy do **analizy jakości danych** z czujnika BNO085, odczytywanych przez program `imu_read`.
Pomaga ocenić:

* stabilność częstotliwości próbkowania (rzeczywisty Hz),
* jitter czasowy,
* ewentualne dropy ramek,
* charakter szumów i zakresy przyspieszeń / prędkości kątowych.

---

### Dane wejściowe

Plik CSV generowany przez `imu_read`:

```
t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk
```

* `t` — czas w sekundach od startu programu (monotoniczny),
* `ax..az` — przyspieszenie liniowe (m/s²),
* `gx..gz` — prędkość kątowa (rad/s),
* `qw..qk` — kwaternion orientacji (Game Rotation Vector).

---

### Użycie

```bash
# Analiza zapisanego CSV
python3 test_logger.py --input run.csv --expect-hz 100 --save-prefix run

# Wykonanie na żywo z potoku
cargo run --bin imu_read -- --hz 100 | tee run.csv | python3 test_logger.py --expect-hz 100

# Wyświetlenie interaktywne (wymaga GUI/X11)
python3 test_logger.py --input run.csv --expect-hz 100 --show
```

---

### Główne argumenty CLI

| Flaga           | Opis                                                   | Domyślna wartość |
| --------------- | ------------------------------------------------------ | ---------------- |
| `--input`       | Plik CSV lub `-` dla stdin                             | `-`              |
| `--expect-hz`   | Oczekiwana częstotliwość (Hz)                          | `100.0`          |
| `--save-prefix` | Zapis wykresów `<prefix>_acc.png`, `<prefix>_gyro.png` | brak             |
| `--show`        | Otwórz okna z wykresami (jeśli GUI)                    | wyłączone        |
| `--max-rows`    | Limit analizy (opcjonalnie)                            | brak             |

---

### Wyniki i raport

Po uruchomieniu skrypt wypisuje w terminalu:

```
=== IMU CSV Analysis ===
rows              : 6050
duration [s]      : 60.55
effective Hz      : 99.99
mean dt [ms]      : 10.002
jitter mean [ms]  : 0.002
jitter std  [ms]  : 0.048
drops (est)       : 3 (0.05%)
```

oraz tworzy wykresy:

* `|a|(t)` — moduł przyspieszenia,
* `|ω|(t)` — moduł prędkości kątowej.

---

### Obliczenia

* Δt = różnica kolejnych `t[i+1]-t[i]`
* mean(Δt), std(Δt) → **jitter**
* efektywny Hz = `n / (t_last - t_first)`
* spadki = `(oczekiwane - rzeczywiste) / oczekiwane`
* |a| = √(ax²+ay²+az²), |ω| = √(gx²+gy²+gz²)

---

### Interpretacja

* Hz ≈ wartość oczekiwana ⇒ stabilne taktowanie.
* Jitter std < 0.2 ms przy 100 Hz ⇒ bardzo dobre połączenie.
* Dropy < 1 % w 60 s ⇒ OK.
* Wykresy |a| i |ω| pozwalają ocenić szumy i dynamikę ruchu.

---

## 2. `dir_classifier.py` — rozpoznawanie kierunku ruchu

### Cel

`dir_classifier.py` służy do **rozpoznawania kierunku ruchu** (góra/dół/lewo/prawo/przód/tył/diagonalne) na podstawie danych z `imu_read`.
Wykorzystuje kwaternion orientacji (Game Rotation Vector) do przeliczenia przyspieszeń z układu czujnika na **układ świata**.

---

### Zasada działania

1. Z każdej próbki pobierane są: `ax,ay,az,gx,gy,gz,qw,qi,qj,qk`.
2. Obliczana jest macierz obrotu R z kwaternionu `q`.
3. `a_world = R * a_body` — przyspieszenie w układzie świata.
4. Ramki z dużą prędkością kątową (|ω| > `gyro_max`) są pomijane.
5. W oknie czasowym (~120 ms) szukany jest pik przyspieszenia.
6. Dominująca oś `a_world` → kierunek ruchu (z progiem `acc_thr`).
7. Jeśli dwie osie są silne (≥ 0.7 · max) ⇒ kierunek „diagonalny”.
8. Wynik jest drukowany, np.:

```
0.742315, , , , ,diagonal up-right
1.091220, , , , ,left
1.536880, , , , ,down
```

---

### Użycie

```bash
# Analiza z pliku
python3 dir_classifier.py --input run.csv --hz 100 --acc-thr 1.5

# Analiza na żywo (z potoku)
cargo run --bin imu_read -- --hz 100 | tee run.csv | python3 dir_classifier.py --hz 100
```

---

### Argumenty CLI

| Flaga             | Opis                                            | Domyślna wartość |
| ----------------- | ----------------------------------------------- | ---------------- |
| `--input`         | Plik CSV lub stdin                              | `-`              |
| `--acc-thr`       | Próg aktywacji gestu (m/s²)                     | `1.5`            |
| `--gyro-max`      | Maks. dozwolony moduł prędkości kątowej (rad/s) | `3.0`            |
| `--win-ms`        | Długość okna detekcji (ms)                      | `120`            |
| `--hz`            | Częstotliwość próbkowania (Hz)                  | `100`            |
| `--print-samples` | Wypisuje każdą próbkę (debug)                   | wyłączone        |

---

### Co robi krok po kroku

* Liczy obrót z GRV:

  > GRV = akcelerometr + żyroskop (bez magnetometru); daje stabilną orientację względem grawitacji.
* Obraca przyspieszenie `a_body` do `a_world`.
* Ignoruje próbki z rotacją (`|ω| > gyro_max`).
* Klasyfikuje kierunek:

  * `a_wz > +thr` → **up**, `a_wz < −thr` → **down**
  * `a_wx` → **right/left**
  * `a_wy` → **forward/back**
  * kombinacja dwóch silnych → **diagonal up-right** itd.

---

### Przykład wyjścia

```
# t, ax_w, ay_w, az_w, |a_w|, label
0.123400,0.014,0.012,2.431,2.431,up
0.324550,-2.210,0.023,0.215,2.210,left
0.548770,0.991,0.882,2.037,2.037,diagonal up-right
```

---

### Parametry i typowe progi

* `acc_thr = 1.5 m/s²` — gesty ręki/ramienia,
* `gyro_max = 3 rad/s` — odcina rotacje bez translacji,
* `win-ms = 120` — daje ~12 próbek przy 100 Hz.

---

### Interpretacja wyników

| Label               | Znaczenie                                     |
| ------------------- | --------------------------------------------- |
| `up` / `down`       | Ruch wzdłuż osi pionowej (świata)             |
| `left` / `right`    | Ruch w poziomie, względem osi X świata        |
| `forward` / `back`  | Ruch wzdłuż osi Y świata                      |
| `diagonal up-right` | Kombinacja ruchów w osi Z i X                 |
| brak etykiety       | Przyspieszenie poniżej progu lub sama rotacja |

---

### Uwaga praktyczna

* Kwaternion GRV nie ma ustalonego **yaw**, więc „forward/back” mogą dryfować — jeśli potrzebna stabilna mapa kierunków ciała, trzeba zapamiętać orientację początkową.
* Przy słabym oświetleniu magnetometru GRV i Linear Accel są wystarczające; Adafruit zaleca właśnie je dla gestów bez absolutnej orientacji.

---

### Przykładowe łączenie narzędzi

```bash
# Zbieranie danych
cargo run --bin imu_read -- --hz 100 > run.csv

# Analiza jakości (częstotliwość, jitter)
python3 test_logger.py --input run.csv --expect-hz 100 --save-prefix run

# Rozpoznawanie gestów kierunkowych
python3 dir_classifier.py --input run.csv --hz 100 --acc-thr 1.5 > dirs.csv
```

---

## 3. Literatura i odniesienia

* **Adafruit** — *9-DOF Orientation IMU Fusion Breakout - BNO085*: opis raportów, zalecenia dot. GRV, Linear Accel i 400 kHz I²C.
* **Bosch Sensortec** — *BNO08x / SH-2 Reference Manual*: formaty raportów, orientacja i fuzja czujników.
* Dane testowe z `imu_read` (Krok 1) — potwierdzają stabilność 100 Hz i poprawne kwaterniony GRV.
