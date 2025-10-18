# Dokumentacja projektu IMU Rust

## 1. Wprowadzenie
Projekt udostępnia kompletne narzędzie CLI do pracy z czujnikiem inercyjnym BNO08X/BNO085 na Raspberry Pi. Program `imu` potrafi:

- szybko zweryfikować komunikację (podkomenda `check`) poprzez zrzut ramek SHTP,
- zestawić strumień danych w jednostkach SI (podkomenda `read`) z automatyczną obsługą resetów i metryk wydajności.

Cała logika sprzętowa znajduje się w module `src/imu.rs`, a `src/main.rs` odpowiada za interfejs użytkownika, logowanie i odczyt danych w formacie CSV.

## 2. Sprzęt i wymagania
- Raspberry Pi 3B+ (albo inny model z aktywnym `i2c-1`).
- Moduł z czujnikiem BNO08X/BNO085 pracujący w trybie I²C.
- Zasilanie 3.3 V – linie I²C w tym układzie nie tolerują 5 V.
- W systemie Raspberry trzeba wcześniej włączyć I²C (`raspi-config` → Interface Options → I2C).

### 2.1. Schemat połączeń I²C
![Raspberry Pi 3B+ pinout](rpi3bplus_pinout.png)

> Potrzebujesz ostrzejszego obrazu? Sięgnij po wariant WebP:  
> ![Pinout w wysokiej rozdzielczości](rpi3bplus_pinout_obrazek.webp)

| Raspberry Pi pin | Funkcja | Połączenie na BNO08X |
| ---------------- | ------- | -------------------- |
| 3.3 V (pin 1)    | VCC     | VIN / 3V3            |
| GND (pin 6)      | GND     | GND                  |
| GPIO2 (pin 3)    | I²C SDA | SDA                  |
| GPIO3 (pin 5)    | I²C SCL | SCL                  |
| (opcjonalnie)    | RST     | RESET / nRST         |
| (opcjonalnie)    | PS0/PS1 | GND dla trybu I²C    |

#### Kroki podłączenia
1. Wyłącz Raspberry Pi, zanim zaczniesz cokolwiek wpinać.
2. Podłącz 3.3 V i GND, upewniając się, że przewody są krótkie i dobrze osadzone.
3. Poprowadź linie SDA oraz SCL zgodnie z tabelą; długość przewodów najlepiej utrzymać poniżej 20 cm.
4. W razie potrzeby zewrzyj piny `PS0` i `PS1` do masy, żeby wymusić tryb I²C.
5. Pin RESET warto podłączyć do GPIO tylko wtedy, gdy planujesz sterowanie programowe.
6. Po włączeniu Raspberry sprawdź, czy czujnik odpowiada: `i2cdetect -y 1` powinien pokazać adres `0x4a`.

## 3. Konfiguracja oprogramowania
1. Zainstaluj Rust i Cargo przy pomocy `rustup`.
2. W katalogu repozytorium wykonaj `cargo build --release`, żeby zbudować binarkę `target/release/imu_rust`.
3. Zweryfikuj dostępność narzędzia i podkomend:
   - `cargo run -- --help`
   - `cargo run -- check --help`
   - `cargo run -- read --help`
4. Do testu komunikacji użyj `cargo run -- check --bus 1 --addr 0x4A`.
5. Strumień danych uruchom poleceniem `cargo run -- read --bus 1 --addr 0x4A --hz 100`.

## 4. Struktura oprogramowania

### 4.1. `src/main.rs` – warstwa CLI
- Wykorzystuje `clap` do obsługi dwóch podkomend: `check` i `read`.
- `check` otwiera wskazaną magistralę I²C, cyklicznie czyta nagłówki/payloady SHTP i wypisuje je w czytelnej formie, co pozwala szybko stwierdzić, czy połączenie działa.
- `read` uruchamia pełne `Imu::init`, konfiguruje raporty sensora, zbiera dane i wypisuje je na stdout jako CSV (`t, ax, ay, ...`). Jednocześnie na stderr lecą logi i metryki (co 5 s: liczba ramek, spadki, efektywne Hz, ostatni błąd).
- Obsługa sygnału `SIGINT` (`ctrlc`) pozwala na bezpieczne wyjście, a przy błędach program próbuje resetu sensora lub pełnej re-inicjalizacji z progresywnym backoffem.

### 4.2. `src/imu.rs` – sterownik wysokiego poziomu
- `Imu::init` tworzy `I2cInterface<I2cdev>` z `linux-embedded-hal`, wykonuje sekwencję bootstrap (soft reset, czyszczenie kolejek, weryfikacja Product ID) oraz włącza wymagane raporty SH-2 (rotacja, przyspieszenia, żyroskop).
- `poll_frame(timeout)` blokuje do momentu złożenia spójnej ramki (`Frame`) zawierającej orientację (kwaternion), przyspieszenia i prędkości kątowe. Dane są konwertowane z przestrzeni Q w SI.
- Wewnętrzne bufory i liczniki dbają o poprawną numerację sekwencji SHTP, a przy każdym pakiecie weryfikowany jest kanał (`Sensor`, `Hub control`, `Command`, itd.).
- Błędy I²C są mapowane na `ImuError`, co umożliwia warstwie CLI podejmowanie akcji naprawczych (reset, ponowne otwarcie magistrali).

## 5. Co dalej?
- Dodanie kolejnych raportów (np. kalibracja magnetometru) oraz konfiguracji czułości.
- Eksport danych do innych formatów (np. Parquet) lub streamowanie po sieci.
- Testy jednostkowe dla parserów raportów oraz testy integracyjne z mockiem I²C.
- Diagram przepływu danych i instrukcja integracji z usługą systemową (systemd, docker).
