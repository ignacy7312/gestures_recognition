# Architektura `imu_rust`

## 1. Widok ogólny
`imu_rust` to binarno-biblioteczny crate w Rust napisany dla Raspberry Pi 3 z czujnikiem BNO085 na magistrali I²C. Crate dzieli się na trzy warstwy:

- **Warstwa narzędzi diagnostycznych** (`src/main.rs`) – prosty „SHTP dump”, przydatny na etapie weryfikacji połączenia.
- **Warstwa biblioteczna** (`src/imu.rs`, re-eksportowana w `src/lib.rs`) – enkapsuluje całe sterowanie układem BNO085: inicjalizację, włączanie raportów, parsowanie ramek i odzyskiwanie po błędach.
- **Warstwa CLI** (`src/bin/imu_read.rs`) – dostarcza kompletną aplikację strumieniującą CSV z przyrostkiem metryk i obsługą sygnałów.

Pozostałe pliki koncentrują się na konfiguracji (`Cargo.toml`, `Cargo.lock`) oraz dokumentacji operacyjnej (`README-dev.md`).

---

## 2. Warstwa biblioteczna (`src/imu.rs`)
### 2.1 Stałe i skale numeryczne
- **Stałe kanałów i raportów** (`CHANNEL_*`, `SENSOR_REPORTID_*`, `SHUB_*`) – mapują wartości z dokumentacji SH-2/BNO08X na nazwy używane w kodzie.
- **Bufory** (`PACKET_*`, `NUM_CHANNELS`) – rozmiary tablic wykorzystywanych podczas wymiany pakietów SHTP.
- **Skale Q** (`Q8_SCALE`, `Q9_SCALE`, `Q14_SCALE`) – przeliczają wartości stałoprzecinkowe z raportów na liczby zmiennoprzecinkowe.

### 2.2 Struktury i typy
- **`Frame`** – publiczna reprezentacja ramki pomiarowej (czas `t`, akceleracje liniowe `ax..az`, prędkości kątowe `gx..gz`, kwaternion `qw..qk`).
- **`ImuConfig`** – publiczny kontener konfiguracji (magistrala I²C, adres urządzenia, częstotliwość raportów). Metoda `device_path()` odwzorowuje numer magistrali na nazwę pliku `/dev/i2c-*`.
- **`ImuError`** – publiczne enum błędów: obejmuje warstwę I²C (`Bus`), błędy protokołu `bno080` (`Comm`), time-outy, reset czujnika, problemy z Product ID oraz ogólne naruszenia protokołu.
- **`ValueSlot<T>`** *(wewnętrzna)* – bufor przechowujący ostatnią wartość i numer raportu dla konkretnego sensora.
- **`SensorState`** *(wewnętrzna)* – agreguje buforowane odczyty (akceleracja liniowa / absolutna / żyroskop / kwaternion) oraz flagi kontrolne (otrzymanie advertisement, init, PID, reset, ostatni błąd).
- **`FrameMarkers`** *(wewnętrzna)* – zapisuje identyfikatory ostatnio zbudowanych pomiarów, by wykrywać powtórzenia/duble.
- **`Imu`** – główny obiekt sterujący życiem czujnika.

Typ pomocniczy `impl From<BnoError<LinuxI2CError, ()>> for ImuError` pozwala na wygodne mapowanie błędów warstwy `bno080` do `ImuError::Comm`.

### 2.3 Publiczne metody `Imu`
- **`Imu::init(config)`** – otwiera `/dev/i2c-*`, tworzy `I2cInterface`, resetuje stan wewnętrzny przez `bootstrap()` i włącza domyślne raporty (`enable_reports`).
- **`enable_reports(hz)`** – przelicza zadaną częstotliwość na interwał mikrosekund i konfiguruje Game Rotation Vector, Linear Acceleration, klasyczny Accelerometer oraz Gyroscope poprzez `send_feature_command`.
- **`poll_frame(timeout)`** – główna pętla pobierania danych: czyta pakiety z interfejsu, aktualizuje stan, buduje kompletną ramkę i zwraca ją. W przypadku przekroczenia `timeout` zwraca `ImuError::Timeout`; jeśli BNO085 sygnalizuje reset – `ImuError::SensorReset`.
- **`handle_reset()`** – stosuje sekwencję `bootstrap()` + ponowne włączenie raportów i wyczyszczenie buforów pomiarów.
- **`last_error()`** – raportuje ostatni kod błędu zwrócony przez układ (przechwycony z kanału command).

### 2.4 Sekwencje inicjalizacji i komunikacji
- **`bootstrap()`** – „zimny start”: czyści stan, wywołuje `I2cInterface::setup`, wykonuje miękki reset (`soft_reset`), konsumuje wiadomości startowe (`eat_all_messages`), weryfikuje Product ID (`verify_product_id`) i ustawia zegar odniesienia.
- **`soft_reset()`** – wysyła polecenie resetu na kanał executable, przetwarza ewentualną odpowiedź (ustawia flagę `pending_reset`).
- **`verify_product_id()`** – wysyła żądanie `SHUB_PROD_ID_REQ`, a następnie oczekuje na odpowiedź `SHUB_PROD_ID_RESP`; w przypadku braku odzewu do 500 ms zgłasza `ImuError::ProductId`.
- **`send_feature_command(report_id, interval_us)`** – buduje raport `SHUB_REPORT_SET_FEATURE_CMD` dla wskazanego raportu i zleca jego wysłanie.
- **`send_packet()` / `send_and_receive_packet()` / `prep_send_packet()`** – pomocnicze funkcje do budowy i wysyłki pakietów SHTP (ustawiają nagłówki, numerację sekwencji itp.).

### 2.5 Obsługa kolejek wiadomości
- **`poll_frame()`** używa zestawu metod pomocniczych do odczytu:
  - `receive_packet_with_timeout()` – zdejmuje jeden pakiet SHTP (lub zero przy time-oucie) wykorzystując `SensorInterface::read_with_timeout`.
  - `handle_received_packet(len)` – demultipleksuje pakiet po kanale i kieruje go do odpowiednich handlerów.
  - `handle_command_channel()` – śledzi listę błędów (`CMD_RESP_ERROR_LIST`) oraz flagę otrzymania advertisement (`CMD_RESP_ADVERTISEMENT`).
  - `handle_exec_channel()` – rejestruje komunikat resetu (`EXECUTABLE_DEVICE_RESP_RESET_COMPLETE`).
  - `handle_hub_control()` – interpretuje odpowiedzi systemowe (`SHUB_COMMAND_RESP`, `SHUB_PROD_ID_RESP`).
  - `handle_sensor_reports()` – parsuje payload z kanału sensorów: iteruje po raportach, zamienia wartości stałoprzecinkowe na `f32`, aktualizuje `SensorState`. Obsługuje raporty kwaternionu (Game / zwykły), akceleracji liniowej / absolutnej oraz żyroskopu.
  - `bump_report_counter()` – monotonizuje identyfikatory raportów, zapobiega przepełnieniom do zera.

### 2.6 Budowanie ramek pomiarowych
- **`try_build_frame()`** – sprawdza, czy dostępne są świeże kwaterniony i odczyt z żyroskopu oraz akcelerometru (liniowy preferowany nad absolutnym). Jeśli tak – konstruuje obiekt `Frame` wraz z czasem monotonicznym.
- **`select_accel_slot()`** – logika wyboru najlepszego zestawu akceleracji (liniowa, w razie braku – absolutna).
- **`clear_measurements()`** – resetuje buforowane wartości i markery ramek.

### 2.7 Funkcje narzędziowe
- **`q8_to_f32`, `q9_to_f32`, `q14_to_f32`** – konwersje wartości w kodowaniu Q do metryk SI.
- **`eat_all_messages(max_iterations)`** – agresywnie konsumuje oczekujące pakiety (używane po resecie).
- **`handle_one_message(timeout_ms)`** – pomocnicza obsługa pojedynczej wiadomości; zwraca `1`, jeśli coś przetworzono.

---

## 3. Warstwa CLI (`src/bin/imu_read.rs`)
### 3.1 Konfiguracja i parametry
- **`Args`** – definicja CLI przy pomocy `clap`: `--bus`, `--addr`, `--hz`, `--timeout-ms`, `--no-header`, `--log-level`.
- **`LogLevel`** – własna enum stanowiąca mapowanie na `LevelFilter`; `init_logging()` konfiguruje `env_logger`.

### 3.2 Główna pętla `main()`
1. Parsuje flagi, uruchamia logger i składa `ImuConfig`.
2. Wywołuje `Imu::init` i rejestruje handler `ctrlc` ustawiający atomową flagę `running`.
3. Przygotowuje `BufWriter` na `stdout`, drukuje nagłówek CSV (chyba że `--no-header`).
4. Wylicza interwał próbkowania i rozpoczyna pętlę:
   - czeka do następnej próbki (`next_tick`),
   - woła `imu.poll_frame(timeout)`; sukces skutkuje zapisem wiersza CSV,
   - obsługuje różne kategorie błędów (Timeout, SensorReset, Comm, Bus, ProductId, Protocol) – każda aktualizuje licznik dropów, loguje opis i uruchamia odpowiednią sekwencję odzyskiwania (`handle_reset` lub `recover_imu`).
5. Co 5 s (`METRICS_INTERVAL`) loguje metryki: sumaryczne i oknowe liczby ramek/dropów, częstotliwość i opis ostatniego błędu.
6. Po wyjściu z pętli (SIGINT) spuszcza bufor i loguje statystyki końcowe.

### 3.3 Funkcje pomocnicze
- **`parse_hex_u8()`** – parser wartości szesnastkowych używany przez flagę `--addr`.
- **`recover_imu(config, running)`** – pętla ponownej inicjalizacji czujnika z wykładniczym backoffem; przerwana, jeśli w międzyczasie ustawiono `running=false`.

---

## 4. Narzędzie diagnostyczne (`src/main.rs`)
Program wypisuje surowe pakiety SHTP po I²C:
1. Otwiera `/dev/i2c-1` i wykonuje krótką pauzę synchronizacyjną.
2. W pętli czyta 4-bajtowy nagłówek (`ShtpHeader::parse`), waliduje długości i wczytuje payload.
3. Loguje metadane o każdej ramce i pierwszych bajtach payloadu, co pomaga potwierdzić „życie” układu po stronie sprzętowej.

---

## 5. Przepływ błędów i odzyskiwanie
1. **Warstwa I²C** – błędy `LinuxI2CError` są zamieniane na `ImuError::Bus` i skutkują pełną re-inicjalizacją (`recover_imu`).
2. **Warstwa SHTP/BNO080** – błędy zwrócone przez crate `bno080` propagują się jako `ImuError::Comm`; CLI próbuje wykonać `handle_reset`, a przy niepowodzeniu przechodzi do `recover_imu`.
3. **Reset sprzętowy** – gdy kanał executable sygnalizuje `EXECUTABLE_DEVICE_RESP_RESET_COMPLETE`, biblioteka ustawia flagę, `poll_frame` zwraca `ImuError::SensorReset`, a CLI inicjuje sekwencję od nowa.
4. **Czas oczekiwania** – jeśli w zadanym czasie nie nadejdzie żadna ramka, `poll_frame` zgłasza `ImuError::Timeout`; CLI liczy to jako drop, ale kontynuuje pracę bez restartu.
5. **Błędy protokołu** – naruszenia struktury pakietu prowadzą do `ImuError::Protocol` i dla bezpieczeństwa wymuszają reset/reinit.

---

## 6. Kluczowe pliki i ich rola
| Ścieżka                      | Rola                                                                 |
|-----------------------------|-----------------------------------------------------------------------|
| `Cargo.toml`                | Deklaruje zależności (`bno080`, `linux-embedded-hal`, `clap`, `ctrlc`, `env_logger`, `thiserror`) i binarny target `imu_read`.
| `src/lib.rs`                | Udostępnia moduł `imu` oraz publiczne typy (`Frame`, `Imu`, `ImuConfig`, `ImuError`). |
| `src/imu.rs`                | Cała logika komunikacji z BNO085 i budowy ramek.                      |
| `src/bin/imu_read.rs`       | Aplikacja CLI do strumieniowania CSV oraz logowania metryk.           |
| `src/main.rs`               | Narzędzie do podglądu surowego SHTP (debug).                          |
| `README-dev.md`             | Skrócone instrukcje uruchomienia, flag i oczekiwanego outputu.        |
| `docs/ARCHITEKTURA_IMU.md`  | Niniejszy dokument szczegółowej architektury i odpowiedzialności kodu.|

---

## 7. Jak korzystać z dokumentacji
1. Zacznij od sekcji 1, aby zrozumieć komponenty projektu.
2. Sekcja 2 prowadzi przez warstwę biblioteczną – polecana dla osób rozszerzających API.
3. Sekcja 3 opisuje zachowanie CLI – przydatna przy integracji w pipeline.
4. Sekcja 4 opisuje narzędzia diagnostyczne do debugowania warstwy sprzętowej.
5. Sekcja 5 zawiera wiedzę o ścieżkach błędów i odzyskiwaniu – ważne przy pracy w warunkach produkcyjnych.

