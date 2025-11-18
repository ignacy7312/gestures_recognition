Poniżej masz **pełny przykład `main.cpp`**: C++20 + OpenCV + TensorFlow Lite + MoveNet Lightning (TFLite, int8), z rozpoznawaniem kierunku ruchu ręki (UP/DOWN/LEFT/RIGHT/FORWARD/BACKWARD).
Komentarze są rozbudowane, żebyś mógł to ogarnąć krok po kroku.

---

## Założenia i model

Używamy modelu MoveNet SinglePose Lightning w wersji TFLite INT8, np.:

* `lite-model_movenet_singlepose_lightning_tflite_int8_4.tflite`
  (wejście: `1×192×192×3` uint8, wyjście: `1×1×17×3` float32 – [y, x, score] dla 17 punktów ciała) ([hexdocs.pm][1])

Model pobierzesz np. z TF Hub / Kaggle (MoveNet SinglePose Lightning TFLite INT8). ([TensorFlow][2])

---

## Przykładowa kompilacja (na RPi / Ubuntu)

Zakładam, że masz:

* OpenCV (np. `libopencv-dev`)
* TensorFlow Lite C++ (nagłówki `tensorflow/lite/...` i biblioteka `-ltensorflow-lite`) – np. z prekompilowanego pakietu dla RPi: ([GitHub][3])

Przykładowa komenda (dostosuj ścieżki `-I` i `-L` do swojej instalacji TFLite):

```bash
g++ -std=c++20 main.cpp -o movenet_demo \
  `pkg-config --cflags --libs opencv4` \
  -I/usr/local/include/tensorflow-lite \
  -L/usr/local/lib -ltensorflow-lite
```

Na Raspberry Pi 3B+ koniecznie używaj **Release/O2** (np. `-O2` albo `-O3`).

---

## `main.cpp` – pełny kod

```cpp
// ============================================================================
// MoveNet SinglePose Lightning (TensorFlow Lite) + OpenCV
// Raspberry Pi 3B+ friendly pose estimation + rozpoznawanie ruchu ręki
//
// - kamera -> OpenCV VideoCapture
// - model MoveNet (TFLite int8), wejście: 1x192x192x3 (uint8), RGB
// - wyjście: 1x1x17x3 (float32): [y, x, score] dla 17 punktów (COCO keypoints)
//   Kolejność (17 punktów):
//     0: nose
//     1: left_eye
//     2: right_eye
//     3: left_ear
//     4: right_ear
//     5: left_shoulder
//     6: right_shoulder
//     7: left_elbow
//     8: right_elbow
//     9: left_wrist
//    10: right_wrist
//    11: left_hip
//    12: right_hip
//    13: left_knee
//    14: right_knee
//    15: left_ankle
//    16: right_ankle
//
// My:
// - bierzemy prawy bark (6), prawy łokieć (8), prawy nadgarstek (10)
// - wyznaczamy "koniec ręki" (preferujemy nadgarstek, jak nie ma, to łokieć)
// - śledzimy ruch tego punktu między klatkami
// - klasyfikujemy kierunek ruchu: UP / DOWN / LEFT / RIGHT / FORWARD / BACKWARD
//   * UP/DOWN/LEFT/RIGHT – z wektora (dx, dy) na ekranie
//   * FORWARD/BACKWARD – z przyrostu długości ramienia (osoba bliżej/dalej)
//
// Kompilacja (przykład):
//   g++ -std=c++20 main.cpp -o movenet_demo \
//       `pkg-config --cflags --libs opencv4` \
//       -I/usr/local/include/tensorflow-lite \
//       -L/usr/local/lib -ltensorflow-lite -O2
//
// Uwaga: ścieżki -I/-L dostosuj do swojej instalacji TensorFlow Lite.
//
// Uruchomienie:
//   ./movenet_demo
//
// Wyjście:
//   - okno "MoveNet Hand Motion" z obrazem z kamery
//   - narysowany szkielet ramion (prawa ręka)
//   - tekst "Direction: ..." z aktualnym kierunkiem ruchu
//   - w konsoli logi dx, dy, dLen, direction
// ============================================================================

#include <opencv2/opencv.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

// --------------------------- KONFIGURACJA ----------------------------------

// Ścieżka do modelu MoveNet (TFLite int8, SinglePose Lightning).
// Pobierz np. lite-model_movenet_singlepose_lightning_tflite_int8_4.tflite
// i umieść w katalogu obok main.cpp.
const std::string MODEL_PATH = "lite-model_movenet_singlepose_lightning_tflite_int8_4.tflite";

// Wejście modelu (wg karty MoveNet):
// 1 x 192 x 192 x 3, typ uint8, kanały: RGB
constexpr int TFLITE_BATCH   = 1;
constexpr int TFLITE_HEIGHT  = 192;
constexpr int TFLITE_WIDTH   = 192;
constexpr int TFLITE_CHANNEL = 3;

// Ilość punktów i współrzędnych na punkt (y, x, score).
constexpr int NUM_KEYPOINTS = 17;
constexpr int KP_CHANNELS   = 3;

// Minimalne prawdopodobieństwo (score) uznania punktu za wiarygodny.
constexpr float MIN_KP_SCORE = 0.3f;

// Progi do klasyfikacji kierunku ruchu (w pikselach).
constexpr float MIN_2D_MOVEMENT_PX = 5.0f;   // minimalny ruch w 2D (x,y)
constexpr float MIN_LENGTH_CHANGE  = 10.0f;  // minimalna zmiana długości ramienia

// Współczynnik wygładzania pozycji końca ręki (0.0 = brak, 0.9 = bardzo mocne).
constexpr float SMOOTHING_ALPHA = 0.7f;

// Indeksy kluczowych punktów dla prawej ręki (zgodnie z kolejnością MoveNet).
constexpr int KP_LEFT_SHOULDER  = 5;
constexpr int KP_RIGHT_SHOULDER = 6;
constexpr int KP_LEFT_ELBOW     = 7;
constexpr int KP_RIGHT_ELBOW    = 8;
constexpr int KP_LEFT_WRIST     = 9;
constexpr int KP_RIGHT_WRIST    = 10;

// --------------------------- STRUKTURY -------------------------------------

// Struktura na jeden keypoint (współrzędne w pikselach + score + flaga ważności).
struct Keypoint {
    cv::Point2f pos;   // pozycja na obrazie (w pikselach)
    float score;       // pewność [0,1]
    bool valid;        // czy powyżej progu
};

// Prosty pomocnik do liczenia odległości Euklidesowej.
float distance(const cv::Point2f& a, const cv::Point2f& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Wygładzanie pozycji punktu (filtr jednoprzebiegowy: IIR 1 rzędu).
cv::Point2f smoothPoint(const cv::Point2f& prevFiltered,
                        const cv::Point2f& current,
                        float alpha)
{
    // alpha blisko 1.0 -> mocno trzymamy poprzednią wartość
    // alpha blisko 0.0 -> prawie w ogóle nie wygładzamy
    return cv::Point2f(
        alpha * prevFiltered.x + (1.0f - alpha) * current.x,
        alpha * prevFiltered.y + (1.0f - alpha) * current.y
    );
}

// Klasyfikacja kierunku ruchu na podstawie:
//  - dx, dy: zmiana położenia końca ręki w pikselach
//  - dLen: zmiana długości ramienia (bark -> koniec ręki) w pikselach
//
// Logika (prosta, edukacyjna):
//  1. Jeśli ruch znikomy w 2D i znikoma zmiana długości -> "NONE".
//  2. Jeśli |dLen| dominuje nad |dx| i |dy| oraz przekracza próg:
//        dLen > 0 -> "FORWARD"  (ramię "wydłuża się" -> osoba bliżej kamery)
//        dLen < 0 -> "BACKWARD" (ramię "skraca się"  -> osoba dalej)
//  3. W pozostałych przypadkach:
//        |dx| > |dy| -> LEFT/RIGHT
//        |dy| >=|dx| -> UP/DOWN
//     Pamiętaj: w OpenCV oś Y rośnie w dół, więc dy>0 = ruch w dół.
std::string classifyDirection(float dx, float dy, float dLen)
{
    float movement2D = std::sqrt(dx * dx + dy * dy);

    // 0. Bardzo mały ruch -> ignorujemy.
    if (movement2D < MIN_2D_MOVEMENT_PX && std::fabs(dLen) < MIN_LENGTH_CHANGE) {
        return "NONE";
    }

    // 1. Sprawdź, czy zmiana długości ramienia dominuje.
    if (std::fabs(dLen) > std::fabs(dx) &&
        std::fabs(dLen) > std::fabs(dy) &&
        std::fabs(dLen) > MIN_LENGTH_CHANGE)
    {
        if (dLen > 0.0f) {
            return "FORWARD";
        } else {
            return "BACKWARD";
        }
    }

    // 2. Ruch w płaszczyźnie obrazu.
    if (std::fabs(dx) > std::fabs(dy)) {
        // dominujący ruch poziomy
        return (dx > 0.0f) ? "RIGHT" : "LEFT";
    } else {
        // dominujący ruch pionowy
        return (dy > 0.0f) ? "DOWN" : "UP";
    }
}

// --------------------------- TFLITE SETUP ----------------------------------

// Tworzy interpreter TensorFlow Lite i ładuje model z pliku .tflite.
std::unique_ptr<tflite::Interpreter> createInterpreter(const std::string& modelPath)
{
    // Wczytanie modelu TFLite z pliku do obiektu FlatBufferModel.
    auto model = tflite::FlatBufferModel::BuildFromFile(modelPath.c_str());
    if (!model) {
        std::cerr << "Blad: nie udalo sie wczytac modelu TFLite z pliku: "
                  << modelPath << std::endl;
        return nullptr;
    }

    // Rejestr wbudowanych operatorów (Conv2D, Relu, DepthwiseConv, itp.)
    tflite::ops::builtin::BuiltinOpResolver resolver;

    // Utworzenie interpretera (silnika wykonującego model).
    std::unique_ptr<tflite::Interpreter> interpreter;
    tflite::InterpreterBuilder(*model, resolver)(&interpreter);
    if (!interpreter) {
        std::cerr << "Blad: nie udalo sie utworzyc interpretera TFLite.\n";
        return nullptr;
    }

    // (Opcjonalnie) ustawiamy liczbę wątków CPU (na RPi 3 sensownie 2).
    interpreter->SetNumThreads(2);

    // Alokacja pamięci na tensory (wejścia/wyjścia).
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        std::cerr << "Blad: nie udalo sie zaalokowac tensorow TFLite.\n";
        return nullptr;
    }

    return interpreter;
}

// --------------------------- GŁÓWNY PROGRAM --------------------------------

int main()
{
    // 1. Otwieramy kamerę (domyślnie /dev/video0).
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Blad: nie udalo sie otworzyc kamery.\n";
        return 1;
    }

    // Na Raspberry Pi 3B+ warto zejść z rozdzielczością,
    // np. do 640x480 albo 320x240 (przykład: 640x480).
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // 2. Tworzymy interpreter TFLite i ladujemy model MoveNet.
    std::unique_ptr<tflite::Interpreter> interpreter = createInterpreter(MODEL_PATH);
    if (!interpreter) {
        std::cerr << "Blad: nie udalo sie zainicjalizowac interpretera (sprawdz MODEL_PATH).\n";
        return 1;
    }

    // Sprawdźmy krótko tensor wejściowy.
    TfLiteTensor* inputTensor = interpreter->input_tensor(0);
    if (!inputTensor) {
        std::cerr << "Blad: brak tensora wejsciowego.\n";
        return 1;
    }

    // Spodziewamy sie: typ uint8, shape [1, 192, 192, 3].
    if (inputTensor->type != kTfLiteUInt8) {
        std::cerr << "Ostrzezenie: tensor wejsciowy nie jest typu UINT8 (jest: "
                  << inputTensor->type << "). Model moze byc inny niz INT8.\n";
    }

    if (inputTensor->dims->size != 4 ||
        inputTensor->dims->data[0] != TFLITE_BATCH ||
        inputTensor->dims->data[1] != TFLITE_HEIGHT ||
        inputTensor->dims->data[2] != TFLITE_WIDTH ||
        inputTensor->dims->data[3] != TFLITE_CHANNEL)
    {
        std::cerr << "Ostrzezenie: niespodziewane wymiary tensora wejsciowego.\n";
    }

    // Tensor wyjsciowy (keypointy).
    TfLiteTensor* outputTensor = interpreter->output_tensor(0);
    if (!outputTensor) {
        std::cerr << "Blad: brak tensora wyjsciowego.\n";
        return 1;
    }
    if (outputTensor->type != kTfLiteFloat32) {
        std::cerr << "Ostrzezenie: tensor wyjsciowy nie jest typu FLOAT32 (jest: "
                  << outputTensor->type << ").\n";
    }

    // Stan do śledzenia ruchu ręki.
    bool hasPrev = false;
    cv::Point2f prevFilteredTip(0.0f, 0.0f);
    float prevArmLength = 0.0f;
    std::string lastDirection = "NONE";

    // Główna pętla: czytamy klatki z kamery, puszczamy MoveNet, liczymy ruch.
    while (true) {
        cv::Mat frameBGR;
        cap >> frameBGR;
        if (frameBGR.empty()) {
            std::cerr << "Blad: pusta klatka z kamery.\n";
            break;
        }

        int frameWidth  = frameBGR.cols;
        int frameHeight = frameBGR.rows;

        // 3. Przygotowanie obrazu wejściowego dla MoveNet:
        //    - konwersja BGR -> RGB
        //    - zmiana rozmiaru do 192x192
        //    - typ: CV_8UC3 (8-bitowe, 3 kanały)
        cv::Mat frameRGB;
        cv::cvtColor(frameBGR, frameRGB, cv::COLOR_BGR2RGB);

        cv::Mat inputResized;
        cv::resize(frameRGB, inputResized, cv::Size(TFLITE_WIDTH, TFLITE_HEIGHT));

        if (inputResized.type() != CV_8UC3) {
            inputResized.convertTo(inputResized, CV_8UC3);
        }

        // 4. Skopiowanie danych z obrazu do tensora wejściowego TFLite.
        //    Tensor ma kształt [1,192,192,3], więc ilość elementów to 192*192*3.
        size_t inputBytes = TFLITE_HEIGHT * TFLITE_WIDTH * TFLITE_CHANNEL * sizeof(uint8_t);
        // Uwaga: OpenCV trzyma dane w pamięci wierszami, w sposób zgodny (kontiguity),
        // więc możemy użyć memcpy, jeśli macierz jest ciągła.
        if (!inputResized.isContinuous()) {
            inputResized = inputResized.clone();
        }

        std::memcpy(inputTensor->data.raw, inputResized.data, inputBytes);

        // 5. Uruchomienie modelu (inference).
        if (interpreter->Invoke() != kTfLiteOk) {
            std::cerr << "Blad: Invoke() TFLite nie powiodl sie.\n";
            break;
        }

        // 6. Odczyt wyniku:
        //    MoveNet lightning (tflite int8) typowo zwraca: [1,1,17,3] float32.
        //    Dla kazdego punktu: [y_norm, x_norm, score].
        const float* outputData = outputTensor->data.f;

        // Tworzymy wektor 17 Keypointów w układzie współrzędnych obrazu (w pikselach).
        std::vector<Keypoint> keypoints(NUM_KEYPOINTS);
        for (int i = 0; i < NUM_KEYPOINTS; ++i) {
            float y_norm = outputData[i * KP_CHANNELS + 0]; // 0..1
            float x_norm = outputData[i * KP_CHANNELS + 1];
            float score  = outputData[i * KP_CHANNELS + 2];

            // Przeskalowanie współrzędnych z [0,1] do rozmiaru oryginalnej klatki.
            // (Tu ignorujemy różnice aspektu – dla prostego demo jest to OK.)
            float x = x_norm * static_cast<float>(frameWidth);
            float y = y_norm * static_cast<float>(frameHeight);

            keypoints[i].pos   = cv::Point2f(x, y);
            keypoints[i].score = score;
            keypoints[i].valid = (score >= MIN_KP_SCORE);
        }

        // 7. Szukamy prawej ręki (right_shoulder, right_elbow, right_wrist).
        Keypoint shoulder, elbow, wrist;
        bool hasRightShoulder = keypoints[KP_RIGHT_SHOULDER].valid;
        bool hasRightElbow    = keypoints[KP_RIGHT_ELBOW].valid;
        bool hasRightWrist    = keypoints[KP_RIGHT_WRIST].valid;

        bool armOk = false;
        if (hasRightShoulder && (hasRightElbow || hasRightWrist)) {
            shoulder = keypoints[KP_RIGHT_SHOULDER];
            elbow    = keypoints[KP_RIGHT_ELBOW];
            wrist    = keypoints[KP_RIGHT_WRIST];
            armOk    = true;
        } else {
            // Opcja awaryjna: spróbuj lewej ręki, jeśli prawa jest niewidoczna.
            bool hasLeftShoulder = keypoints[KP_LEFT_SHOULDER].valid;
            bool hasLeftElbow    = keypoints[KP_LEFT_ELBOW].valid;
            bool hasLeftWrist    = keypoints[KP_LEFT_WRIST].valid;
            if (hasLeftShoulder && (hasLeftElbow || hasLeftWrist)) {
                shoulder = keypoints[KP_LEFT_SHOULDER];
                elbow    = keypoints[KP_LEFT_ELBOW];
                wrist    = keypoints[KP_LEFT_WRIST];
                armOk    = true;
            }
        }

        cv::Point2f armTip(-1.0f, -1.0f);

        if (armOk) {
            // Preferujemy nadgarstek jako "koniec ręki".
            if (wrist.valid) {
                armTip = wrist.pos;
            } else if (elbow.valid) {
                armTip = elbow.pos;
            }

            // Pozycja barku (początek ramienia).
            cv::Point2f shoulderPos = shoulder.pos;

            // Aktualna długość ramienia (bark -> koniec).
            float armLength = distance(shoulderPos, armTip);

            if (!hasPrev) {
                // Pierwsza sensowna klatka.
                prevFilteredTip = armTip;
                prevArmLength   = armLength;
                hasPrev         = true;
                lastDirection   = "NONE";
            } else {
                // Wygładzenie pozycji końca ręki.
                cv::Point2f filteredTip = smoothPoint(prevFilteredTip, armTip, SMOOTHING_ALPHA);

                // Zmiana pozycji w pikselach (dx, dy).
                float dx = filteredTip.x - prevFilteredTip.x;
                float dy = filteredTip.y - prevFilteredTip.y;

                // Zmiana długości ramienia.
                float dLen = armLength - prevArmLength;

                // Klasyfikacja kierunku.
                std::string direction = classifyDirection(dx, dy, dLen);
                if (direction != "NONE") {
                    lastDirection = direction;
                }

                // Debug na konsoli.
                std::cout << "dx=" << dx
                          << " dy=" << dy
                          << " dLen=" << dLen
                          << " -> direction=" << direction << std::endl;

                // Aktualizacja historii.
                prevFilteredTip = filteredTip;
                prevArmLength   = armLength;
            }

            // 8. Rysowanie szkieletu ramienia na obrazie.

            // Bark.
            cv::circle(frameBGR, shoulder.pos, 5, cv::Scalar(0, 255, 255), cv::FILLED);

            // Łokieć (jeśli ważny).
            if (elbow.valid) {
                cv::circle(frameBGR, elbow.pos, 5, cv::Scalar(0, 255, 0), cv::FILLED);
                cv::line(frameBGR, shoulder.pos, elbow.pos, cv::Scalar(255, 0, 0), 2);
            }

            // Nadgarstek (jeśli ważny).
            if (wrist.valid) {
                cv::circle(frameBGR, wrist.pos, 5, cv::Scalar(0, 0, 255), cv::FILLED);
                if (elbow.valid) {
                    cv::line(frameBGR, elbow.pos, wrist.pos, cv::Scalar(255, 0, 0), 2);
                } else {
                    cv::line(frameBGR, shoulder.pos, wrist.pos, cv::Scalar(255, 0, 0), 2);
                }
            }

        } else {
            // Nie mamy wiarygodnej ręki w tej klatce – resetujemy stan.
            hasPrev       = false;
            lastDirection = "NONE";
            std::cout << "Info: brak wiarygodnej reki w tej klatce.\n";
        }

        // 9. Wyświetlenie kierunku ruchu na obrazie.
        std::string text = "Direction: " + lastDirection;
        cv::putText(frameBGR, text, cv::Point(30, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        // (Opcjonalnie) możemy narysować wszystkie keypointy do debugowania.
        for (int i = 0; i < NUM_KEYPOINTS; ++i) {
            if (keypoints[i].valid) {
                cv::circle(frameBGR, keypoints[i].pos, 3, cv::Scalar(255, 255, 0), cv::FILLED);
            }
        }

        // 10. Pokazujemy obraz w oknie.
        cv::imshow("MoveNet Hand Motion", frameBGR);

        char c = static_cast<char>(cv::waitKey(1));
        if (c == 'q' || c == 27) { // 'q' lub ESC
            break;
        }
    }

    return 0;
}
```

---

## Co tu jest ważne z perspektywy RPi 3B+?

**1. Lekki model**

* MoveNet Lightning jest projektowany właśnie na urządzenia mobilne / edge, działa dużo szybciej niż OpenPose. ([TensorFlow Blog][4])

**2. Małe wejście**

* Zawsze 192×192×3 (RGB, uint8) – mało danych → tanie obliczeniowo. ([MathWorks][5])

**3. Quantization (int8/uint8)**

* Model jest zkwantyzowany → operuje na 8-bitach → mniejszy, szybszy na ARM/NEON.

**4. OpenCV na zewnątrz, TFLite w środku**

* OpenCV ogarnia kamerę, rysowanie, resize.
* TensorFlow Lite ogarnia tylko inferencję modelu (TensorFlow Lite C++ API). ([fdmcs.math.cnrs.fr][6])

---

## Krótkie podsumowanie działania algorytmu

1. Z kamery pobierasz klatkę (BGR).
2. Konwertujesz do RGB, skalujesz do 192×192, kopiujesz do tensora TFLite.
3. Wołasz `interpreter->Invoke()` – model MoveNet liczy 17 kluczowych punktów.
4. Wyjście (1×1×17×3 float) zamieniasz na `std::vector<Keypoint>` w pikselach.
5. Dla ramienia:

   * bierzesz bark + łokieć + nadgarstek (prawa ręka, ewentualnie lewa),
   * liczysz długość ramienia i śledzisz koniec ręki w czasie,
   * z różnic dx, dy, dLen klasyfikujesz UP/DOWN/LEFT/RIGHT/FORWARD/BACKWARD.
6. Na obraz nakładasz skeleton + tekst „Direction: ...”.

---

Jeśli chcesz, w kolejnym kroku mogę:

* dorzucić **CMakeLists.txt** pod RPi,
* rozpisać dokładnie **instalację TensorFlow Lite C++** *konkretnie* pod Raspberry Pi 3B+ krok po kroku (np. na bazie prekompilowanego `tensorflow-lite.deb` albo build-from-source z QEngineering).

[1]: https://hexdocs.pm/tfl_interp/movenet.html?utm_source=chatgpt.com "Pose Estimation: MoveNet — tfl_interp v0.1.16"
[2]: https://www.tensorflow.org/hub/tutorials/movenet?utm_source=chatgpt.com "MoveNet: Ultra fast and accurate pose detection model."
[3]: https://github.com/prepkg/tensorflow-lite-raspberrypi?utm_source=chatgpt.com "prepkg/tensorflow-lite-raspberrypi"
[4]: https://blog.tensorflow.org/2021/08/pose-estimation-and-classification-on-edge-devices-with-MoveNet-and-TensorFlow-Lite.html?utm_source=chatgpt.com "Pose estimation and classification on edge devices with ..."
[5]: https://www.mathworks.com/help/coder/ug/deploy-tflite-pose-estimation.html?utm_source=chatgpt.com "Deploy Pose Estimation Application Using TensorFlow Lite ..."
[6]: https://fdmcs.math.cnrs.fr/gerardo.granados/tensorflow/-/blob/v1.15.0-rc2/tensorflow/lite/g3doc/guide/inference.md?utm_source=chatgpt.com "tensorflow/lite/g3doc/guide/inference.md"
