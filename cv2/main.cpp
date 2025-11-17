// ============================================================================
// Minimalny przykład: rozpoznawanie ruchu ręki (UP/DOWN/LEFT/RIGHT/FORWARD/BACKWARD)
// w C++20 z użyciem OpenCV i modelu OpenPose (DNN).
//
// Kompilacja (Linux, OpenCV4):
//   g++ -std=c++20 main.cpp -o hand_motion `pkg-config --cflags --libs opencv4`
//
// Uruchomienie:
//   ./hand_motion
//
// Wymagania:
//   - OpenCV z modułem dnn (np. 4.x)
//   - Model OpenPose COCO (18 punktów szkieletu):
//       1) Plik prototxt (architektura sieci), np.:
//          openpose_pose_coco.prototxt
//          (np. z repo opencv_extra: testdata/dnn/openpose_pose_coco.prototxt) :contentReference[oaicite:0]{index=0}
//
//       2) Plik caffemodel z wagami, np.:
//          pose_iter_440000.caffemodel
//          (model COCO z projektu OpenPose) :contentReference[oaicite:1]{index=1}
//
//   Skopiuj oba pliki do katalogu z programem lub podaj pełne ścieżki.
// ============================================================================

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <string>

// ------------------ KONFIGURACJA MODELU I ALGORYTMU -------------------------

// Ścieżki do plików modelu – ZMIEŃ na swoje ścieżki, jeśli trzeba.
const std::string POSE_PROTO  = "models/openpose_pose_coco.prototxt";
const std::string POSE_MODEL  = "models/pose_iter_440000.caffemodel";

// Rozmiar wejściowy dla sieci (typowy dla przykładów OpenPose + OpenCV)
const int IN_WIDTH  = 368;   // szerokość obrazu wejściowego do sieci
const int IN_HEIGHT = 368;   // wysokość obrazu wejściowego do sieci

// Minimalne prawdopodobieństwo (pewność) dla punktu szkieletu
const float KEYPOINT_THRESHOLD = 0.1f;

// Progi dla detekcji ruchu (w pikselach):
const float MIN_2D_MOVEMENT_PX = 15.0f;   // minimalna zmiana pozycji w x/y
const float MIN_LENGTH_CHANGE  = 20.0f;   // minimalna zmiana długości ramienia

// Współczynnik wygładzania (prosty filtr IIR dla położenia końca ręki)
// 0.0 = brak wygładzania, 1.0 = brak reakcji na nowe dane
const float SMOOTHING_ALPHA = 0.8f;

// Indeksy punktów dla modelu COCO (18 punktów)
// Zgodnie z dokumentacją modelu COCO używaną w OpenPose: :contentReference[oaicite:2]{index=2}
// 0: Nose
// 1: Neck
// 2: RShoulder
// 3: RElbow
// 4: RWrist
// 5: LShoulder
// 6: LElbow
// 7: LWrist
// (Reszta nas tu nie interesuje)

// ------------------ POMOCNICZE STRUKTURY / FUNKCJE -------------------------

// Funkcja wyciąga wszystkie 18 punktów szkieletu z wyjścia sieci
// Wejście:
//   netOutputBlob - wynik net.forward(), wymiar: [1, nPoints, H, W]
//   frameWidth, frameHeight - rozmiar oryginalnej klatki z kamery
// Wyjście:
//   vector<cv::Point> size 18; jeśli punkt nie jest wykryty -> (-1, -1)
std::vector<cv::Point> getPoseKeypoints(const cv::Mat& netOutputBlob,
                                        int frameWidth,
                                        int frameHeight)
{
    int H = netOutputBlob.size[2]; // wysokość mapy cieplnej
    int W = netOutputBlob.size[3]; // szerokość mapy cieplnej
    int nPoints = netOutputBlob.size[1];

    std::vector<cv::Point> keypoints(nPoints, cv::Point(-1, -1));

    // netOutputBlob jest 4D: [N=1, C=nPoints, H, W]
    // Będziemy przechodzić po każdym kanale (czyli po każdym punkcie szkieletu)
    for (int n = 0; n < nPoints; ++n)
    {
        // Pobieramy mapę cieplną dla punktu n (2D float)
        cv::Mat heatMap(H, W, CV_32F,
                        (void*)netOutputBlob.ptr(0, n));

        // Szukamy maksimum na mapie (najbardziej prawdopodobne położenie punktu)
        double maxVal;
        cv::Point maxLoc;
        cv::minMaxLoc(heatMap, nullptr, &maxVal, nullptr, &maxLoc);

        // Jeśli pewność (maxVal) jest zbyt mała, punkt uznajemy za niewykryty
        if (maxVal > KEYPOINT_THRESHOLD)
        {
            // Przeskalowanie współrzędnych z przestrzeni HxW do rozmiaru oryginalnej klatki
            int x = static_cast<int>(frameWidth  * maxLoc.x / static_cast<float>(W));
            int y = static_cast<int>(frameHeight * maxLoc.y / static_cast<float>(H));
            keypoints[n] = cv::Point(x, y);
        }
        else
        {
            keypoints[n] = cv::Point(-1, -1);
        }
    }

    return keypoints;
}

// Prosta pomocnicza funkcja sprawdzająca, czy punkt jest "ważny" (wykryty)
bool isValidPoint(const cv::Point& p)
{
    return p.x >= 0 && p.y >= 0;
}

// Oblicza euklidesową odległość między dwoma punktami (float)
float distance(const cv::Point2f& a, const cv::Point2f& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Wygładzanie położenia (filtr IIR 1 rzędu)
// newFiltered = alpha * prevFiltered + (1 - alpha) * current
cv::Point2f smoothPoint(const cv::Point2f& prevFiltered,
                        const cv::Point2f& current,
                        float alpha)
{
    return cv::Point2f(
        alpha * prevFiltered.x + (1.0f - alpha) * current.x,
        alpha * prevFiltered.y + (1.0f - alpha) * current.y
    );
}

// Na podstawie wektora ruchu i zmiany długości ramienia
// zwraca nazwę kierunku ruchu ręki.
// Uwaga: w OpenCV układ współrzędnych jest taki, że:
//   - x rośnie w prawo
//   - y rośnie w dół
// Więc:
//   dx > 0  -> ruch w prawo (RIGHT)
//   dx < 0  -> ruch w lewo (LEFT)
//   dy > 0  -> ruch w dół (DOWN)
//   dy < 0  -> ruch w górę (UP)
//
// Forward/Backward określamy heurystycznie na podstawie
// zmiany długości ramienia w pikselach:
//
//   dLen > 0  -> ramię "wydłuża się" w obrazie -> interpretujemy jako FORWARD
//   dLen < 0  -> ramię "skróca się" w obrazie -> interpretujemy jako BACKWARD
//
// Jednocześnie wymagamy, by ta zmiana długości była odpowiednio duża
// i większa niż zmiany w x/y, by uniknąć błędnej klasyfikacji.
std::string classifyDirection(float dx, float dy, float dLen)
{
    float movement2D = std::sqrt(dx * dx + dy * dy);

    // Jeśli ruch jest bardzo mały we wszystkich wymiarach,
    // to po prostu nie klasyfikujemy go.
    if (movement2D < MIN_2D_MOVEMENT_PX && std::fabs(dLen) < MIN_LENGTH_CHANGE)
    {
        return "NONE";
    }

    // Sprawdź czy zmiana długości ramienia dominuje nad ruchem w płaszczyźnie
    if (std::fabs(dLen) > std::fabs(dx) &&
        std::fabs(dLen) > std::fabs(dy) &&
        std::fabs(dLen) > MIN_LENGTH_CHANGE)
    {
        if (dLen > 0)
            return "FORWARD";
        else
            return "BACKWARD";
    }

    // W przeciwnym razie klasyfikujemy ruch tylko na podstawie x/y:
    if (std::fabs(dx) > std::fabs(dy))
    {
        // Ruch w poziomie
        if (dx > 0)
            return "RIGHT";
        else
            return "LEFT";
    }
    else
    {
        // Ruch w pionie
        if (dy > 0)
            return "DOWN";
        else
            return "UP";
    }
}

// ------------------------------ MAIN ---------------------------------------

int main()
{
    // 1. Otwieramy kamerę (domyślnie urządzenie 0).
    //    Dla prostoty nie obsługujemy tutaj argumentów linii komend.
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cerr << "Blad: nie udalo sie otworzyc kamery (VideoCapture(0)).\n";
        return 1;
    }

    // 2. Ładujemy sieć DNN do estymacji pozy (OpenPose COCO)
    std::cout << "Ladowanie modelu OpenPose...\n";
    cv::dnn::Net net = cv::dnn::readNetFromCaffe(POSE_PROTO, POSE_MODEL);
    if (net.empty())
    {
        std::cerr << "Blad: nie udalo sie wczytac sieci. "
                     "Sprawdz sciezki POSE_PROTO i POSE_MODEL.\n";
        return 1;
    }

    // Opcjonalnie wybieramy backend / target.
    // Tu: CPU + backend OpenCV (przyjazne dla początkujących).
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    std::cout << "Model zaladowany. Uruchamianie petli przetwarzania...\n";

    // Zmienne do śledzenia ruchu ręki:
    bool hasPrev = false;                 // czy mamy poprzednią pozycję
    cv::Point2f prevFilteredTip;          // poprzednio wygładzona pozycja końca ręki
    float prevArmLength = 0.0f;           // poprzednia długość ramienia (od barku do końca)
    std::string lastDirection = "NONE";   // ostatni wykryty kierunek (dla wyświetlania)

    // Główna pętla: czytamy klatki z kamery, wykrywamy szkielet, liczymy ruch.
    while (true)
    {
        cv::Mat frame;
        cap >> frame; // pobierz następną klatkę
        if (frame.empty())
        {
            std::cerr << "Blad: pusta klatka z kamery.\n";
            break;
        }

        int frameWidth  = frame.cols;
        int frameHeight = frame.rows;

        // 3. Przygotowujemy wejście dla sieci DNN.
        //    Tworzymy tzw. "blob" – znormalizowany obraz wejściowy.
        //    scale = 1/255 (normalizacja do [0,1])
        //    rozmiar: IN_WIDTH x IN_HEIGHT
        cv::Mat inputBlob = cv::dnn::blobFromImage(
            frame,
            1.0 / 255.0,                   // skala
            cv::Size(IN_WIDTH, IN_HEIGHT), // rozmiar
            cv::Scalar(0, 0, 0),           // średnia (brak odejmowania średniej)
            false,                         // swapRB (BGR->RGB) - tutaj false, bo model jest w BGR
            false                          // crop - bez przycinania
        );

        // Ustawiamy blob jako wejście sieci i wykonujemy propagację w przód.
        net.setInput(inputBlob);
        cv::Mat netOutput = net.forward();

        // 4. Wyciągamy punkty szkieletu w układzie współrzędnych oryginalnego obrazu.
        std::vector<cv::Point> keypoints = getPoseKeypoints(netOutput, frameWidth, frameHeight);

        // Dla prostoty będziemy śledzić JEDNĄ rękę:
        //   najpierw próbujemy prawą (RShoulder, RElbow, RWrist),
        //   jeśli się nie uda, to lewą (LShoulder, LElbow, LWrist).

        cv::Point shoulder(-1, -1);
        cv::Point elbow(-1, -1);
        cv::Point wrist(-1, -1);

        // Indeksy COCO dla prawej ręki
        cv::Point rShoulder = keypoints[2];
        cv::Point rElbow    = keypoints[3];
        cv::Point rWrist    = keypoints[4];

        // Indeksy COCO dla lewej ręki
        cv::Point lShoulder = keypoints[5];
        cv::Point lElbow    = keypoints[6];
        cv::Point lWrist    = keypoints[7];

        // Prosta strategia:
        //   - jeśli prawa ręka ma wykryty bark i Kolor (łokieć lub nadgarstek), śledzimy prawą
        //   - w przeciwnym razie próbujemy lewą
        bool rightArmOk = isValidPoint(rShoulder) && (isValidPoint(rElbow) || isValidPoint(rWrist));
        bool leftArmOk  = isValidPoint(lShoulder) && (isValidPoint(lElbow)  || isValidPoint(lWrist));

        if (rightArmOk)
        {
            shoulder = rShoulder;
            elbow    = rElbow;
            wrist    = rWrist;
        }
        else if (leftArmOk)
        {
            shoulder = lShoulder;
            elbow    = lElbow;
            wrist    = lWrist;
        }

        // Wyznaczamy punkt reprezentujący "koniec ręki"
        cv::Point2f armTip(-1.0f, -1.0f);

        if (isValidPoint(wrist))
        {
            // Idealnie chcemy śledzić nadgarstek (punkt najbardziej "na końcu" ręki)
            armTip = wrist;
        }
        else if (isValidPoint(elbow))
        {
            // Jeśli nadgarstek nie został wykryty, bierzemy łokieć.
            armTip = elbow;
        }

        // Jeśli mamy bark i jakiś punkt końca ręki, możemy analizować ruch
        if (isValidPoint(shoulder) && armTip.x >= 0 && armTip.y >= 0)
        {
            cv::Point2f shoulderF(shoulder.x, shoulder.y);

            // Aktualna długość ramienia w pikselach
            float armLength = distance(shoulderF, armTip);

            // Wygładzanie pozycji końca ręki
            cv::Point2f currentFilteredTip;
            if (!hasPrev)
            {
                // Pierwsza klatka – nie mamy historii, więc przyjmujemy bieżący punkt
                currentFilteredTip = armTip;
                prevFilteredTip    = armTip;
                prevArmLength      = armLength;
                hasPrev            = true;
                lastDirection      = "NONE";
            }
            else
            {
                // W kolejnych klatkach mieszamy poprzednią filtrowaną pozycję z nową
                currentFilteredTip = smoothPoint(prevFilteredTip, armTip, SMOOTHING_ALPHA);

                // Oblicz wektor ruchu (z filtrowanych pozycji)
                float dx = currentFilteredTip.x - prevFilteredTip.x;
                float dy = currentFilteredTip.y - prevFilteredTip.y;

                // Zmiana długości ramienia (też nieco wygładzona przez to,
                // że pozycja końca ręki jest filtrowana)
                float dLen = armLength - prevArmLength;

                // Klasyfikacja kierunku ruchu
                std::string direction = classifyDirection(dx, dy, dLen);

                // Zachowujemy ostatni kierunek – przydatne do wizualizacji
                if (direction != "NONE")
                {
                    lastDirection = direction;
                }

                // Logi debugujące – pokazujemy wektor ruchu i decyzję.
                std::cout << "dx=" << dx << " dy=" << dy
                          << " dLen=" << dLen
                          << "  -> direction=" << direction << "\n";

                // Aktualizacja historii (poprzednie filtrowane wartości)
                prevFilteredTip = currentFilteredTip;
                prevArmLength   = armLength;
            }

            // 5. Wizualizacja szkieletu ręki na obrazie:
            //    - kółka w miejscach punktów,
            //    - linie łączące bark-łokieć-nadgarstek.

            // Bark
            cv::circle(frame, shoulder, 5, cv::Scalar(0, 255, 255), cv::FILLED); // żółty
            // Łokieć (jeśli jest)
            if (isValidPoint(elbow))
            {
                cv::circle(frame, elbow, 5, cv::Scalar(0, 255, 0), cv::FILLED);   // zielony
                cv::line(frame, shoulder, elbow, cv::Scalar(255, 0, 0), 2);      // niebieska linia bark->łokieć
            }
            // Nadgarstek (jeśli jest)
            if (isValidPoint(wrist))
            {
                cv::circle(frame, wrist, 5, cv::Scalar(0, 0, 255), cv::FILLED);   // czerwony
                if (isValidPoint(elbow))
                {
                    cv::line(frame, elbow, wrist, cv::Scalar(255, 0, 0), 2);      // niebieska linia łokieć->nadgarstek
                }
                else
                {
                    cv::line(frame, shoulder, wrist, cv::Scalar(255, 0, 0), 2);   // bark->nadgarstek
                }
            }
        }
        else
        {
            // Brak wiarygodnej ręki w tej klatce – resetujemy stan
            hasPrev       = false;
            lastDirection = "NONE";
            std::cout << "Rama: nie wykryto wiarygodnej reki.\n";
        }

        // 6. Wizualizacja kierunku na ekranie (tekst)
        std::string text = "Direction: " + lastDirection;
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.8;
        int thickness = 2;
        cv::putText(frame, text, cv::Point(30, 40), fontFace, fontScale,
                    cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);

        // 7. Wyświetlenie okna z obrazem
        cv::imshow("Hand Motion Demo", frame);

        // Wyjście z programu po wciśnięciu klawisza 'q'
        char c = static_cast<char>(cv::waitKey(1));
        if (c == 'q' || c == 27) // 'q' lub ESC
        {
            break;
        }
    }

    return 0;
}
