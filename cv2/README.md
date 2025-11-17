```
g++ -std=c++20 main.cpp -o hand_motion `pkg-config --cflags --libs opencv4`

```


lol

```
wget https://raw.githubusercontent.com/opencv/opencv_extra/master/testdata/dnn/openpose_pose_coco.prototxt -O openpose_pose_coco.prototxt

wget https://huggingface.co/camenduru/openpose/resolve/main/models/pose/coco/pose_iter_440000.caffemodel

```

i zapisac do models/

Krótkie omówienie algorytmu

1. Wykrywanie postaci / szkieletu

Używamy modelu OpenPose (wersja COCO, 18 punktów) załadowanego przez cv::dnn::readNetFromCaffe(...). 
LearnOpenCV
+1

Dla każdej klatki z kamery tworzymy blob (blobFromImage), podajemy go do sieci (net.forward()), a wynik to 4D tensor z mapami ciepła – po jednej mapie dla każdego punktu szkieletu. 
CV-Tricks.com

Funkcja getPoseKeypoints dla każdej mapy ciepła znajduje punkt o najwyższej wartości (maksymalna pewność) i skaluje jego współrzędne do rozmiaru oryginalnego obrazu.

2. Lokalizacja ramion / rąk

Z modelu COCO bierzemy indeksy:

Prawa ręka: RShoulder (2), RElbow (3), RWrist (4)

Lewa ręka: LShoulder (5), LElbow (6), LWrist (7) 
LearnOpenCV

Najpierw próbujemy zlokalizować prawą rękę (bark + łokieć/nadgarstek). Jeśli się nie uda – korzystamy z lewej.

Punkt reprezentujący „koniec ręki” to:

Nadgarstek, jeśli jest wykryty,

W przeciwnym razie – łokieć.

3. Obliczanie kierunku ruchu

Mamy bark (shoulder) oraz koniec ręki (armTip).

Liczymy długość ramienia w pikselach: armLength = |armTip - shoulder|.

Wygładzamy pozycję końca ręki prostym filtrem IIR:

filtered = alpha * filtered_prev + (1 - alpha) * current

Między kolejnymi klatkami z filtrowanych współrzędnych liczymy:

dx – zmiana w poziomie,

dy – zmiana w pionie,

dLen – różnica długości ramienia.

Funkcja classifyDirection decyduje:

Jeśli zmiana długości dominuje i jest duża → FORWARD lub BACKWARD (ramię „wydłuża się” lub „skraca” w obrazie).

W przeciwnym razie patrzy na dx i dy:

|dx| > |dy|: RIGHT (dx>0) lub LEFT (dx<0)

|dy| ≥ |dx|: DOWN (dy>0) lub UP (dy<0)

Dodatkowo jest próg minimalnego ruchu, żeby drobne drgania nie generowały sztucznych kierunków (ruch < MIN_2D_MOVEMENT_PX i zmiana długości < MIN_LENGTH_CHANGE → NONE).

4. Wizualizacja

Na klatce z kamery rysujemy:

Okrąg w miejscu barku, łokcia, nadgarstka.

Linię bark–łokieć–nadgarstek (segment ręki).

Aktualny (ostatni stabilny) kierunek ruchu pokazujemy tekstem Direction: ... w lewym górnym rogu klatki.

5. Ograniczenia tak prostej metody

Model 2D nie ma prawdziwej informacji o głębokości, więc FORWARD/BACKWARD jest tylko heurystyką opartą o zmianę długości projekcji ramienia w pikselach.

Działa najlepiej, gdy:

W kadrze jest jedna osoba.

Postać jest odpowiednio duża (ramiona dobrze widoczne).

Ręce nie są mocno zasłonięte (np. przez ciało).

W praktycznej aplikacji można:

Dodać lepszy filtr (historia kilku klatek + głosowanie),

Uwzględnić więcej punktów (barki, biodra) dla stabilniejszego wyboru osoby,

Połączyć to z dodatkowym modelem 3D lub danymi z IMU (jak w Twoim większym projekcie).