# L04 Открытый контракт

## Запуск

Одна сцена на ROS domain и Gazebo partition. По умолчанию namespace `rover`, frame prefix `rover/`, world `lab`, seed `42`, target RTF `1.0`, profile `ideal`. Для scene launch используются аргументы `namespace:=...`, `frame_prefix:=...`, `world_name:=...`, `seed:=...`, `target_rtf:=...`, `profile:=...`.

Target RTF должен быть больше 0 и не больше 2. Namespace и frame prefix начинаются с буквы и используют буквы, цифры, `_` и `/`; world name использует буквы, цифры и `_`, начинается с буквы. Prefix нормализуется с завершающим `/`.

Для observer/check те же параметры имеют вид `--namespace`, `--frame-prefix`, `--world-name`, `--seed`, `--target-rtf`, `--profile`. Применяйте совпадающие параметры при наблюдении работающей сцены.

`observe --duration` задаёт число **simulated seconds** после готовности и прогрева в 1 simulated second. Время ожидания ограничено steady timeout `--timeout` (по умолчанию 90 секунд). Для медленного компьютера разрешено увеличить timeout до 300 секунд. Параметр duration лежит в диапазоне 1–60 simulated seconds, steps — 1–10000, seed — 1–4294967295. `observe` и `scenario` используют уже запущенную сцену; `check` запускает свежую headless сцену и завершает её после проверки.

## Прикладные интерфейсы

ROS topics ниже относительны namespace. Домашние Gazebo names начинаются с `/lab/homework/` и не изменяются. В `homework/wiring.yaml` уже записаны их фиксированные suffixes.

| Канал | ROS topic | ROS type | Gazebo suffix | Gazebo type | Направление |
|---|---|---|---|---|---|
| command | `cmd_vel` | `geometry_msgs/msg/Twist` | `actuator/velocity` | `gz.msgs.Twist` | ROS → Gazebo |
| scan | `scan` | `sensor_msgs/msg/LaserScan` | `front/ranges` | `gz.msgs.LaserScan` | Gazebo → ROS |
| imu | `imu` | `sensor_msgs/msg/Imu` | `body/inertial` | `gz.msgs.IMU` | Gazebo → ROS |
| image | `camera/image_raw` | `sensor_msgs/msg/Image` | `front/rgb` | `gz.msgs.Image` | Gazebo → ROS |
| info | `camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `front/calibration` | `gz.msgs.CameraInfo` | Gazebo → ROS |

Clock, odometry, joint states, TF и ground truth observer подключены инфраструктурой. Source code `configuration.py` задаёт модель; самостоятельно исправлять его не требуется.

## Время и frames

- `/clock` имеет один источник Gazebo через bridge. Потребители ROS используют `use_sim_time=true`.
- `odom.header.frame_id` равен `PREFIXodom`, child frame — `PREFIXbase_link`.
- `scan` имеет frame `PREFIXlidar_frame`, IMU — `PREFIXimu_frame`, Image/CameraInfo — `PREFIXcamera_optical_frame`.
- DiffDrive владеет `odom → base_link`; RSP — связями модели. Wheel joints называются `left_wheel_joint` и `right_wheel_joint`.
- Namespace, frame prefix и world name независимы; служебный launch согласует их. TF `/tf` и `/tf_static`, а также `/clock` глобальны внутри данного ROS domain.
- Native ground truth описывает модель в Gazebo world. Начальная высота корпуса около 0.15 m; odometry использует свою начальную плоскую систему. Для проверки движения сравниваются приращения.

Нулевой stamp допустим при старте. Проверка рабочего потока начинается после прогрева и ожидает advancing clock и согласованные stamps текущей эпохи. Reset начинает новую эпоху; данные до и после reset не смешиваются при подсчёте rate.

## Датчики и команды

| Величина | Настройка |
|---|---|
| Physics step | 0.001 s |
| LiDAR | 10 Hz, 180 rays, поле зрения около 180°, диапазон 0.08–12 m |
| IMU | 100 Hz, rad/s и m/s² |
| Camera | 10 Hz, RGB8, 160 × 120, согласованные CameraInfo |
| Odometry | 50 Hz |
| Прямая команда | `Twist.linear.x` в m/s |
| Поворот | `Twist.angular.z` в rad/s |

В учебных опытах держитесь в пределах `|v| ≤ 0.3 m/s`, `|ω| ≤ 0.5 rad/s`, явно посылайте stop. Готовый scenario runner выполняет ограниченный сценарий и посылает нулевую команду при завершении. Штатный DiffDrive не предполагается автоматически останавливающимся по command timeout.

На неподвижной горизонтальной платформе IMU ожидает около +9.81 m/s² по z, гироскоп ideal — около нуля. Profiles для gyro: `ideal` — нулевые Gaussian mean/stddev; `white` — mean 0, stddev 0.02 rad/s; `biased` — mean 0.08, stddev 0.02 rad/s. Это учебное смещение через Gaussian mean. Accelerometer noise в профилях white/biased имеет stddev 0.05 m/s².

LiDAR может возвращать `inf` для лучей без отражения. Для камеры проверяется реальное содержимое Image, размеры и calibration metadata. Ground truth, wheel odometry и sensor observations имеют разные роли.

## Публичная приёмка

Точные checks и наблюдения сохраняются в JSON. Базовый сценарий после прогрева проверяет:

- sensor rates по sim stamps: ±15% от заданной частоты;
- scan: 180 rays, более 30 конечных дальностей, центральная дальность около 2.82 m на practice и 2.22 m на homework с допуском 0.15 m до движения;
- camera: RGB8, 160 × 120, непустое изображение с разными pixel values и согласованные размеры/intrinsics CameraInfo;
- stationary gyro: mean/std должны отличаться от заданного профиля менее чем на 0.001 rad/s для ideal и 0.008 rad/s для white/biased;
- stationary accelerometer z: отклонение среднего от +9.81 меньше 0.25 m/s²;
- straight motion: world forward displacement и odometry displacement в интервале 0.28–0.55 m;
- stop: смещение world pose после остановки меньше 0.02 m;
- odometry/TF: согласованные child frame и положение xy с расхождением меньше 0.02 m;
- turn motion: изменение yaw в интервале 0.4–0.8 rad;
- pause: sim time и iterations не растут после подтверждения paused;
- step: приращение iterations равно N, приращение sim time равно `N × 0.001 s` с вычислительным допуском `1e-8 s`;
- reset: iterations и sim time возвращаются к началу; следующий измерительный прогон начинается полным перезапуском.

Порог по RTF отсутствует: сравниваются simulated-time поведение и наблюдаемые данные. Стандартный timeout ограничивает зависание; при слабом компьютере увеличьте его и сохраните значение в evidence. Проверка повторяемости сравнивает метрики с указанными допусками. Полного побитового совпадения изображений или случайной последовательности между компьютерами не требуется.

Для собственного анализатора действуют правила и крайние случаи из [условия домашней работы](Homework_L04.md). Численный результат считается правильным с обычной точностью float; тесты меняют выборки и имена в пределах этого контракта.
