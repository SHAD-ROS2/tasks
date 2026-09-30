# Домашнее задание 03. Repair the rover model

ROS 2 Jazzy · Python · ориентир 2–3 часа после практики

Вам передали ровер, который выглядит почти правильно в RViz, но данные LiDAR,
камеры, IMU и odometry не согласованы. Нужно восстановить систему по заданному
контракту и реализовать проверку преобразований на время измерения.

Сначала пройдите [практику](Practice_L03.md). Учебные примеры можно использовать
как справочник. Причины неисправностей домашнего ровера определите самостоятельно.

## 1. Что сдать

1. Исправленные description/configuration и Python validator в `task03/`.
2. Рабочую RViz configuration в `src/rover_description/rviz/homework.rviz`.
3. Один собственный regression test, выявляющий подмену measurement stamp на latest.
4. `diagnosis.yaml` в корне `task03/` по [шаблону](diagnosis.template.yaml).

Не изменяйте готовый fixture, геометрический oracle и transport/queue validator,
чтобы подогнать наблюдения под неправильную модель. Приёмка использует независимый
observer. Не заменяйте ROS-взаимодействие прямым чтением эталонных координат.

## 2. Запуск

```bash
./course up
./course build
./course run homework rviz:=true
```

Во втором терминале:

```bash
./course check
```

Исходный starter собирается и запускается, но `check` и policy tests не проходят.
`NOT_IMPLEMENTED` обозначает незаполненную Python policy. Работа в RViz помогает
диагностике; headless-проверка является обязательной частью результата.

В scene работает один `/clock`; все потребители используют `use_sim_time=true`.
Перед запуском домашней сцены остановите practice launch.

## 3. Физическая спецификация

Размер корпуса `0.60 × 0.40 × 0.16` м. `base_link` находится в центре корпуса.
Радиус колеса `0.10` м, ширина `0.06` м. Центры колёс относительно базы:
`(±0.21, ±0.25, −0.10)` м. Высота base_link в odom равна `0.20` м.

Body axes: x вперёд, y влево, z вверх. Camera optical axes: z вперёд,
x вправо, y вниз. Все длины в метрах, углы в радианах.

| Parent / child | xyz, м | roll, pitch, yaw, рад |
|---|---|---|
| `map / odom` | `0, 0, 0` | `0, 0, 0` |
| `base_link / lidar_link` | `0.10, 0, 0.16` | `0, 0, 0` |
| `base_link / camera_link` | `0.31, 0, 0.06` | `0, 0, 0` |
| `camera_link / camera_optical_frame` | `0, 0, 0` | `−π/2, 0, −π/2` |
| `base_link / imu_link` | `−0.12, 0.04, 0.04` | `0, 0, π/2` |

Ожидаемое дерево содержит `map → odom → base_link`, от базы отходят сенсоры и
колёса, от camera_link — camera_optical_frame. Fixture публикует map/odom и
odom/base_link. `robot_state_publisher` публикует связи модели. Дополнительный
publisher для уже существующего ребра не требуется.

Постоянный `map → odom` — упрощение этого стенда. В системе с локализацией
это преобразование может меняться.

## 4. Topics и frames

Namespace по умолчанию `/rover`, frame prefix `rover/`. В таблице topic names и
frame IDs указаны без этих префиксов:

| Topic | Type | Frame |
|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | `lidar_link` |
| `camera/image_raw` | `sensor_msgs/Image` | `camera_optical_frame` |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | `camera_optical_frame` |
| `imu` | `sensor_msgs/Imu` | `imu_link` |
| `odom` | `nav_msgs/Odometry` | header: `odom`, child: `base_link` |
| `joint_states` | `sensor_msgs/JointState` | имена joints модели |
| `observation/lidar` | `geometry_msgs/PointStamped` | `lidar_link` |
| `observation/camera` | `geometry_msgs/PointStamped` | `camera_optical_frame` |
| `observation/imu` | `geometry_msgs/PointStamped` | `imu_link` |
| `validation/report` | `std_msgs/String`, JSON | поля source/target/stamp |

Три observation topics — готовые **калибровочные точки стенда**, выраженные
в соответствующем sensor frame. Они сохраняют время исходного измерения.
IMU observation не является оценкой позиции от инерциального датчика.
Такой адаптер позволяет написать одну общую функцию проверки вместо трёх
обработчиков различных sensor messages.

После преобразования в `rover/odom` точки должны совпасть с:

| Sensor | Контрольная точка в odom, м |
|---|---|
| LiDAR | `(3.0, 0.0, 0.36)` |
| Camera | `(4.0, 0.4, 0.4)` |
| IMU calibration | `(1.0, 2.0, 0.6)` |

Допустимая евклидова ошибка `0.005` м. Observer отдельно проверяет доступность TF
и совпадение с известной геометрией: успешный lookup сам по себе недостаточен.
Pose Odometry должен совпадать с TF `odom → base_link` на тот же stamp.

## 5. Время и Python validator

Симуляционное время начинается с `10.0` с. Fixture публикует TF с частотой
20 Hz, измерения приходят с задержкой `0.20` симуляционных секунд. Робот движется,
поэтому запрос latest может успешно вернуть геометрически неправильный результат.

Изменяемый файл:
`src/rover_tf_validator/rover_tf_validator/validator.py`.
Нужно реализовать две функции:

- `quaternion_is_valid(rotation, tolerance=1e-3)`;
- `validate_observation(observation, lookup)`.

Готовые contracts, геометрическое применение transform, TF adapter, очередь,
ограничение ожидания и JSON serialization находятся рядом. Контракт функции:

1. Отклонить пустые или начинающиеся с `/` frame IDs, нечисловые/нефинитные
   координаты и неположительный `stamp_ns`. Нулевой measurement stamp в этом
   задании запрещён: в TF2 zero имеет специальный смысл latest.
2. Выполнить один неблокирующий `lookup(target_frame, source_frame, stamp_ns)`
   именно на время измерения. Target домашнего задания — prefixed `odom`.
3. Сохранить status/detail ошибки lookup. Готовая очередь повторяет временно
   неуспешные запросы в пределах `0.50` steady seconds.
4. Проверить конечность чисел transform и quaternion: четыре компоненты
   `x, y, z, w`, `abs(norm(q)−1) <= 0.001`. Не нормализовать некорректный вход
   молча. Hook также проверяет raw TF до помещения в buffer.
5. Применить transform и вернуть отчёт с исходными source/target/stamp.

Статусы: `OK`, `INVALID_INPUT`, `UNKNOWN_FRAME`, `DISCONNECTED`,
`TIME_UNAVAILABLE`. `NOT_IMPLEMENTED` зарезервирован для starter.
JSON format создаёт предоставленная функция `report`; её схему менять не нужно.

Не заменяйте неуспешный запрос latest, текущим временем или identity transform.
Не блокируйте callback ожиданием: `/tf` и `/clock` обслуживаются тем же executor.
Старый stamp у static TF сам по себе не делает преобразование недействительным.

Полное восстановление пользовательского состояния после reset clock не входит
в обязательную домашнюю работу; после reset перезапустите validator/listeners.

## 6. Проверки

```bash
./course test
./course check
```

Быстрые policy tests без запуска ROS graph:

```bash
./course exec python3 -m pytest src/rover_tf_validator/test/test_policy.py -q
```

Чистый pipeline в новом checkout/контейнере:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update --rosdistro jazzy  # один раз в новом контейнере
rosdep install --from-paths src --ignore-src -y
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Проверьте другой namespace и frame prefix. Остановите предыдущий launch, затем:

```bash
./course run homework namespace:=testbot frame_prefix:=testbot/ rviz:=false
```

Во втором терминале используйте observer с теми же именами:

```bash
./course exec ros2 run rover_tf_validator acceptance --frame-prefix testbot/ --output /workspace/evidence/testbot.json --ros-args -r __ns:=/testbot -p use_sim_time:=true
```

Итоговые варианты могут менять имена, задержку от 0 до 1.0 симуляционной секунды, скорость
движения и времена запросов. Проверяются интерполяция внутри истории, задержанный
TF, missing/disconnected frames, past/future timestamps, zero stamp, invalid
quaternion и static TF. Все эти классы являются частью опубликованного контракта.

Observer проверяет ожидаемые publishers стенда и единственного parent для каждого
frame. Это проверка назначенных ролей в учебном graph, а не универсальное доказательство
авторства каждого TF message в произвольной ROS-системе.

Один ваш тест должен численно отличать правильный historical lookup от latest.
Одной проверки «исключение не возникло» недостаточно.

## 7. Evidence и баллы

Для каждой найденной причины запишите исходный симптом, probe, установленную
причину, минимальное изменение и повторную проверку. Включите результат движения
с задержанными данными: неподвижная сцена может скрыть ошибку времени.

| Компонент | Баллы |
|---|---:|
| Чистая сборка и package quality | 15 |
| Модель, frames, согласованность sensor observations и odometry | 40 |
| Validator и объявленные варианты времени/ошибок | 30 |
| Воспроизводимость, ограниченное ожидание и cleanup | 10 |
| Собственный regression и evidence | 5 |

Оценивается зафиксированный commit в личном репозитории. Публичные тесты помогают
проверке, а итоговый observer работает отдельно от изменяемого студентом кода.
Дедлайн сообщает преподаватель.

```bash
git add task03
git commit -m "L03: repair rover frames and measurement-time validation"
git push
```

Перед push проверьте `git diff --stat`, состав сдачи и отсутствие generated
build/install/log. Скриншот RViz можно добавить как evidence, но он не заменяет
численную проверку и собственный тест.
