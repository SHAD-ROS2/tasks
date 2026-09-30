# Практика 03. Frames, время и модель

90 минут · Python · работа с преподавателем · отдельной оцениваемой сдачи нет

Цель — предсказывать, как изменение frame, joint или timestamp повлияет на
наблюдение, и проверять предсказание численно. Все эксперименты выполняются
на исправной учебной тележке. [Домашнее задание](Homework_L03.md) использует
другую конфигурацию ровера.

На каждом шаге запишите: **предсказание, команду, наблюдение и объяснение**.
Достаточно коротких заметок. Не меняйте несколько параметров одновременно.

## 0–10. Запуск и чтение данных

На хосте из `task03/`:

```bash
./course up
./course doctor
./course build
./course run practice rviz:=true moving:=false
```

Откройте [RViz](http://localhost:6083/vnc.html?autoconnect=true&resize=scale).
Во втором терминале:

```bash
./course exec ros2 node list
./course exec ros2 topic echo /rover/probe --once
./course exec ros2 run tf2_ros tf2_echo rover/world rover/probe_link
```

Остановите непрерывный `tf2_echo` через Ctrl+C. Найдите в сообщении `probe`
frame и stamp. Контрольная точка неподвижна в `rover/world` и имеет координаты
`(2.0, 0.5, 0.4)` м. Значения в сообщении выражены в `rover/probe_link`.

Учебная цепочка: `world → cart_base → arm_link → probe_link`.
`world → cart_base` публикует fixture, связи модели — `robot_state_publisher`.
`arm_joint` управляет положением arm_link; имена joints не получают frame prefix.

**Обсудите:** почему координаты одной неподвижной точки могут меняться?
Что изменится при выборе `rover/cart_base` как Fixed Frame RViz?

## 10–25. Joint origin и visual origin

Сначала предскажите, повлияет ли каждый запуск на координаты `probe_link`.
Перед новым launch останавливайте предыдущий через Ctrl+C.

```bash
./course run practice rviz:=true moving:=false mount_x:=0.05 visual_x:=0.10
./course run practice rviz:=true moving:=false mount_x:=0.15 visual_x:=0.10
./course run practice rviz:=true moving:=false mount_x:=0.05 visual_x:=0.20
```

Эти три команды выполняются по очереди. Каждый раз сравните модель в RViz и:

```bash
./course exec ros2 run tf2_ros tf2_echo rover/cart_base rover/arm_link
```

`mount_x` задаёт положение joint относительно parent. `visual_x` задаёт положение
геометрии внутри arm_link. Запишите, какая часть результата изменилась в каждом
случае. Найдите соответствующие origins в
`src/rover_description/urdf/cart.urdf.xacro`.

## 25–40. Joint state, ось и Xacro

Верните исходную сцену:

```bash
./course run practice rviz:=true moving:=false
```

Во втором терминале меняйте угол, сохраняя остальную конфигурацию:

```bash
./course exec ros2 param set /rover/fixture joint_angle 0.0
./course exec ros2 param set /rover/fixture joint_angle 0.8
./course exec ros2 topic echo /rover/joint_states --once
```

Сопоставьте `name` и `position`, затем проверьте TF. Угол задаётся в радианах.
Неподвижное крепление probe относительно arm_link остаётся тем же.

Остановите launch, предскажите поведение при другой оси и проверьте:

```bash
./course run practice rviz:=true moving:=false arm_axis:="0 1 0" joint_angle:=0.4
```

Разверните Xacro в обычный URDF и найдите итоговое значение axis:

```bash
./course exec bash -c 'xacro /workspace/src/rover_description/urdf/cart.urdf.xacro frame_prefix:=rover/ arm_axis:="0 1 0" > /tmp/cart.urdf'
./course exec cat /tmp/cart.urdf
```

Разворачивание само по себе не запускает publisher и не создаёт ROS graph.

## 40–60. Время измерения и задержка

Остановите предыдущий launch. Запустите движущуюся сцену с заметной задержкой:

```bash
./course run practice rviz:=true moving:=true delay:=0.6
```

Файл упражнения:
`src/rover_tf_validator/rover_tf_validator/practice_exercise.py`.
В `choose_time(msg, mode)` допишите выбор query time: время из header для
`measurement` и специальный latest query для `latest`. Используйте
`rclpy.time.Time` и его `from_msg`. Остальной код преобразует одну точку.

```bash
./course exec ros2 run rover_tf_validator practice_exercise --ros-args -r __ns:=/rover -p use_sim_time:=true -p mode:=measurement
```

Затем остановите listener и повторите с `-p mode:=latest`.
Сравните `point_world` с `(2.0, 0.5, 0.4)`. Объясните зависимость ошибки от
движения и задержки. После обсуждения можно свериться с готовым демонстрационным
`practice_lookup`, у которого те же ROS arguments.

Поставьте сцену на паузу и возобновите её:

```bash
./course exec ros2 service call /rover/pause std_srvs/srv/SetBool '{data: true}'
./course exec ros2 topic echo /clock --once
./course exec ros2 service call /rover/pause std_srvs/srv/SetBool '{data: false}'
```

На паузе ROS time и новые измерения остановлены, но процесс и steady time
продолжают работать. Обсудите, какими часами следует ограничивать ожидание TF.

## 60–75. Небольшое расследование

Преподаватель даёт один вариант неисправной учебной конфигурации.
Сначала зафиксируйте симптом. Выберите проверку, которая отличает хотя бы две
гипотезы: неверный frame, отсутствие пути или отсутствие данных на нужное время.
Исправьте причину и повторите ту же проверку.

Полезные probes:

```bash
./course exec ros2 topic echo /rover/probe --once
./course exec ros2 param get /rover/fixture use_sim_time
./course exec ros2 run tf2_tools view_frames
./course exec ros2 run tf2_ros tf2_echo rover/world rover/probe_link
```

`view_frames` создаёт файлы в текущей директории контейнера (`/workspace`).
Схема дерева показывает структуру; отдельный lookup проверяет доступность
преобразования на выбранное время.

## 75–90. Домашнее задание

Посмотрите исправный ровер и пример отчёта, обсудите
[публичный контракт](Homework_L03.md). Затем остановите учебную сцену:

```bash
./course run homework rviz:=true
```

Во втором терминале:

```bash
./course check
```

Исходный результат должен быть неуспешным. Сборка и запуск исправны, а ремонт
модели и реализация validator относятся к домашней работе.

Перед окончанием занятия убедитесь, что можете воспроизвести начальное состояние
самостоятельно. Для возврата к исходной учебной конфигурации достаточно остановить
launch и заново выполнить `./course run practice rviz:=true` без overrides.
