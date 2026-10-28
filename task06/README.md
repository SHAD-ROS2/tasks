# L06: свой робот и воспроизводимая ошибка оси

Новый независимый учебный пример: тележка с поворотным щупом. RViz показывает
кинематику; значения суставов создаёт **синтетический** источник. Здесь нет
Gazebo, controller manager, приводов и измерений физического движения. Корень
`base_link` не закреплён joint к миру; положение базы в пространстве в этом опыте
не моделируется. `Fixed Frame: base_link` в RViz — выбор системы отображения.

- [Сценарий практики на 90 минут](docs/Practice_L06.md).
- `src/inspection_cart/urdf/cart.urdf` — исправный пример.
- `src/inspection_cart/urdf/cart-broken.urdf` — один дефект оси `turret_joint`.
- `tools/check_ros.py` — ограниченный по времени smoke test установленного launch,
  реальных `/joint_states` и TF; запускается в свободном ROS domain.
- `tools/check_bag.py` — дополнительная проверка record/replay обеих моделей
  с одним источником states и clock; создаёт новый bag, старый не перезаписывает.
  Запускайте на исходных моделях **до** исправления `cart-broken.urdf`: проверка
  ожидает, что broken-вариант ещё содержит дефект.

Домашнее задание проекта выдаётся отдельно и выполняется в личном приватном
репозитории.

## Окружение

ROS 2 Jazzy. Повторно используйте уже собранное окружение L05 (`task05/`),
останавливая его сцену управления перед этой практикой. Новый Dockerfile и
Compose project не нужны. Из корня публичного репозитория `SHAD-ROS2/tasks`
один раз скопируйте `task06/` в `task05/l06/`:

```bash
test ! -e task05/l06 && cp -R task06 task05/l06
```

Если `task05/l06/` уже существует, команда его не изменяет: продолжайте работу
в имеющейся копии. Все изменения модели, свои тесты и evidence в этом workflow
сохраняются в `task05/l06/`; исходный `task06/` остаётся учебным комплектом.
В host shell:

```bash
cd task05
./course up
docker compose exec workspace bash
```

В container shell:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=66
cd /workspace/l06
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
python3 -m pytest -q src/inspection_cart/test
python3 tools/check_ros.py
ros2 launch inspection_cart view.launch.py model:=clean
```

Каждый дополнительный shell требует тех же двух `source`, выбранного
`ROS_DOMAIN_ID` и `cd /workspace/l06`. **66 — пример:** в общей DDS-сети преподаватель
назначает отдельное свободное значение каждому студенту/паре. Во всех терминалах
одного опыта используйте одно значение; между независимыми опытами — разные.
Не запускайте две сцены одного опыта одновременно. Рабочий стол
открывается обычной `./course desktop` в host shell L05.

Пакеты `robot_state_publisher`, `rviz2`, `tf2_ros`, `rclpy`, `sensor_msgs`,
`rosbag2` и `python3-pytest` есть в проверенном образе L05. Синтетический источник
запускается по умолчанию и не требует дополнительных packages.

Для интерактивных sliders дополнительно нужен `ros-jazzy-joint-state-publisher-gui`;
он может отсутствовать в старом образе L05. Проверьте перед занятием:

```bash
ros2 pkg prefix joint_state_publisher_gui
```

Если пакет доступен, остановите предыдущий launch и используйте
`state_source:=gui`. Установку этой зависимости в учебное окружение преподаватель
готовит **до** занятия; обязательная практика использует `synthetic`.

```bash
ros2 launch inspection_cart view.launch.py state_source:=gui
```

Без окон добавьте `rviz:=false`. Источник `none` предназначен для ручной публикации
или replay; launch не создаёт свой `/joint_states`. Для replay нужны
`state_source:=none use_sim_time:=true`; эта комбинация проверяется launch-файлом.

## Проверка без ROS

Из этой папки, Python 3.8+:

```bash
python3 -m unittest discover -s src/inspection_cart/test -v
L06_MODEL="$PWD/src/inspection_cart/urdf/cart-broken.urdf" \
  python3 -m unittest discover -s src/inspection_cart/test -v
```

Вторая команда **должна упасть** на контракте горизонтального движения щупа.
Это ожидаемое воспроизведение дефекта. При наличии pytest можно использовать
`python3 -m pytest -q src/inspection_cart/test`; `requirements-test.txt` нужен
только при установке pytest на host. ROS не импортируется чистыми тестами.

## Наблюдаемый контракт

В `base_link`: X вперёд, Y влево, Z вверх; длины в m, углы в rad. При
`turret_joint = −π/2, 0, +π/2` начало `probe_tip` должно оказаться соответственно
в `(0.07, −0.30, 0.34)`, `(0.37, 0, 0.34)`, `(0.07, 0.30, 0.34)` m. Ось
поворота вертикальная, высота неизменна. Нулевой угол выглядит одинаково в обеих
моделях; поэтому одного красивого скриншота недостаточно.

Чистый numerical oracle поддерживает только tree joints fixed/revolute/continuous/
prismatic и вычисляет положение начала frame. Это узкая проверка данного
контракта; синтаксис URDF, инерции, контакты и controller behavior она не доказывает.

## Файлы инцидента

`rosbag2` сохраняет входные joint states. С моделью и тем же входом RSP
восстанавливает TF. Рядом с bag сохраните SHA-256 **фактически запущенного URDF**,
его копию, commit/dirty state исходников, команды и версию ROS. Поэтому bag с
входами можно воспроизвести и на broken модели, и после минимального исправления.
`/tf` и `/tf_static` в этом workflow не записываются: единственный RSP остаётся
владельцем этих transforms. Другой допустимый workflow — записать TF и затем
воспроизводить его без живого RSP; на этой практике он не используется.
