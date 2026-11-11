# Task 07. Команды для Gazebo

Из каталога `task07/` соберите образ Task 05, если он ещё не собран. Затем запустите рабочий стол и редактор.

```bash
docker compose -f ../task05/compose.yaml build workspace
docker compose --profile demo up -d --pull never --wait workspace editor
```

Откройте [Gazebo](http://localhost:6087/vnc.html?autoconnect=true&resize=scale)
и [браузерный редактор](http://localhost:8087/?workspace=/workspace/l07/l07-demo.code-workspace).

В терминале A соберите пакет и объявите функцию запуска сцены.

```bash
docker compose exec workspace bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select stability_lab
source install/setup.bash

scene() {
  local base="$1" fault="$2"
  shift 2
  ros2 launch stability_lab sim.launch.py "base:=$base" "fault:=$fault" gui:=true "$@"
}
```

В терминале B откройте второй shell контейнера. Команды пишут результаты
в каталог `RUN` под `evidence/`, доступный в браузерном редакторе.

```bash
docker compose exec workspace bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p evidence
RUN=$(mktemp -d evidence/show-XXXXXXXX)

observe() {
  local label="$1"
  shift
  ros2 run stability_lab observe --output "$RUN/$label" "$@"
}

motion() {
  local label="$1"
  shift
  ros2 run stability_lab motion --output "$RUN/$label" "$@"
}
```

Запускайте выбранные пары из [таблицы симптомов](../DIAGNOSTICS.md). Каждая строка использует свежую сцену. Обычно запустите A, затем B. После завершения B
остановите A через `Ctrl+C` и дождитесь приглашения shell. В строках 8–9
запустите B первым и сразу запустите A. В строках 10–13 запустите A и дождитесь
модели в Gazebo. Пока сцена на паузе, launch ждёт Play перед активацией
контроллеров. Запустите B непосредственно перед **Play**, потому что `observe`
ждёт свежие данные до 45 секунд. Если повторяете строку, выберите новый `RUN`
либо новое имя результата.

| № | Терминал A | Терминал B | Что показать |
|---|---|---|---|
| 1 | `scene free none` | `observe stand --duration 10` | Штатная стойка, контроль для строк 8–9. |
| 2 | `scene free none` | `motion squat --duration 12 --depth 0.18` | Штатный присед и возврат. |
| 3 | `scene free inertia` | `motion inertia --duration 12 --depth 0.18` | Та же траектория, наклон корпуса около 10° при схожей ошибке слежения. |
| 4 | `scene fixed none` | `motion axis-control --duration 8 --depth 0.18` | Штатное сгибание ног при закреплённом тазе. |
| 5 | `scene fixed joint_axis` | `motion axis-fault --duration 8 --depth 0.18` | Ось левого колена отклонена на 20°. Стопа уходит вбок примерно на 3.8 cm. |
| 6 | `scene fixed none` | `motion yaw-control --duration 8 --depth 0` | Штатное удержание корпуса, `desired` поворота около нуля. |
| 7 | `scene fixed none` | `motion yaw-chatter --profile reference_chatter --duration 8` | `desired` уже колеблется, `actual` следует за ним. |
| 8 | `scene free torso_com` | `observe torso-com --duration 10` | Смещённый центр масс, наклон вперёд и остановка по `tilt_limit`. |
| 9 | `scene free link_mass` | `observe link-mass --duration 10` | Ошибка массы предплечья, наклон вперёд и вбок. |
| 10 | `scene free spawn_drop start_paused:=true` | `observe spawn-drop --duration 4` | Посадка после падения с 20 см со штатными подошвами. |
| 11 | `scene free spawn_bounce start_paused:=true` | `observe spawn-bounce --duration 4` | Та же высота и пол, завышенное восстановление подошв. После первого касания G1 отскакивает примерно на 11 см. В проверочном прогоне последующий наклон вызвал `tilt_limit` и паузу. |
| 12 | `scene free none start_paused:=true lateral_push:=true` | `observe grip --duration 4` | Стопы держатся на полу под боковой нагрузкой. |
| 13 | `scene free slippery_feet start_paused:=true lateral_push:=true` | `observe slip --duration 4` | Та же нагрузка, низкое трение. Сравните `Foot XY drift`, наклон и контакт стоп. |
| 14 | `scene fixed none` | `motion servo-control --profile yaw_sweep --duration 8` | Плавное задание поворота, `actual` близок к `desired`. |
| 15 | `scene fixed slow_servo` | `motion servo-lag --profile yaw_sweep --duration 8` | То же плавное задание, `actual` заметно отстаёт при повороте и возврате. |
| 16 | `scene fixed none` | `motion elbow-control --profile elbow_sweep --duration 8` | Запланированное сгибание левого локтя. |
| 17 | `scene fixed none` | `motion elbow-swap --profile swapped_joints --duration 8` | Та же цель для левого локтя попала в команду правого локтя. Сравните имена и позиции в сообщении. |

В редакторе через `Cmd+P` или `Ctrl+P` откройте PNG и JSON прогона. Для
`reference_chatter` можно заранее открыть готовый
[график проверочного прогона](assets/reference-chatter.png).
Для `spawn_bounce` покажите [сравнение высоты таза после первого касания](assets/spawn-bounce.png).
Для `slippery_feet` сопоставьте смещение стоп с флагами контакта в CSV.
Для `slow_servo` сравните кривые
`desired` и `actual` поворота корпуса, а для `swapped_joints` — обе кривые локтей
и `probe_intended_joint` / `probe_commanded_joint` в JSON.
