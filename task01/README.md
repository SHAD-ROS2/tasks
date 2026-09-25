# ROS 2 · занятие 01 — Hidden Gift 2.0

90 минут · ROS 2 Jazzy · Python · индивидуальная работа

> **Frozen release `2026.1-l01-rc2`.** Файлы `.env` и
> `release.lock.json` выбирают проверенный multi-architecture image по immutable
> digest. Используйте выданные преподавателем версии этих файлов.

Работайте в личном private-репозитории, в уже существующей папке `task01/`.
Команды `docker compose`, `./course` и `git` выполняйте в терминале хоста;
команды `course ...` и `ros2 ...` — во встроенном терминале IDE.

## Результат работы

Завершите основную логику action server
`shad_interfaces/action/ExploreZone` для одного корректного goal. Робот должен:

- обследовать заданную область;
- вернуть центр зелёного прямоугольника либо подтвердить его отсутствие;
- отправлять простой feedback во время поиска;
- явно остановиться перед завершением action.

Проверяется поведение решения. LLM разрешены; проверьте полученный результат теми
же командами, что и собственный код. Геометрические и навигационные помощники уже
даны, чтобы основное время занятия ушло на ROS graph, action и state machine.

| Уровень | Cases |
|---|---|
| Core, 100 баллов | `found`, `absent` |
| Bonus после core | `inside`, `cancel` |
| Следующие недели | `busy`, `feedback_health`, `sensor_loss`, `namespace`, `deadline`, QoS, callback groups и locks |

## 1. Запуск окружения

Нужны Docker, Docker Compose v2 и Git. Поддерживаются Linux, macOS Apple Silicon
и Windows через WSL2.

В терминале хоста, начиная из корня личного репозитория:

```bash
git switch master
cd task01
docker compose up -d --wait
./course doctor
```

Откройте IDE: <http://127.0.0.1:8080>. Поле робота:
<http://127.0.0.1:8081>.

`./course doctor` проверяет Docker/Compose, порты, workspace, свободное место,
ROS imports и headless smoke test с discovery, `SensorSample` и движением по
`cmd_vel`. Команда создаёт или обновляет `reports/environment.json`.

Валидный JSON-файл `reports/environment.json`, созданный командой
`./course doctor`, должен быть закоммичен и находиться в `master` вместе с
решением к моменту проверки. Отчёт содержит только переносимые диагностические
поля и предназначен для коммита.

Doctor ожидает минимум 3 GiB в видимом workspace. Для image, Docker cache и
следующих занятий предусмотрите около 30 GB свободного места на диске хоста.

В терминале IDE:

```bash
course build
course run
```

В другом терминале IDE отправьте пробный goal:

```bash
course goal --area 2 2 8 8
```

Стартовый код проверяет работу стенда и задаёт точки расширения для решения.

## 2. Посмотрите ROS graph

В терминале IDE:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /lab/sample --verbose
ros2 topic echo /lab/sample --once --qos-reliability best_effort
ros2 action list -t
ros2 interface show shad_interfaces/action/ExploreZone
ros2 action info /lab/explore_zone
```

До запуска student server в graph присутствует `/lab/gift_world`. После
`course build` и `course run` появляются `/lab/gift_server` и action
`/lab/explore_zone`. Наличие endpoint подтверждает discovery; behavioral tests
отдельно проверяют результат поиска и остановку.

## 3. Обязательный контракт

В core-тесте приходит один корректный goal, а начальная позиция робота находится
вне подарка.

| Объект | Требование |
|---|---|
| Мир | Квадрат 11 × 11 м; робот — точка с unicycle-кинематикой |
| Подарок | Один зелёный прямоугольник со сторонами по осям, каждая сторона ≥ 1 м; либо подарка в области нет |
| Search area | Корректный прямоугольник внутри мира; adapter выполняет входную валидацию |
| Данные | Решение получает наблюдения через ROS topics |
| Управление | `cmd_vel`, `geometry_msgs/Twist` |
| Ограничения | `abs(linear.x) ≤ 2`, `abs(angular.z) ≤ 3`; команды содержат конечные числа |
| Action | Goal к `explore_zone` с `min_x`, `min_y`, `max_x`, `max_y` |
| Feedback | Во время работы приходит простой feedback о ходе поиска |
| Найдено | `target_found=true`; ошибка центра по евклидовой метрике ≤ 0.05 м |
| Отсутствие | После полного покрытия вернуть `target_found=false` |
| Завершение | Оба core-исхода — `SUCCEEDED`; перед result сервер публикует zero `Twist` |

Browser view служит для ручной диагностики. Решение использует публичные ROS
topics/action и `cmd_vel`; внешний grader предоставляет тот же опубликованный
контракт.

### Terminal status

| Ситуация | Terminal status | `target_found` |
|---|---|---|
| Центр найден | `SUCCEEDED (4)` | `true` |
| Область полностью проверена, подарок отсутствует | `SUCCEEDED (4)` | `false` |

Для bonus-cancel ожидается `CANCELED (5)` и `target_found=false`. Adapter уже
обрабатывает некорректную область и предоставляет validation/cancellation
infrastructure.

## 4. Что реализовать

### `src/hidden_gift/hidden_gift/mission.py`

Реализуйте класс `Mission` как чистый конечный автомат: observations поступают
через аргументы, а управляющие команды и outcomes возвращаются как значения:

- `raster(area)` формирует маршрут покрытия;
- `drive_to(sample, target)` возвращает
  `(linear_velocity, angular_velocity, waypoint_reached)`;
- при зелёном sample внутри goal создайте `RectangleProbe(area, sample)` и
  передавайте ему последующие samples;
- `RectangleProbe` измеряет границы и возвращает `Decision`, затем
  `Outcome('found', x, y)` либо `Outcome('abort')`;
- после полного маршрута без обнаружения верните `Outcome('absent')`;
- во время движения возвращайте простой feedback через `Decision`.

Помощники рассчитаны на точечного робота, прямоугольник по осям и идеальный
датчик — это явные предположения учебного стенда.

### `src/hidden_gift/hidden_gift/server.py`

Сопоставьте `Outcome('found')` и `Outcome('absent')` с успешным terminal status
и соответствующими полями result. Готовый adapter предоставляет publishers,
subscriptions, action server, feedback scheduling и explicit stop.

Student-owned core ограничен `Mission.step()` и отображением `found/absent` в
`SUCCEEDED`.

## 5. Проверка

В терминале IDE:

```bash
course build
course test --case found
course test --case absent
# Оба core-case одной командой:
course test
```

Core-тесты также проверяют feedback и explicit stop.

После зелёного core можно запустить bonus:

```bash
course test --case inside
course test --case cancel
```

Для ручной диагностики:

```bash
course run
# В другом терминале:
course goal --area 2 2 8 8
```

Следите за robot view, feedback, terminal status и zero `Twist` перед result.

### Типичные симптомы

| Симптом | Что проверить первым |
|---|---|
| Action отсутствует | `course build`, процесс server, namespace, source overlay |
| Goal принят, робот стоит | Наличие samples и ненулевая команда `Decision` во время движения |
| Пустая область завершилась `ABORTED` | Переход в `absent` после полного покрытия |
| Подарок найден, result содержит `false` | Переход Mission в `found` и отображение result в server |
| После result сохраняется движение | Публикацию zero `Twist` перед завершением |

## 6. Оценивание

| Проверка | Баллы |
|---|---:|
| `found`: найти подарок, вернуть центр с точностью 0.05 м, отправить feedback и явно остановиться | 55 |
| `absent`: подтвердить отсутствие после покрытия, вернуть `SUCCEEDED`, отправить feedback и явно остановиться | 45 |

Итоговый grader использует преподавательский образ и другие seeds, сохраняя тот
же опубликованный core contract. Public tests дают предварительную обратную
связь. Инфраструктурная ошибка направляется на перепроверку.

В итоговой оценке курса 90% составляет среднее лучших 9 из 10 практик, ещё 10% —
итоговый ролик длительностью до 3 минут.

## 7. `docs/verification.md`

Заполните файл собственными короткими ответами, всего 5–10 строк:

- `found`: `PASS` или `FAIL: одна короткая причина`;
- `absent`: `PASS` или `FAIL: одна короткая причина`;
- одна конкретная граница решения; при зелёном core можно указать bonus
  `inside` или `cancel`, который остался за пределами вашей реализации;
- использование LLM: `да` или `нет`.

`docs/verification.md` — обязательная запись самопроверки. Баллы определяет
поведение решения в `found` и `absent`.

## 8. Сдача

Все файлы решения должны находиться внутри `task01/`.

В терминале хоста, из `task01/`:

```bash
git switch master
git status --short
git add src/hidden_gift reports/environment.json docs/verification.md
git diff --cached --stat
git commit -m "L01: implement and verify Hidden Gift action"
git push origin master
```

В итоговый коммит входят:

- `src/hidden_gift/`;
- созданный командой `./course doctor` файл `reports/environment.json`;
- заполненный `docs/verification.md`.

После push проверьте на GitHub, что итоговые изменения и
`reports/environment.json` находятся в `master`.

**В дедлайн будет взят последний коммит в `master` для проверки.**

## 9. Troubleshooting

### Linux UID/GID

Если `id -u` выводит значение, отличное от 1000, добавьте в `.env`:

```bash
LOCAL_UID=<вывод id -u>
LOCAL_GID=<вывод id -g>
```

Перезапустите окружение:

```bash
docker compose down
docker compose up -d --wait
./course doctor
```

### Windows

Выполняйте команды из Ubuntu/WSL2 и размещайте clone в Linux filesystem,
например в `~/`, для корректной производительности bind mounts.

### Порты

При занятом порте задайте свободные `IDE_PORT` и `SIM_PORT` в `.env`, затем
перезапустите Compose. Открывайте адреса с выбранными портами.

### Удалённый Linux

Используйте SSH tunnels, сохраняя сервисы на localhost:

```bash
ssh -L 8080:127.0.0.1:8080 -L 8081:127.0.0.1:8081 USER@HOST
```

### Диагностика

Для обращения к преподавателю подготовьте название и версию ОС и полный вывод:

```bash
./course doctor
docker compose ps
docker compose logs --tail=200 workspace
```

Если doctor запущен только из IDE, поле host Docker имеет значение
`not_checked`; запуск `./course doctor` на хосте добавляет host-проверку.

После работы остановите окружение:

```bash
docker compose down
```

## Официальные материалы

- [ROS 2 Actions design](https://design.ros2.org/articles/actions.html)
- [ROS 2 callback groups](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html)
- [Python action tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Docker multi-platform builds](https://docs.docker.com/build/building/multi-platform/)
