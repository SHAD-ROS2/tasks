# Занятие 03. Время, TF2 и модель робота

ROS 2 Jazzy · Python · 90 минут практики + самостоятельная домашняя работа

Сначала выполните [практику](docs/Practice_L03.md) на исправной учебной тележке.
Затем переходите к [домашнему заданию Repair the rover](docs/Homework_L03.md).
Практика формативная. На оценку сдаётся домашнее задание.

## Первый запуск

На хосте из `task03/`:

```bash
cp .env.example .env
./course up
./course doctor
./course build
./course run practice rviz:=true
```

Первый `up` собирает локальный образ поверх закреплённого digest образа курса.
Он загружает RViz/Xacro и браузерный рабочий стол; повторные запуски используют
кэш. На Linux/WSL до запуска поставьте в `.env` значения `LOCAL_UID` и
`LOCAL_GID` из `id -u` и `id -g`, если они отличаются от 1000.

Откройте [рабочий стол RViz](http://localhost:6083/vnc.html?autoconnect=true&resize=scale).
RViz работает внутри того же Linux-контейнера, что и ROS nodes. Устанавливать
ROS или X server на хост не требуется. Для запуска без графики используйте
`rviz:=false`; тесты не зависят от окна браузера.

Каждую дополнительную команду запускайте из второго терминала хоста через
`./course exec ...` либо откройте `./course shell`. Остановите текущий launch
через Ctrl+C перед запуском другой сцены: в одном ROS domain должен быть один
источник `/clock`.

## Основные команды

```bash
./course run practice rviz:=true
./course run homework rviz:=true
./course test
./course check
./course shell
./course down
```

Исходная домашняя работа намеренно не проходит поведенческие тесты и `check`.
Сообщение `NOT_IMPLEMENTED` относится к Python TODO. Это ожидаемое состояние
starter; `doctor`, сборка и запуск учебной практики должны работать сразу.

Прямые ROS-команды доступны после `./course shell`:

```bash
ros2 launch rover_bringup practice.launch.py rviz:=true
ros2 launch rover_bringup homework.launch.py rviz:=false
```

## Где находятся материалы

| Путь | Назначение |
|---|---|
| `docs/Practice_L03.md` | Пошаговые эксперименты на занятии |
| `docs/Homework_L03.md` | Публичный контракт домашней работы |
| `docs/diagnosis.template.yaml` | Шаблон evidence для сдачи |
| `src/rover_description` | Xacro/URDF и геометрия |
| `src/rover_bringup` | Launch, конфигурация и RViz |
| `src/rover_fixture` | Готовый источник времени, движения и синтетических данных |
| `src/rover_tf_validator` | Учебный lookup, домашний Python TODO и observer |

Учебная тележка и домашний ровер имеют разные frames и конфигурации.
Готовый учебный lookup демонстрирует один приём. Домашний validator добавляет
проверку входа, обработку ошибок и общую политику проверки измерений.

## Если что-то не работает

- **Browser connection failed:** дождитесь `./course up`, выполните `./course doctor`.
  Если порт занят, поменяйте `RVIZ_PORT` в `.env` и перезапустите среду.
- **RViz пустой:** проверьте, что launch запущен с `rviz:=true`, затем Fixed Frame
  и ошибки Displays. Перезапуск browser tab не перезапускает ROS nodes.
- **Package not found:** выполните `./course build`, откройте новый `./course shell`.
- **Extrapolation при запуске:** listener ещё наполняет buffer. Повторяющаяся ошибка
  после прогрева требует проверки stamps, frames и `use_sim_time`.
- **Время ведёт себя странно:** остановите старые launches. Проверьте
  `./course exec ros2 topic info /clock --verbose` и число publishers.

Изменения сохраняются на хосте благодаря bind mount. Не добавляйте `build/`,
`install/`, `log/` и `.env` в сдачу.

### Завершение RViz в browser desktop

Перед завершением сохраните нужную RViz configuration через File → Save Config As.
`./course run` и `./course launch` при Ctrl+C сначала закрывают окно RViz, затем
останавливают ROS launch. Остановка ограничена по времени: до 3 s для GUI,
5 s для launch, затем при необходимости TERM/KILL его группы процессов.

На проверенном Docker arm64 с software rendering RViz иногда аварийно завершает
Image/Camera displays с `Owner died` или `pthread_mutex` именно при закрытии.
Helper может напечатать сообщение о принудительном завершении и вернуть ненулевой
код; после его завершения можно снова выполнить `./course run ...`. Это известное
ограничение desktop-окружения, не дополнительная поломка домашней модели.
Численная приёмка запускается с `rviz:=false` и от этого ограничения не зависит.
При прямом `ros2 launch` сначала закройте RViz через X окна, затем нажмите Ctrl+C;
это уменьшает гонки сигналов, но не гарантирует отсутствие указанной ошибки GUI.
