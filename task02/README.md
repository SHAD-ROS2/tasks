# ROS 2 · занятие 02 — Black-box graph rescue

В этой работе нужно восстановить незнакомый six-node warehouse stack, доказать
его работу в чистом окружении и запустить два независимых экземпляра. Стартовый
проект намеренно неисправен: успешный запуск до ваших изменений не ожидается.

Полное условие и внешний контракт: [`docs/Practice_L02.md`](docs/Practice_L02.md).

## Старт

Из терминала хоста в папке `task02/`:

```bash
cp .env.example .env
./course up
./course doctor
./course shell
```

Файлы остаются обычной локальной папкой и примонтированы в контейнер как
`/workspace`. Чтобы показывать и редактировать их в VS Code, откройте второй
терминал хоста в `task02/` и выполните:

```bash
code .
```

VS Code не обязан быть установлен внутри контейнера: сохранённые на хосте
изменения сразу видны в `/workspace`.

На Linux/WSL, если `id -u` или `id -g` выводит не `1000`, перед первым запуском
замените `LOCAL_UID` и `LOCAL_GID` в `.env` на эти значения.

Внутри контейнера рабочая папка уже открыта на корне Task 02, а ROS 2 Jazzy
подключён. Канонический pipeline:

```bash
rosdep update --rosdistro jazzy  # один раз в новом контейнере
rosdep install --from-paths src --ignore-src -y
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Целевые команды запуска:

```bash
ros2 launch warehouse_bringup warehouse.launch.py namespace:=robot_1
ros2 launch warehouse_bringup two_robots.launch.py
```

Сначала зафиксируйте наблюдаемый симптом и только потом меняйте код или
конфигурацию. Не переносите в репозиторий готовые решения и не ослабляйте
контракт ради прохождения одной локальной проверки.

## Результат

- исправленный workspace остаётся внутри `task02/`;
- в корне `task02/` создан `diagnosis.yaml` по опубликованной схеме;
- single-robot и two-robot acceptance воспроизводятся после чистой сборки;
- изменения закоммичены и отправлены в ветку `master` личного репозитория.

Завершив работу, выйдите из shell и остановите окружение:

```bash
exit
./course down
```
