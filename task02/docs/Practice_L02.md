# Практика 02 · Black-box graph rescue

**Workspaces, launch, parameters, QoS, and discovery**
90 минут · ROS 2 Jazzy · Python · индивидуальная работа

## 1. Сценарий

Вам передали warehouse stack из шести ROS 2 nodes. У разработчика он работал в
накопленном окружении, но переданный source workspace не проходит чистый
pipeline и не поднимает обещанный graph. Нужно восстановить систему по её
наблюдаемому контракту, а затем запустить два экземпляра робота без cross-talk.

Это задача на диагностику интеграции. Не переписывайте прикладную логику и не
подменяйте ROS-взаимодействие прямыми вызовами между nodes. Исправление должно
работать после новой сборки, в другом namespace и при изменённом порядке старта.

Публичное условие намеренно не перечисляет внесённые неисправности и не указывает
файлы, которые требуется изменить. Оцениваются evidence, root cause, минимальное
исправление и воспроизводимая проверка.

## 2. Что сдать

1. Исправленный workspace в существующей папке `task02/`.
2. `diagnosis.yaml` в корне `task02/`, заполненный по
   [`docs/diagnosis.template.yaml`](diagnosis.template.yaml).
3. Коммит в ветке `master`, отправленный в личный private-репозиторий до дедлайна.

Не добавляйте `build/`, `install/`, `log/`, дампы окружения или скопированный
эталон. Итог проверяется из чистого checkout вашего коммита.

## 3. Окружение и чистый pipeline

На хосте, из `task02/`:

```bash
cp .env.example .env
./course up
./course doctor
./course shell
```

На Linux/WSL, если `id -u` или `id -g` выводит не `1000`, перед первым запуском
замените `LOCAL_UID` и `LOCAL_GID` в `.env` на эти значения. `course doctor`
должен подтвердить ROS distribution, RMW, domain и доступность workspace.

В контейнере:

```bash
rosdep update --rosdistro jazzy  # один раз в новом контейнере
rosdep install --from-paths src --ignore-src -y
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Сборка в уже подготовленном shell не доказывает воспроизводимость. Приёмка
начинается из нового контейнера и workspace без результатов прежнего `colcon
build`. Команды должны завершаться без ручного копирования файлов в `install/`.

## 4. Внешний контракт одного робота

Команда:

```bash
ros2 launch warehouse_bringup warehouse.launch.py namespace:=robot_1
```

должна поднять ровно следующие nodes:

| Node |
| --- |
| `/robot_1/range_sensor` |
| `/robot_1/obstacle_filter` |
| `/robot_1/mission_planner` |
| `/robot_1/drive_controller` |
| `/robot_1/safety_monitor` |
| `/robot_1/operator_panel` |

Публичные application topics внутри namespace:

| Topic | Type | Обязательное наблюдаемое свойство |
| --- | --- | --- |
| `range/readings` | `warehouse_interfaces/msg/RangeReading` | sensor stream, best effort, volatile, depth 5 |
| `perception/obstacle` | `warehouse_interfaces/msg/ObstacleState` | reliable, volatile |
| `control/motion` | `warehouse_interfaces/msg/MotionCommand` | reliable, volatile |
| `cmd_vel` | `geometry_msgs/msg/Twist` | reliable, volatile |
| `safety/state` | `warehouse_interfaces/msg/SafetyState` | reliable, transient local, depth 1 |

Все имена application topics должны оставаться относительными namespace робота.
Служебные ROS topics вроде `/rosout` и `/parameter_events` не входят в таблицу.

Параметры и значения базовой конфигурации:

| Node | Parameter | Value |
| --- | --- | ---: |
| `range_sensor` | `publish_rate_hz` | `4.0` |
| `range_sensor` | `readings_m` | `[0.35]` |
| `obstacle_filter` | `stop_distance_m` | `0.6` |
| `mission_planner` | `cruise_speed_mps` | `0.35` |
| `drive_controller` | `max_speed_mps` | `0.5` |
| `safety_monitor` | `stop_distance_m` | `0.5` |

`operator_panel` — late joiner: он стартует после producer-части stack и всё
равно должен получить последнее актуальное `safety/state`, даже если после его
подключения новое состояние не публиковалось.

## 5. Контракт двух роботов

Команда:

```bash
ros2 launch warehouse_bringup two_robots.launch.py
```

должна одновременно поднять экземпляры `/robot_1` и `/robot_2`: двенадцать
application nodes, по шесть на robot namespace. Одинаковые относительные topic
names внутри двух namespaces допустимы и ожидаемы.

Сообщение, произведённое одним экземпляром, не должно появляться как данные
второго. Поле `source_namespace` в сообщениях обязано соответствовать namespace
producer. Проверка не должна зависеть от фиксированной задержки «на всякий
случай»: готовность подтверждается состоянием graph и наблюдаемыми сообщениями.

Namespace решает изоляцию имён внутри общего ROS domain. Не меняйте
`ROS_DOMAIN_ID` между `/robot_1` и `/robot_2`: в этой части задания оба экземпляра
должны быть видны одному диагностическому процессу.

## 6. Порядок работы и evidence

Перед первым изменением сохраните исходный симптом: выполненную команду,
существенные строки вывода и наблюдаемое состояние. После каждого смыслового
исправления повторите минимальную проверку, которая отличает исправную систему
от исходной.

Для финальной приёмки зафиксируйте evidence как минимум для:

- чистого dependency/build/test pipeline;
- запуска package по имени из установленного workspace;
- шести ожидаемых nodes и типов application topics одного экземпляра;
- фактических значений параметров из YAML;
- QoS endpoints и поведения late joiner;
- двенадцати nodes и отсутствия cross-talk у двух экземпляров;
- корректного завершения launch без оставшихся процессов.

В `diagnosis.yaml` пишите наблюдения, а не предположения: точную команду,
релевантный результат, установленную причину, минимальное изменение и повторную
проверку. Большой diff сам по себе не является доказательством.

## 7. Приёмка

Работа считается завершённой, если из чистого checkout воспроизводятся все
условия ниже:

- `rosdep`, `colcon build`, `colcon test` и `colcon test-result` завершаются
  успешно;
- каждый package явно декларирует свои прямые build, runtime и test dependencies;
  наличие уже установленного пакета в образе не заменяет manifest contract;
- ROS может найти и запустить `warehouse_bringup` из `install/`;
- single-robot launch создаёт указанный graph с параметрами из конфигурации;
- все publisher/subscriber pairs совместимы по QoS;
- поздний `operator_panel` получает последнее safety state;
- two-robot launch создаёт два полных namespace без absolute-name leakage и
  cross-talk;
- shutdown останавливает все запущенные процессы;
- `diagnosis.yaml` связывает каждый исправленный симптом с evidence и
  verification.

Итоговая проверка может менять namespace, порядок старта и допустимые значения
параметров, сохраняя опубликованный контракт. Решение не должно зависеть от
конкретных `/robot_1`, одной последовательности сообщений или накопленного
overlay.

## 8. Баллы

| Компонент | Баллы |
| --- | ---: |
| Clean build, dependencies и package quality | 15 |
| Single-robot functional contract | 25 |
| Parameters, QoS и late joiner | 20 |
| Two-robot isolation и namespace portability | 25 |
| Cleanup и воспроизводимость | 10 |
| Evidence в `diagnosis.yaml` | 5 |

Публичные проверки дают формативную обратную связь. Итоговая проверка запускается
внешним observer на зафиксированном преподавателем commit SHA и не доверяет
изменяемым студентом workflow или локальным отчётам.

## 9. Сдача

На хосте, из корня личного репозитория:

```bash
git switch master
git add task02
git commit -m "L02: recover warehouse ROS graph"
git push origin master
```

Commit SHA никуда вписывать не нужно. После push убедитесь на GitHub, что файлы
находятся в `master` и изменения не вышли за пределы `task02/`.

## 10. Официальные материалы

- [Creating a workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Creating a package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Using colcon](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Managing dependencies with rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)
- [Launch for large projects](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
- [Passing ROS arguments](https://docs.ros.org/en/jazzy/How-To-Guides/Node-arguments.html)
- [QoS settings](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [Discovery](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Discovery.html)
- [ROS domain ID](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)
