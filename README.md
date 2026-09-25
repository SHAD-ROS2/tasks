# SHAD ROS 2 — задания курса

Каждое задание находится в отдельной папке `taskNN/`. Полное условие и команды
для конкретного задания находятся в `taskNN/README.md`.

| Задание | Тема |
| --- | --- |
| `task01/` | ROS 2 graph, interfaces and actions · Hidden Gift 2.0 |
| `task02/` | Workspaces, launch, parameters, QoS and discovery · Black-box graph rescue |

## Где работать

- Общий репозиторий [SHAD-ROS2/tasks](https://github.com/SHAD-ROS2/tasks) используется для практики на занятиях.
- Личный приватный репозиторий используется для домашних заданий и сдачи.
- Все файлы решения храните внутри папки соответствующего задания: `task01/`,
  `task02/` и так далее.

## Подготовка окружения

Среда курса запускается в Docker. Нужны Git, запущенный Docker и Compose v2
(команда `docker compose`). Рекомендуется 16 ГБ RAM и 30 ГБ свободного места.

- macOS: установите и запустите
  [Docker Desktop](https://docs.docker.com/desktop/setup/install/mac-install/).
- Windows: установите
  [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/)
  с WSL 2. Команды курса выполняйте в терминале Ubuntu/WSL, а репозиторий
  клонируйте в домашний каталог WSL.
- Ubuntu 24.04: установите
  [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) и
  [Compose plugin](https://docs.docker.com/compose/install/linux/). Настройте
  запуск Docker без `sudo` по
  [официальной post-install инструкции](https://docs.docker.com/engine/install/linux-postinstall/).

Проверьте установку:

```bash
git --version
docker version
docker compose version
docker run --rm hello-world
```

Для практики на занятии клонируйте общий репозиторий:

```bash
git clone https://github.com/SHAD-ROS2/tasks.git
cd tasks
```

Для домашней работы примите приглашение в личный приватный репозиторий и
клонируйте его по HTTPS-ссылке из GitHub. Затем откройте `taskNN/README.md` и
выполните описанные там шаги.

## Сдача

Сохраните решение в папке задания, сделайте commit и отправьте изменения в
личный приватный репозиторий:

```bash
git status
git add taskNN
git commit -m "Complete taskNN"
git push
```

Замените `taskNN` на номер задания, например `task01` или `task02`.

**В дедлайн будет взят последний коммит в `master` для проверки.**

## Если что-то не работает

Запустите команды из папки задания и пришлите преподавателю полный вывод вместе
с названием и версией операционной системы:

```bash
./course doctor
docker compose ps
docker compose logs --tail=200 workspace
docker version --format '{{.Server.Os}}/{{.Server.Arch}}'
```
