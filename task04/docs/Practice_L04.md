# L04 Совместная практика

90 минут после лекции. Преподаватель запускает команды, показывает результат и объясняет наблюдение; вы повторяете те же действия. Практическая работа помогает освоить инструменты и отдельно не оценивается как домашнее задание.

Используйте два терминала в каталоге `task04/`. До занятия выполните `./course up`, `./course doctor`, `./course build`. Окна Gazebo и RViz доступны через адрес `./course desktop`.

## 00–10 Первый запуск

Терминал A:

```bash
./course practice
```

Терминал B:

```bash
./course exec ros2 topic list -t
./course exec ros2 topic info /clock --verbose
./course observe --mode practice --duration 5 --output evidence/practice-baseline.json
```

Найдите ровер, стену и изображение камеры. Проверьте наличие scan, IMU, odom и clock. Укажите, кто создаёт показания и кто показывает их в RViz. Окно с моделью само по себе не подтверждает наличие измерений.

## 10–23 Две системы обмена сообщениями

```bash
./course exec gz topic -l
./course exec ros2 topic info /rover/scan --verbose
```

Откройте `practice/settings.yaml`. Найдите строку `scan` и сопоставьте имя Gazebo с ROS topic, оба типа и направление.

Контролируемый эксперимент:

1. Остановите сцену через Ctrl+C в терминале A.
2. Скопируйте `practice/settings.yaml` в `practice/settings.yaml.backup`.
3. Только для строки `scan` измените direction с `GZ_TO_ROS` на `ROS_TO_GZ`.
4. Выполните `./course build`, затем `./course practice`.
5. Проверьте, существует ли Gazebo поток `/lab/practice/laser` и поступают ли ROS measurements. Сравните это с вашим прогнозом.
6. Остановите сцену, восстановите файл из backup, удалите backup и повторите `./course build`.

Для одной Gazebo выборки:

```bash
./course exec gz topic -e -t /lab/practice/laser -n 1
```

Команда наблюдения отсутствующего потока может ждать. Завершайте такую ручную диагностику Ctrl+C. У автоматической проверки есть собственный ограниченный timeout.

## 23–35 Команды движения

Снова запустите исправную практику в терминале A. В терминале B:

```bash
./course scenario --mode practice --output evidence/practice-scenario.json
```

Сценарий выполняет движение и проверки управления симуляцией; следите за сообщениями о фазах. Найдите наблюдения прямого движения, поворота и остановки. Сравните odometry и положение модели в мире. Назовите владельца `odom → base_link` и объясните, какие transforms публикует RSP.

После завершения сценария с reset остановите launch. Следующий опыт начинайте новым запуском.

## 35–50 Частота сенсора и скорость симуляции

Сначала запустите:

```bash
./course practice target_rtf:=1.0
```

Во втором терминале:

```bash
./course observe --mode practice --duration 10 --output evidence/rtf-1.json
```

Остановите сцену. В терминале A запустите `./course practice target_rtf:=0.5`, затем в терминале B:

```bash
./course observe --mode practice --duration 10 --target-rtf 0.5 --output evidence/rtf-half.json
```

До запуска запишите прогноз для интервала measurement stamps и интервала доставки. После запуска сравните:

- simulated time и elapsed steady time;
- измеренный RTF;
- частоту scan по stamps и по поступлению сообщений;
- sample count и возможные пропуски.

LiDAR настроен на 10 Hz по времени симуляции. Машина может не достичь заданного RTF; используйте фактически измеренное значение при объяснении. Настройка RTF не должна менять заданный physics step.

## 50–64 Пауза шаги и reset

Остановите предыдущую сцену, запустите новую практику. В терминале B:

```bash
./course control pause
./course control step --steps 1000
./course control reset
```

Перед каждым действием предскажите изменение sim time. Просмотрите результат control и world stats. При шаге 1 ms тысяча шагов соответствует 1 simulated second. Убедитесь, что после stepping мир снова paused. Принятие запроса и завершение шагов — разные события.

После reset зафиксируйте новую эпоху времени, остановите launch и начните следующий опыт полным перезапуском. Поздние сообщения после pause могут относиться к измерениям до паузы.

## 64–74 Проверка без окон

Убедитесь, что интерактивная сцена остановлена:

```bash
./course check --mode practice --output evidence/practice-headless.json
```

Проверка сама запускает свежую симуляцию, наблюдает её и завершает процессы. В отчёте должны присутствовать LiDAR, IMU и настоящие изображения камеры. Закрытые окна не означают выключенные rendering sensors.

## 74–82 Шум и смещение

На неподвижной платформе выполните три отдельных запуска, каждый раз прекращая предыдущий:

```bash
./course practice headless profile:=ideal
./course practice headless profile:=white
./course practice headless profile:=biased
```

Для каждого запуска выполните во втором терминале соответствующую команду:

```bash
./course observe --mode practice --profile ideal --duration 20 --output evidence/noise-ideal.json
./course observe --mode practice --profile white --duration 20 --output evidence/noise-white.json
./course observe --mode practice --profile biased --duration 20 --output evidence/noise-biased.json
```

Каждую команду наблюдения выполняйте только при работающей сцене с тем же profile. Сравните среднее и разброс gyro z. Объясните, почему увеличение выборки не устраняет постоянное смещение. Если времени мало, преподаватель выполняет сравнение на общем экране.

## 82–90 Домашнее задание

Откройте [условие ДЗ](Homework_L04.md). Найдите три файла для самостоятельной работы, открытые интерфейсы и критерии сдачи. Настройки домашней сцены и анализатор на практике не решаем.

В конце сохраните практические отчёты для себя. Остановите launch и выполните `./course down`.
