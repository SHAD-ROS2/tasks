# L06: тележка, ошибочная ось, bag и regression test

90 минут вместе с преподавателем. Основной результат — объяснить ошибку по
наблюдениям, сохранить воспроизводимый вход, получить падающий численный тест и
проверить минимальное исправление. Параллельно каждый выбирает идею своего
робота для первой домашки. Используем новый маленький пример; решения предыдущих
домашних работ не нужны.

## Подготовить до занятия

Пройдите [подготовку окружения](../README.md). Сборка и скачивание образа не входят
в 90 минут. Нужны три терминала в одном container и browser desktop L05.
В каждом container shell:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=66
cd /workspace/l06
source install/setup.bash
```

**66 — пример:** в общей DDS-сети используйте выделенный преподавателем свободный
domain для своей пары, одинаковый во всех её терминалах. Независимым парам нужны
разные значения, иначе их publishers будут обнаруживать друг друга.
Первый запуск: `colcon build --symlink-install`, затем `source install/setup.bash`.
Не держите работающей сцену L05/Gazebo или другую копию L06 в этом domain.
Никаких controllers здесь не запускается.

## 0–10 минут. Идея и минимальный контракт

Нарисуйте своего будущего робота: мобильная платформа, рука на тележке,
манипулятор на рельсе или другой небольшой механизм. Укажите, что у него
перемещается, и выберите хотя бы один ограниченный `revolute` joint. Набросайте
дерево links/joints и два положения, отличающих правильную сборку от ошибочной.

В учебной тележке колёса `continuous`, щуп `revolute` с конечными пределами
`[−π/2, +π/2]`. Корень — `base_link`, joint к `world` отсутствует. Пока не
демонстрируем движение базы, физическое равновесие или исполнительные механизмы.

## 10–25 минут. Увидеть симптом и собрать наблюдения

В терминале A:

```bash
ros2 launch inspection_cart view.launch.py model:=broken state_source:=synthetic
```

В RViz включены модель, сетка и frames. Щуп должен поворачиваться в горизонтальной
плоскости вокруг вертикальной оси; зелёная точка — `probe_tip`. Источник генерирует
углы по формуле, а не измеряет робота. Нулевой угол обеих моделей выглядит одинаково.

В терминале B:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
```

Через несколько секунд остановите `hz` через Ctrl+C. Ожидаемый source rate —
20 Hz; CLI показывает наблюдаемую частоту, которая зависит также от работы
subscriber и машины. Назовите publisher, имена суставов, units и источник stamp.
Частота сама по себе не подтверждает правильность оси.

По готовности: `ros2 topic bw /joint_states` и `ros2 topic delay /joint_states`.
`delay` здесь означает now − header.stamp при совместимых часах; это не отдельное
измерение сетевого пути. Наблюдайте structured log
`event=sample synthetic=true tick=... turret_rad=...` в терминале A.

Сформулируйте симптом: «при изменении `turret_joint` меняется высота щупа, хотя
контракт требует неизменные 0.34 m». Не начинайте с догадки «сломался ROS».

## 25–40 минут. Сохранить воспроизводимый инцидент

Пока A работает, в B запишите только вход модели:

```bash
mkdir -p evidence
ros2 bag record --storage sqlite3 -o evidence/axis-input --topics /joint_states
```

Через 12–15 секунд нажмите Ctrl+C и дождитесь закрытия recorder. Имя выходной
папки должно быть новым; для повторного опыта используйте `axis-input-02`.

В B:

```bash
ros2 bag info evidence/axis-input
MODEL_SHARE="$(ros2 pkg prefix inspection_cart)/share/inspection_cart/urdf"
cp "$MODEL_SHARE/cart-broken.urdf" evidence/incident-model.urdf
ros2 run inspection_cart provenance \
  --model evidence/incident-model.urdf \
  --source-root . --output evidence/model-provenance.json
ros2 doctor --report > evidence/ros-doctor.txt
```

Сохраните команду запуска и короткое описание expected/actual рядом с bag.
`metadata.yaml` хранит topic/type, duration, storage и message counts; он не
содержит вашу модель, значения параметров, commit и все логи процесса.
SHA-256 фиксирует точные байты URDF, даже если рабочее дерево ещё не закоммичено.

Checkpoint: bag содержит `/joint_states`, а не `/tf`; модель сохранена отдельно.
Проверьте `source_commit` и `source_dirty` в JSON; при копировании без `.git`
commit будет `null` — тогда укажите commit исходного репозитория в отчёте вручную.

## 40–55 минут. Тест до исправления

Остановите launch A через Ctrl+C. Сначала по схеме и контракту вычислите положение
`probe_tip` при `turret_joint = +π/2`: что станет с offset +0.30 m вдоль X после
положительного поворота вокруг Z? До исправления модели напишите собственный
маленький тест. В B:

```bash
mkdir -p tests
```

Создайте `tests/test_student_regression.py`. Сначала сформулируйте ожидаемые
координаты сами; затем сравните с учебным вариантом:

```python
from math import pi
from pathlib import Path
import pytest
from inspection_cart.model import frame_position

def test_probe_goes_left_at_positive_quarter_turn():
    model = Path(__file__).resolve().parents[1] / "src/inspection_cart/urdf/cart-broken.urdf"
    actual = frame_position(model, "probe_tip", {"turret_joint": pi / 2})
    assert actual == pytest.approx((0.07, 0.30, 0.34), abs=1e-6)
```

Выполните **до** изменения URDF:

```bash
python3 -m pytest -q tests/test_student_regression.py
```

Ожидается FAIL. При `+π/2` контракт требует `(0.07, 0.30, 0.34)` m, broken
модель даёт `(0.37, −0.04, 0.30)` m. При нуле дефект не проявляется. Сравните
joint origin, axis и неподвижный offset щупа; не меняйте ожидаемые координаты,
чтобы получить зелёный тест.

Исправьте **только** `axis` сустава `turret_joint` в `cart-broken.urdf`: поворот
должен идти вокруг локальной вертикальной оси. Повторите ту же команду теста.
Она должна стать PASS. Готовый `src/inspection_cart/test/test_model.py` —
reference и помощь при нехватке времени: он содержит обе крайние позы и
проверку нуля. Его не нужно копировать вместо своего одного regression case.
Благодаря `--symlink-install` изменённый URDF читается
из source; для новых ресурсов/изменений setup.py нужна повторная сборка.
RSP уже прочитал описание на старте, поэтому после изменения нужен новый launch.
Сохраните отдельный `evidence/fixed-model.urdf` и provenance для него, тем же
способом, что и для исходного `incident-model.urdf`: у «до» и «после» разные hashes.

Checkpoint: объясните, какую физически осмысленную ошибку ловит этот тест. Его
результат относится к геометрическому контракту, а не к динамике или управлению.

## 55–75 минут. Один вход: до и после исправления

В A запустите display без источника states, с ROS simulated time:

```bash
ros2 launch inspection_cart view.launch.py \
  model:=broken state_source:=none use_sim_time:=true
```

В B:

```bash
ros2 bag play evidence/axis-input --clock --rate 0.5
```

Состояния из bag двигают модель с исправленной осью. Bag публикует `/joint_states`
и генерирует `/clock`; RSP и RViz используют `use_sim_time=true`. Новый
`robot_state_publisher` публикует `/tf` и `/tf_static`. Запущенного synthetic или
GUI publisher быть не должно. Не публикуйте `/tf` одновременно из bag и RSP.

В C во время replay:

```bash
ros2 topic info /joint_states --verbose
ros2 topic info /clock --verbose
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /rviz2 use_sim_time
ros2 run tf2_ros tf2_echo base_link probe_tip --ros-args -p use_sim_time:=true
```

Проверьте одного publisher `/joint_states` и `/clock`; в этом опыте оба — bag
player. Высота щупа остаётся 0.34 m. Остановите `tf2_echo` через Ctrl+C.
Первый/последний packet replay не обязан совпасть с точной границей sweep:
recorder подключился к уже работающему источнику.

Для сравнения «до» сохранённая `evidence/incident-model.urdf` остаётся broken.
Сначала сохраните исправленный исходник в `evidence/fixed-model.urdf`, затем
временно замените `src/inspection_cart/urdf/cart-broken.urdf` файлом
`evidence/incident-model.urdf`, перезапустите A и проиграйте **тот же** bag.
После опыта восстановите исходник из `evidence/fixed-model.urdf` и прогоните тест.
Проще при демонстрации преподавателя показать broken replay до редактирования,
затем перезапустить display после fix. Перед каждым replay запускайте новый RSP
и RViz: это убирает старые TF buffers и обратный скачок времени.

## 75–85 минут. Воспроизводимость и проверка установленного пакета

Остановите bag и launch, затем в B:

```bash
python3 tools/check_ros.py
python3 -m pytest -q src/inspection_cart/test tests/test_student_regression.py
colcon test --packages-select inspection_cart --event-handlers console_direct+
colcon test-result --verbose
```

Smoke test сам запускает headless display с synthetic source, ждёт реальные
changing states и TF, проверяет численный контракт дуги и завершает только свой
launch. Он ограничен wall timeout 12 s; ошибки запуска находятся в
`evidence/ros-smoke-launch.log`. В домене не должно быть других publishers.
CI может запускать pure tests без desktop; ROS smoke остаётся отдельным уровнем.

Изоляция: не используйте чужой ROS domain, не зависите от ранее открытого RViz,
не оставляйте background publisher. Чистая fixture — несколько заданных angles
и маленькая модель. GTest применим к C++ math, pytest к Python; `launch_testing`
подходит для assertions над жизненным циклом нескольких nodes. В этой короткой
практике numerical regression и ROS smoke уже выполняются; отдельный GTest или
launch_testing suite не выдаются за реализованные.

## 85–90 минут. Следующий самостоятельный шаг

Покажите соседу свой sketch и два контрольных положения. Назовите один шаг,
который сможете проверить без Gazebo. Домашка 1 — собственная visual/kinematic
модель, запуск RViz и доказательства корректной сборки. Следующие две части
проекта добавят физику/управление и поведение с проверкой результата.

В `evidence/incident.md` запишите пять строк: команда, expected, actual, что
проверили, минимальный fix. Нужный комплект: bag + metadata, model snapshot/hash,
environment/commit, короткие логи и test output. Большое видео его не заменяет.
Завершение: Ctrl+C в работающих foreground процессах. L05 окружение выключается
из host terminal `task05/` через `./course down` после окончания занятия.
