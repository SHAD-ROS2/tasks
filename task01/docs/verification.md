# L01 · verification

> Student ID и Commit SHA сюда вписывать не нужно: преподаватель определяет их
> автоматически по вашему private-репозиторию и ветке `master`.

## Обязательные проверки

- `course test --case found`: pass, 55/55, position_error_m ≈ 0.016–0.022, explicit_stop=true.
- `course test --case absent`: pass, 45/45, explicit_stop=true.

Дополнительно вручную проверено `course goal --area 2 2 8 8` на живом сервере:
терминальный status 4 (SUCCEEDED), target_found=true, feedback приходил регулярно
на всём протяжении миссии.

## Известная граница решения

Значение `distance_covered` в feedback обнуляется в момент перехода от фазы
raster-поиска к `RectangleProbe` (у него собственный счётчик дистанции, начинающийся
с нуля), поэтому метрика не монотонна на всей миссии. Задание явно не требует
монотонной метрики пути или целевой частоты feedback, поэтому это не влияет на core
score, но отмечено как известное ограничение.

## Использование LLM
LLM (Claude, ассистент в Claude Code) написал реализацию `Mission.step()` в
`mission.py` (делегирование в `RectangleProbe`, движение по `raster` через `drive_to`,
переход в `absent` после исчерпания маршрута) и правку `terminal_kind()` в
`server.py` (маппинг `found`/`absent` в `succeed`). Результат проверен тем же
набором core-тестов: `course test --case found` и `course test --case absent`,
оба pass, плюс ручной прогон `course goal`.
