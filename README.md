# DTSX — DTS:X Decoder

> **An experimental C++ decoder and renderer for DTS:X, immersive audio, and
> multichannel PCM workflows.**

[![License](https://img.shields.io/badge/License-PolyForm%20Noncommercial%201.0.0-blue.svg)](LICENSE)

**DTSX** is a technical project focused on decoding DTS Core, DTS-HD, legacy
DTS:X ExSS/XLL object audio, and experimental DTS-UHD/ACE/P2 streams. It
provides a command-line pipeline for stream inspection, object metadata
extraction, speaker-layout rendering, and PCM24 WAV output.

The project is intended for developers, audio engineers, researchers, and
enthusiasts working with immersive audio, spatial rendering, multichannel
audio, and codec interoperability.

> **Note:** DTSX is an independent project. DTS, DTS:X, DTS-HD, DTS-UHD, IMAX,
> and other third-party names and trademarks are used only to describe
> technical compatibility or the subject of research. This project is not
> affiliated with or endorsed by the respective rights holders.

---

## Overview

The decoder combines native C/C++ stream parsing and rendering with FFmpeg for
container demultiplexing and supported channel-bed decoding. Its main areas are:

- DTS Core, DTS-HD, XLL, LBR, ExSS, and UHD stream processing;
- legacy DTS:X object metadata and waveform extraction;
- object rendering to common surround and height layouts;
- experimental internal ACE/P2 decoding with runtime fallback;
- multichannel PCM24 WAV and RF64 output;
- metadata and object-sidecar export for analysis workflows.

The repository is intended primarily for technical, research, experimental,
and non-commercial use.

## Project Scope

DTSX is a decoder and research-oriented command-line utility, not a general
media player. The current implementation targets deterministic decoding,
metadata inspection, speaker-layout conversion, and reproducible audio output.

Supported output layouts include mono, stereo, 5.1, 7.1, 5.1.2, 5.1.4,
7.1.2, and 7.1.4. Arbitrary room geometry and unrestricted speaker coordinates
are outside the currently supported CLI scope.

## Requirements

- Windows x64;
- Microsoft Visual C++ toolchain with C++17 support;
- FFmpeg and FFprobe available through `PATH`, or supplied explicitly through
  command-line options.

## Build

```powershell
.\build.bat
```

The resulting executable is written to `bin\Release\dtsx-decode.exe` by
default.

## Quick Start

```text
dtsx-decode --input film.mkv --layout 7.1.4 --output film_7.1.4.wav
dtsx-decode --input track.dtshd --render objects --metadata-output metadata.jsonl
```

## Project Status

DTSX is an evolving technical project. Decoder behavior, supported bitstream
features, command-line options, and internal APIs may change as compatibility
work continues. Review the source, current revision, and license before using
the project in another workflow.

## License

This project is distributed under the **PolyForm Noncommercial License 1.0.0**.
See [LICENSE](LICENSE) for the complete terms.

Review the license carefully before using the project in a product, paid
service, commercial development workflow, or any activity with an anticipated
commercial application.

The PolyForm license covers the original DTSX project code. Bundled third-party
components remain subject to their respective copyright and license terms.

## Contributing

Technical contributions are welcome. Bug reports should include the operating
system, decoder version, input stream characteristics, selected layout, exact
command line, and a minimal reproducible sample when redistribution is allowed.
Keep pull requests focused and avoid committing copyrighted media fixtures,
generated build products, decoder dumps, or unrelated SDK material.

## Keywords

**DTS:X decoder · DTS-HD · DTS-UHD · immersive audio · spatial audio · object
audio · multichannel audio · surround sound · 3D audio · PCM24 · audio codec**

---

# Русский

# DTSX — декодер DTS:X

Консольная C++-утилита для подготовки DTS:X decoder/render pipeline. Она принимает
`mkv`, `mka`, `mp4`, `m4a`, `m2ts`, `dts` или `dtshd`, использует установленные `ffprobe` и `ffmpeg`,
проверяет входной и выходной layout и пишет многоканальный PCM24 WAV.

Режим `bed` передаёт DTS Core/DTS-HD/XLL открытому FFmpeg decoder и пишет
channel-based PCM без объектных метаданных. Режим `objects` демультиплексирует
elementary DTS через FFmpeg, разбирает legacy DTS:X ExSS/XLL и объектные
метаданные, рендерит доступные object waveforms в выбранный layout и смешивает
их с channel bed, декодированным FFmpeg.

## Использование

```text
dtsx-decode --input film.mkv --layout 7.1 --output film_7.1.wav
dtsx-decode -i track.dtshd --layout 5.1(side)
dtsx-decode -i film.mkv --audio-track 1 --render objects --verbose
dtsx-decode -i film.mkv --render objects --objects-output-dir stems
dtsx-decode -i film.mkv --metadata-output metadata.jsonl
dtsx-decode -i film.mkv --objects-output-dir stems --objects-output-bed
dtsx-decode -i imax.dtshd --layout 7.1.4 --imax-dsp -o imax_7.1.4.wav
```

Без `--output` имя создаётся рядом со входом:

```text
film_7.1.wav
```

## Параметры

| Параметр | Влияние |
|---|---|
| `-i`, `--input PATH` | Входной контейнер или elementary stream. Разрешены `.mkv`, `.mka`, `.mp4`, `.m4a`, `.m2ts`, `.dts`, `.dtshd`. |
| `-o`, `--output PATH` | Явный путь выходного WAV. Указанное имя никогда не переписывается. |
| `--audio-track N` | Номер аудиодорожки среди аудиодорожек, начиная с нуля. Передаётся FFprobe и FFmpeg как `a:N`. |
| `--layout NAME` | Обязательный выходной layout, если FFprobe не сообщил однозначный layout. Определяет количество, назначение и порядок каналов. |
| `--sample-rate HZ` | Частота выходного PCM. Без параметра сохраняется частота выбранной дорожки. |
| `--render bed` | Декодировать channel-based Core/HD/XLL через FFmpeg. Объектные метаданные не применяются. |
| `--render objects` | Декодировать legacy DTS:X object waveforms из ExSS/XLL, применить нативно восстановленные metadata gains и добавить результат к FFmpeg bed. Это режим по умолчанию. |
| `--metadata-output PATH` | Сохранить диагностический JSONL с Core/ExSS/XLL headers, presentations, object IDs и point-source metadata. |
| `--objects-output-dir PATH` | Сохранить доступные монофонические object waveforms как отдельные PCM24 WAV и JSONL рядом с ними. |
| `--objects-output-bed` | Только с `--objects-output-dir`: добавить `bed.wav` и `bed.json` — многоканальный bed без объектов для будущего object-viewer. |
| `--imax-dsp` | Только для потока с IMAX Type1 Certified Content применить восстановленный AVRx0-профиль: 70 Гц, относительный LFE trim +10 дБ и bass collection из каналов Small. |
| `--imax-small LIST` | Конфигурация физических колонок для `--imax-dsp`: `all` (по умолчанию), `none` или имена через запятую, например `FC,SL,SR`. Small-канал проходит HP4 70 Гц, а его LP4-бас суммируется в LFE; Large остаётся полнополосным. |
| `--p2-decoder MODE` | P2 backend: `dll` (по умолчанию) или `internal` для диагностического ACE PCM. Внутренний backend использует DLL fallback, пока не покрыты все каналы кадра. |
| `--ffmpeg PATH` | Явный путь к `ffmpeg.exe`. По умолчанию используется `ffmpeg` из `PATH`. |
| `--ffprobe PATH` | Явный путь к `ffprobe.exe`. По умолчанию используется `ffprobe` из `PATH`. |
| `--overwrite` | Разрешить замену существующего выходного файла. Без флага существующий файл сохраняется. |
| `--verbose` | Показать probe-информацию, channel order и запускаемые дочерние процессы. |
| `--help` | Показать справку и успешно завершить работу. |
| `--version` | Показать версию. |

Поддерживаемые layouts первого этапа:

```text
mono
stereo
5.1
5.1(side)
7.1
5.1.2
5.1.4
7.1.2
7.1.4
```

Object renderer воспроизводит стандартные destination layouts из списка выше.
Физические размеры комнаты и произвольные абсолютные координаты колонок не
считаются подтверждённой возможностью декодера и в текущий CLI не входят.
DTS-UHD/ACE/P2 оставлен отдельным низкоприоритетным этапом; текущий object path
предназначен для legacy DTS:X на базе ExSS/XLL.

## Координатный sidecar объекта

Каждая строка с `coordinateStatus: "decoded"` описывает интервал
`ptsSamples .. ptsSamples + durationSamples`. Помимо сферических координат она
содержит:

- `metadataMode` - режим объектных metadata;
- `renderable` - допускает ли native renderer этот point source в данном
  режиме;
- `audioActive` - есть ли ненулевой PCM sample в waveform за этот интервал;
- `peakSample` - максимальный абсолютный PCM24 sample за интервал.

`audioActive` является вычисленным состоянием waveform, а не отдельным флагом
битстрима. `metadata_present` для определения активности не используется:
он сообщает о наличии обновления metadata, а не о звучании объекта.

## Выходной WAV

Файл содержит PCM24 little-endian и `WAVE_FORMAT_EXTENSIBLE` с channel mask.
В заголовке заранее резервируется `ds64`, поэтому writer может завершить большой
файл как RF64, не меняя расположение аудиоданных.

## Сборка и lint

Сборка подготовлена в стиле `tools/auro3d-decode`:

```powershell
.\build.bat
```

Проверка форматирования и состава исходников:

```powershell
powershell -ExecutionPolicy Bypass -File .\lint.ps1
```

## Статус проекта

DTSX — развивающийся технический проект. Поддержка отдельных вариантов
битстрима и внутреннего ACE/P2 backend может меняться. Перед интеграцией
проверьте актуальную версию исходников, историю изменений и условия лицензии.

Проект является независимым. Названия DTS, DTS:X, DTS-HD, DTS-UHD, IMAX и
другие сторонние товарные знаки используются исключительно для описания
технической совместимости или предмета исследования.

## Лицензия

Проект распространяется по лицензии **PolyForm Noncommercial License 1.0.0**.
Полный текст находится в файле [LICENSE](LICENSE).

Перед использованием проекта в коммерческом продукте, платном сервисе,
коммерческой разработке или другой деятельности с предполагаемым коммерческим
применением обязательно ознакомьтесь с условиями лицензии.

Лицензия PolyForm распространяется на оригинальный код проекта DTSX. Сторонние
компоненты сохраняют собственные авторские права и условия лицензирования.

## Участие в разработке

Технические улучшения приветствуются. В отчёте об ошибке укажите версию
декодера, параметры входного потока, выбранный layout, точную командную строку
и минимальный воспроизводимый пример, если его разрешено распространять.
Не добавляйте в pull request медиасэмплы, результаты сборки, дампы декодеров и
посторонние материалы SDK.
