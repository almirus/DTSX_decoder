"use strict";

const COLORS = [
  "#48dce5", "#ff7c8a", "#b58cff", "#ffd166", "#62df9b",
  "#ff9f55", "#65a9ff", "#f079cf", "#b9df61", "#ff625f",
];
const OBJECT_FADE_SECONDS = 0.5;
const OBJECT_TRAIL_MAX_POINTS = 96;
const STREAMING_AUDIO_THRESHOLD_BYTES = 192 * 1024 * 1024;
const STREAMING_JSON_THRESHOLD_BYTES = 32 * 1024 * 1024;
const REFERENCE_SPEAKER_CHANNELS = [
  "FL", "FR", "FC", "LFE", "BL", "BR",
  "SL", "SR", "TFL", "TFR", "TBL", "TBR",
];

const ui = {
  openFolder: document.querySelector("#openFolder"),
  folderFallback: document.querySelector("#folderFallback"),
  folderName: document.querySelector("#folderName"),
  objectList: document.querySelector("#objectList"),
  objectCount: document.querySelector("#objectCount"),
  objectsMaster: document.querySelector("#objectsMaster"),
  objectTrails: document.querySelector("#objectTrails"),
  objectTrailLength: document.querySelector("#objectTrailLength"),
  objectTrailLengthValue: document.querySelector("#objectTrailLengthValue"),
  objectDepthScale: document.querySelector("#objectDepthScale"),
  objectMetadataSize: document.querySelector("#objectMetadataSize"),
  objectMetadataGain: document.querySelector("#objectMetadataGain"),
  objectGuideLines: document.querySelector("#objectGuideLines"),
  bedSpeakers: document.querySelector("#bedSpeakers"),
  centerSofa: document.querySelector("#centerSofa"),
  masterVolume: document.querySelector("#masterVolume"),
  masterVolumeValue: document.querySelector("#masterVolumeValue"),
  playPause: document.querySelector("#playPause"),
  playIcon: document.querySelector("#playIcon"),
  stop: document.querySelector("#stop"),
  loop: document.querySelector("#loop"),
  timeline: document.querySelector("#timeline"),
  timelineRuler: document.querySelector("#timelineRuler"),
  timelineEvents: document.querySelector("#timelineEvents"),
  currentTime: document.querySelector("#currentTime"),
  duration: document.querySelector("#duration"),
  canvas: document.querySelector("#scene"),
  sceneStatus: document.querySelector("#sceneStatus"),
  toast: document.querySelector("#toast"),
};

const state = {
  context: null,
  masterGain: null,
  limiter: null,
  objects: [],
  objectsEnabled: true,
  objectSettings: {
    trailsEnabled: true,
    trailSeconds: 0.35,
    depthScaleEnabled: true,
    metadataSizeEnabled: false,
    metadataGainEnabled: true,
    guideLinesEnabled: true,
    bedSpeakersEnabled: true,
    sofaEnabled: true,
  },
  beds: [],
  duration: 0,
  offset: 0,
  startedAt: 0,
  playing: false,
  resumeAfterSeek: false,
  sources: [],
  generation: 0,
  camera: { yaw: -0.68, pitch: -0.28, zoom: 1.0 },
};

if (!window.showDirectoryPicker) document.body.classList.add("no-directory-picker");

function showError(message) {
  ui.toast.textContent = message;
  ui.toast.classList.add("visible");
  clearTimeout(showError.timer);
  showError.timer = setTimeout(() => ui.toast.classList.remove("visible"), 5200);
}

async function ensureAudio() {
  if (!state.context) {
    state.context = new AudioContext();
    state.masterGain = state.context.createGain();
    state.limiter = state.context.createDynamicsCompressor();
    state.masterGain.gain.value = Number(ui.masterVolume.value);
    state.limiter.threshold.value = -1;
    state.limiter.knee.value = 0;
    state.limiter.ratio.value = 20;
    state.limiter.attack.value = 0.003;
    state.limiter.release.value = 0.12;
    state.masterGain.connect(state.limiter);
    state.limiter.connect(state.context.destination);
  }
  if (state.context.state === "suspended") await state.context.resume();
}

function stemBase(name) {
  return name.replace(/\.coordinates\.jsonl$/i, "").replace(/\.wav$/i, "");
}

function chunkId(view, offset) {
  return String.fromCharCode(
    view.getUint8(offset), view.getUint8(offset + 1),
    view.getUint8(offset + 2), view.getUint8(offset + 3));
}

async function inspectWav(file) {
  const header = await file.slice(0, Math.min(file.size, 1024 * 1024))
    .arrayBuffer();
  const view = new DataView(header);
  if (view.byteLength < 44
      || !["RIFF", "RF64"].includes(chunkId(view, 0))
      || chunkId(view, 8) !== "WAVE") {
    throw new Error(`${file.name}: неподдерживаемый WAV-контейнер`);
  }
  let offset = 12;
  let format = null;
  let dataSize64 = null;
  let sampleCount64 = null;
  let dataOffset = null;
  let dataSize = null;
  while (offset + 8 <= view.byteLength) {
    const id = chunkId(view, offset);
    const size = view.getUint32(offset + 4, true);
    const payload = offset + 8;
    if (id === "ds64" && size >= 24 && payload + 24 <= view.byteLength) {
      dataSize64 = Number(view.getBigUint64(payload + 8, true));
      sampleCount64 = Number(view.getBigUint64(payload + 16, true));
    } else if (id === "fmt " && size >= 16
        && payload + 16 <= view.byteLength) {
      format = {
        tag: view.getUint16(payload, true),
        numberOfChannels: view.getUint16(payload + 2, true),
        sampleRate: view.getUint32(payload + 4, true),
        blockAlign: view.getUint16(payload + 12, true),
        bitsPerSample: view.getUint16(payload + 14, true),
      };
    } else if (id === "data") {
      dataOffset = payload;
      dataSize = size === 0xFFFFFFFF ? dataSize64 : size;
      break;
    }
    offset = payload + size + (size & 1);
  }
  if (!format || dataOffset === null || !Number.isFinite(dataSize)) {
    throw new Error(`${file.name}: неполный заголовок WAV/RF64`);
  }
  const frames = Number.isFinite(sampleCount64) && sampleCount64 > 0
    ? sampleCount64
    : Math.floor(dataSize / format.blockAlign);
  return {
    ...format,
    dataOffset,
    dataSize,
    frames,
    duration: frames / format.sampleRate,
    rf64: chunkId(view, 0) === "RF64",
  };
}

async function createStreamingAudio(file, wav) {
  const objectUrl = URL.createObjectURL(file);
  const media = document.createElement("audio");
  media.preload = "metadata";
  media.playsInline = true;
  media.src = objectUrl;
  try {
    await new Promise((resolve, reject) => {
      const loaded = () => {
        cleanup();
        resolve();
      };
      const failed = () => {
        cleanup();
        reject(new Error(
          `${file.name}: браузер не смог открыть потоковый WAV/RF64`));
      };
      const cleanup = () => {
        media.removeEventListener("loadedmetadata", loaded);
        media.removeEventListener("error", failed);
      };
      media.addEventListener("loadedmetadata", loaded);
      media.addEventListener("error", failed);
      media.load();
    });
  } catch (error) {
    URL.revokeObjectURL(objectUrl);
    throw error;
  }
  return {
    kind: "media",
    media,
    mediaNode: state.context.createMediaElementSource(media),
    objectUrl,
    duration: wav.duration,
    sampleRate: wav.sampleRate,
    numberOfChannels: wav.numberOfChannels,
    wav,
  };
}

async function loadAudioAsset(file) {
  const wav = await inspectWav(file);
  if (file.size >= STREAMING_AUDIO_THRESHOLD_BYTES) {
    return createStreamingAudio(file, wav);
  }
  const buffer = await state.context.decodeAudioData(await file.arrayBuffer());
  return {
    kind: "buffer",
    buffer,
    duration: buffer.duration,
    sampleRate: buffer.sampleRate,
    numberOfChannels: buffer.numberOfChannels,
    wav,
  };
}

function releaseAudioAsset(asset) {
  if (!asset || asset.kind !== "media") return;
  asset.media.pause();
  asset.media.removeAttribute("src");
  asset.media.load();
  asset.mediaNode.disconnect();
  URL.revokeObjectURL(asset.objectUrl);
}

function bedDownmix(channel) {
  const side = Math.SQRT1_2;
  const center = Math.SQRT1_2;
  const lfe = 0.5;
  const name = String(channel).toUpperCase();
  if (name === "FL") return { left: 1, right: 0 };
  if (name === "FR") return { left: 0, right: 1 };
  if (name === "FC" || name === "C") {
    return { left: center, right: center };
  }
  if (name === "LFE" || name === "LFE1" || name === "LFE2") {
    return { left: lfe, right: lfe };
  }
  if (["BL", "SL", "TFL", "TBL", "WL"].includes(name)) {
    return { left: side, right: 0 };
  }
  if (["BR", "SR", "TFR", "TBR", "WR"].includes(name)) {
    return { left: 0, right: side };
  }
  if (["BC", "TC", "TFC", "TRC"].includes(name)) {
    return { left: center, right: center };
  }
  throw new Error(`bed.json: неизвестный bed-канал ${channel}`);
}

function downmixLabel(matrix) {
  const db = (value) => value >= 0.999
    ? "0 dB"
    : `${(20 * Math.log10(value)).toFixed(1)} dB`;
  if (matrix.left && matrix.right) return `L/R ${db(matrix.left)}`;
  if (matrix.left) return `L ${db(matrix.left)}`;
  return `R ${db(matrix.right)}`;
}

function disconnectBed(bed) {
  releaseAudioAsset(bed.audio);
  bed.splitter.disconnect();
  for (const channel of bed.channels) {
    channel.leftGain.disconnect();
    channel.rightGain.disconnect();
  }
  bed.merger.disconnect();
  bed.outputGain.disconnect();
}

function setBedChannelGain(channel, enabled, time) {
  channel.enabled = enabled && !channel.silent;
  channel.leftGain.gain.setTargetAtTime(
    channel.enabled ? channel.matrix.left : 0, time, 0.008);
  channel.rightGain.gain.setTargetAtTime(
    channel.enabled ? channel.matrix.right : 0, time, 0.008);
}

function updateBedMaster(bed, time) {
  bed.enabled = bed.channels.some((channel) => channel.enabled);
  bed.outputGain.gain.setTargetAtTime(
    bed.enabled ? 1 : 0, time, 0.008);
}

function setBedEnabled(bed, enabled, time) {
  for (const channel of bed.channels) {
    setBedChannelGain(channel, enabled, time);
  }
  updateBedMaster(bed, time);
}

function createBedRouting(definition, audio) {
  if (!Array.isArray(definition.channels) || !definition.channels.length) {
    throw new Error("bed.json: отсутствует массив channels");
  }
  if (audio.numberOfChannels !== definition.channels.length) {
    throw new Error(
      `${definition.audioFile}: в WAV ${audio.numberOfChannels} каналов, `
      + `в bed.json указано ${definition.channels.length}`);
  }
  const splitter = state.context.createChannelSplitter(audio.numberOfChannels);
  const merger = state.context.createChannelMerger(2);
  const outputGain = state.context.createGain();
  merger.channelInterpretation = "discrete";
  outputGain.gain.value = 1;
  merger.connect(outputGain);
  outputGain.connect(state.masterGain);
  const channels = definition.channels.map((name, index) => {
    const matrix = bedDownmix(name);
    let peak = 0;
    if (audio.kind === "buffer") {
      const samples = audio.buffer.getChannelData(index);
      for (const sample of samples) peak = Math.max(peak, Math.abs(sample));
    }
    const silent = audio.kind === "buffer" && peak === 0;
    const leftGain = state.context.createGain();
    const rightGain = state.context.createGain();
    leftGain.gain.value = silent ? 0 : matrix.left;
    rightGain.gain.value = silent ? 0 : matrix.right;
    splitter.connect(leftGain, index, 0);
    splitter.connect(rightGain, index, 0);
    leftGain.connect(merger, 0, 0);
    rightGain.connect(merger, 0, 1);
    return {
      name,
      index,
      enabled: !silent,
      silent,
      peak,
      matrix,
      leftGain,
      rightGain,
    };
  });
  if (audio.kind === "media") audio.mediaNode.connect(splitter);
  return {
    name: definition.audioFile,
    layout: definition.layout || `${channels.length} ch`,
    audio,
    buffer: audio.buffer || null,
    duration: audio.duration,
    enabled: channels.some((channel) => channel.enabled),
    splitter,
    merger,
    outputGain,
    channels,
  };
}

function coordinatePoint(value, header = {}) {
  if (value.coordinateStatus !== "decoded") return null;
  const ptsSamples = Number.isFinite(value.startSamples)
    ? value.startSamples
    : value.ptsSamples;
  const sampleRate = Number.isFinite(value.sampleRate)
    ? value.sampleRate
    : header.sampleRate;
  if (![ptsSamples, sampleRate, value.azimuthDeg, value.elevationDeg]
      .every(Number.isFinite)) return null;
  return {
    ptsSamples,
    durationSamples: Number.isFinite(value.durationSamples)
      ? value.durationSamples
      : 0,
    sampleRate,
    time: ptsSamples / sampleRate,
    duration: Number.isFinite(value.durationSamples)
      ? value.durationSamples / sampleRate
      : 0,
    azimuth: value.azimuthDeg,
    elevation: value.elevationDeg,
    distance: Number.isFinite(value.distance) ? value.distance : 1,
    widthDeg: Number.isFinite(value.widthDeg) ? value.widthDeg : 0,
    heightDeg: Number.isFinite(value.heightDeg) ? value.heightDeg : 0,
    rotationDeg: Number.isFinite(value.rotationDeg) ? value.rotationDeg : 0,
    objectId: value.objectId ?? header.objectId,
    waveformId: value.waveformId ?? header.waveformId,
    renderable: value.renderable !== false,
    audioActive: value.audioActive !== false,
    peakSample: Number.isFinite(value.peakSample) ? value.peakSample : 0,
    objectGainPresent: value.objectGainPresent === true,
    objectGainCode: Number.isFinite(value.objectGainCode)
      ? value.objectGainCode
      : null,
    objectGainExponent: Number.isFinite(value.objectGainExponent)
      ? value.objectGainExponent
      : null,
    presentationGainCode: Number.isFinite(value.presentationGainCode)
      ? value.presentationGainCode
      : null,
    metadataGainQ23: Number.isFinite(value.metadataGainQ23)
      ? value.metadataGainQ23
      : null,
    metadataGainLinear: effectiveMetadataGain(value),
    metadataGainDb: Number.isFinite(value.metadataGainDb)
      ? value.metadataGainDb
      : null,
    interpolation: value.interpolation || "linear",
  };
}

function effectiveMetadataGain(value) {
  if (Number.isFinite(value.metadataGainLinear)) {
    return Math.max(0, value.metadataGainLinear);
  }
  if (Number.isFinite(value.metadataGainQ23)) {
    return Math.max(0, value.metadataGainQ23 / 8388608);
  }
  if (Number.isFinite(value.metadataGainDb)) {
    return Math.pow(10, value.metadataGainDb / 20);
  }
  return 1;
}

function parseCoordinateLine(line, lineNumber, records) {
  if (!line) return;
  try {
    const value = JSON.parse(line);
    if (value.type === "header" && value.version >= 2) {
      records.header = value;
    } else if (value.type === "state") {
      records.states.push(value);
    } else if (value.type === "activity") {
      records.activities.push(value);
    } else {
      const point = coordinatePoint(value);
      if (point) records.legacy.push(point);
    }
  } catch (error) {
    console.warn(`JSONL, строка ${lineNumber}:`, error);
  }
}

function activityAt(activities, sample) {
  let low = 0;
  let high = activities.length;
  while (low < high) {
    const mid = Math.floor((low + high) / 2);
    if (activities[mid].startSamples <= sample) low = mid + 1;
    else high = mid;
  }
  const activity = activities[low - 1];
  if (!activity) return null;
  const end = activity.startSamples + activity.durationSamples;
  return sample < end ? activity : null;
}

function finishCoordinateRecords(records) {
  const points = records.legacy;
  const header = records.header || {};
  const activities = records.activities
    .filter((entry) => Number.isFinite(entry.startSamples)
      && Number.isFinite(entry.durationSamples)
      && entry.durationSamples > 0)
    .sort((a, b) => a.startSamples - b.startSamples);
  const hasActivityTimeline = activities.length > 0;
  for (const state of records.states) {
    if (!Number.isFinite(state.startSamples)
        || !Number.isFinite(state.durationSamples)
        || state.durationSamples <= 0) continue;
    const start = state.startSamples;
    const end = start + state.durationSamples;
    const boundaries = [start, end];
    for (const activity of activities) {
      const activityEnd = activity.startSamples + activity.durationSamples;
      if (activityEnd <= start) continue;
      if (activity.startSamples >= end) break;
      boundaries.push(Math.max(start, activity.startSamples));
      boundaries.push(Math.min(end, activityEnd));
    }
    boundaries.sort((a, b) => a - b);
    const uniqueBoundaries = boundaries.filter(
      (value, index) => index === 0 || value !== boundaries[index - 1]);
    for (let index = 0; index + 1 < uniqueBoundaries.length; index += 1) {
      const segmentStart = uniqueBoundaries[index];
      const segmentEnd = uniqueBoundaries[index + 1];
      if (segmentEnd <= segmentStart) continue;
      const activity = activityAt(activities, segmentStart);
      const point = coordinatePoint({
        ...state,
        startSamples: segmentStart,
        durationSamples: segmentEnd - segmentStart,
        audioActive: hasActivityTimeline
          ? Boolean(activity) && activity.active !== false
          : true,
        peakSample: activity?.peakSample || 0,
      }, header);
      if (point) points.push(point);
    }
  }
  points.sort((a, b) => a.time - b.time);
  return points;
}

function coordinateRecords() {
  return { header: null, states: [], activities: [], legacy: [] };
}

function parseCoordinates(text) {
  const records = coordinateRecords();
  for (const [index, raw] of text.split(/\r?\n/).entries()) {
    parseCoordinateLine(raw.trim(), index + 1, records);
  }
  return finishCoordinateRecords(records);
}

async function parseCoordinatesFile(file) {
  if (file.size < STREAMING_JSON_THRESHOLD_BYTES || !file.stream) {
    return parseCoordinates(await file.text());
  }
  const records = coordinateRecords();
  const reader = file.stream().getReader();
  const decoder = new TextDecoder();
  let remainder = "";
  let lineNumber = 0;
  while (true) {
    const { value, done } = await reader.read();
    const text = remainder + decoder.decode(value || new Uint8Array(), {
      stream: !done,
    });
    const lines = text.split(/\r?\n/);
    remainder = done ? "" : lines.pop();
    for (const raw of lines) {
      lineNumber += 1;
      parseCoordinateLine(raw.trim(), lineNumber, records);
    }
    if (done) break;
  }
  if (remainder) {
    parseCoordinateLine(remainder.trim(), lineNumber + 1, records);
  }
  return finishCoordinateRecords(records);
}

function findStaticCoordinate(points) {
  if (!points.length) return null;
  const audioActivePoints = points.filter((point) => point.audioActive);
  const relevantPoints = audioActivePoints.length
    ? audioActivePoints
    : points;
  const first = relevantPoints[0];
  const isStatic = relevantPoints.every((point) => {
    let azimuthDelta = Math.abs(point.azimuth - first.azimuth);
    if (azimuthDelta > 180) azimuthDelta = 360 - azimuthDelta;
    return azimuthDelta < 1e-6
      && Math.abs(point.elevation - first.elevation) < 1e-6
      && Math.abs(point.distance - first.distance) < 1e-6;
  });
  return isStatic ? first : null;
}

function prepareVisualTimeline(points, staticCoordinates) {
  let lastVisibleIndex = -1;
  let fadeStartSamples = null;
  for (const [index, point] of points.entries()) {
    const visible = (point.renderable || staticCoordinates)
      && point.audioActive;
    if (visible) {
      lastVisibleIndex = index;
      fadeStartSamples = null;
    } else if (fadeStartSamples === null) {
      const previous = points[lastVisibleIndex];
      fadeStartSamples = previous
        ? previous.ptsSamples + previous.durationSamples
        : point.ptsSamples;
    }
    point.visualActive = visible;
    point.lastVisibleIndex = lastVisibleIndex;
    point.fadeStartSamples = fadeStartSamples;
  }
}

async function loadFiles(files, folderLabel) {
  stopPlayback(true);
  for (const object of state.objects) {
    releaseAudioAsset(object.audio);
    object.gain.disconnect();
    object.metadataGain.disconnect();
    if (object.panner) object.panner.disconnect();
  }
  for (const bed of state.beds) disconnectBed(bed);
  await ensureAudio();
  const filesByName = new Map(
    files.map((file) => [file.name.toLowerCase(), file]));
  const bedDefinitions = [];
  for (const file of files) {
    if (!/\.json$/i.test(file.name)
        || /\.coordinates\.jsonl$/i.test(file.name)) continue;
    try {
      const value = JSON.parse(await file.text());
      if (value.type === "dtsx-object-viewer-bed"
          && value.kind === "multichannel_bed") {
        if (typeof value.audioFile !== "string") {
          throw new Error(`${file.name}: отсутствует audioFile`);
        }
        const audioFile = filesByName.get(value.audioFile.toLowerCase());
        if (!audioFile) {
          throw new Error(`${file.name}: не найден ${value.audioFile}`);
        }
        bedDefinitions.push({ definition: value, audioFile });
      }
    } catch (error) {
      if (file.name.toLowerCase() === "bed.json") throw error;
      console.warn(`${file.name}:`, error);
    }
  }
  const bedAudioNames = new Set(
    bedDefinitions.map((bed) => bed.audioFile.name.toLowerCase()));
  const byBase = new Map();
  for (const file of files) {
    if (!/\.(wav|coordinates\.jsonl)$/i.test(file.name)) continue;
    if (bedAudioNames.has(file.name.toLowerCase())) continue;
    const base = stemBase(file.name);
    if (!byBase.has(base)) byBase.set(base, {});
    if (/\.wav$/i.test(file.name)) byBase.get(base).wav = file;
    else byBase.get(base).coordinates = file;
  }

  const pairs = [...byBase.entries()].filter(([, value]) => value.wav);
  if (!pairs.length && !bedDefinitions.length) {
    throw new Error("В папке не найдены object WAV или bed.json");
  }

  ui.folderName.textContent = folderLabel;
  ui.folderName.title = folderLabel;
  ui.sceneStatus.innerHTML = "<strong>Читаю WAV и координаты…</strong><span>Подготовка Web Audio</span>";
  ui.sceneStatus.classList.remove("hidden");

  const decoded = await Promise.all(pairs.map(async ([name, pair], index) => {
    const audio = await loadAudioAsset(pair.wav);
    try {
      if (audio.numberOfChannels !== 1) {
        throw new Error(`${pair.wav.name}: ожидался mono WAV, найдено каналов: ${audio.numberOfChannels}`);
      }
      const coordinates = pair.coordinates
        ? await parseCoordinatesFile(pair.coordinates)
        : [];
      const staticPosition = findStaticCoordinate(coordinates);
      const staticCoordinates = Boolean(staticPosition);
      prepareVisualTimeline(coordinates, staticCoordinates);
      const panner = typeof state.context.createStereoPanner === "function"
        ? state.context.createStereoPanner()
        : null;
      return {
        name,
        fileName: pair.wav.name,
        audio,
        buffer: audio.buffer || null,
        duration: audio.duration,
        coordinates,
        staticCoordinates,
        staticPosition,
        enabled: true,
        gain: state.context.createGain(),
        metadataGain: state.context.createGain(),
        lastMetadataGain: null,
        panner,
        lastPan: 0,
        hasPanPosition: false,
        trail: [],
        lastTrailTime: null,
        lastTrailPosition: null,
        color: COLORS[index % COLORS.length],
        sampleRate: audio.sampleRate,
      };
    } catch (error) {
      releaseAudioAsset(audio);
      throw error;
    }
  }));

  const beds = await Promise.all(bedDefinitions.map(async (bed) => {
    const audio = await loadAudioAsset(bed.audioFile);
    try {
      return createBedRouting(bed.definition, audio);
    } catch (error) {
      releaseAudioAsset(audio);
      throw error;
    }
  }));

  for (const object of decoded) {
    if (object.audio.kind === "media") {
      object.audio.mediaNode.connect(object.gain);
    }
    object.gain.connect(object.metadataGain);
    if (object.panner) {
      object.panner.pan.value = 0;
      object.metadataGain.connect(object.panner);
      object.panner.connect(state.masterGain);
    } else {
      object.metadataGain.connect(state.masterGain);
    }
  }
  state.objects = decoded;
  state.objectsEnabled = decoded.some((object) => object.enabled);
  state.beds = beds;
  const playableDurations = [
    ...decoded.map((object) => object.duration),
    ...beds.map((bed) => bed.duration),
  ];
  state.duration = Math.max(...playableDurations);
  state.offset = 0;
  ui.timeline.max = String(state.duration);
  ui.timeline.value = "0";
  ui.timeline.disabled = false;
  ui.playPause.disabled = false;
  ui.stop.disabled = false;
  ui.duration.textContent = formatTime(state.duration);
  ui.currentTime.textContent = formatTime(0);
  ui.objectCount.textContent = String(decoded.length);
  ui.sceneStatus.classList.add("hidden");
  renderTimelineDecorations();
  renderObjectList();
  renderObjectsMaster();
}

function renderObjectList() {
  ui.objectList.replaceChildren();
  for (const [index, object] of state.objects.entries()) {
    const row = document.createElement("div");
    const audible = state.objectsEnabled && object.enabled;
    row.className = `object-row object-entry${audible ? "" : " muted"}`;
    const identity = object.coordinates[0];
    const audioMode = object.audio.kind === "media" ? "поток" : "buffer";
    const subtitle = identity
      ? `WF ${identity.waveformId} · ${
          object.staticCoordinates
            ? "статичный"
            : `${object.coordinates.length} поз.`
        } · ${audioMode}`
      : `нет decoded-координат · ${object.sampleRate} Hz · ${audioMode}`;
    row.innerHTML = `
      <i class="object-swatch" style="background:${object.color};color:${object.color}"></i>
      <span class="object-id" style="color:${object.color}"
            title="ID объекта">${escapeHtml(identity?.objectId ?? "?")}</span>
      <div class="object-copy">
        <div class="object-title" title="${escapeHtml(object.fileName)}">${escapeHtml(object.name)}</div>
        <div class="object-meta">${escapeHtml(subtitle)}</div>
      </div>
      <button class="object-toggle ${object.enabled ? "on" : ""}"
              aria-label="${object.enabled ? "Выключить" : "Включить"} ${escapeHtml(object.name)}"
              aria-pressed="${object.enabled}"></button>`;
    row.querySelector("button").addEventListener("click", () => {
      object.enabled = !object.enabled;
      state.objectsEnabled = state.objects.some((item) => item.enabled);
      object.gain.gain.setTargetAtTime(
        state.objectsEnabled && object.enabled ? 1 : 0,
        state.context.currentTime,
        0.008);
      renderObjectList();
      renderObjectsMaster();
    });
    ui.objectList.append(row);
  }
  for (const bed of state.beds) {
    const heading = document.createElement("div");
    heading.className = `bed-heading${bed.enabled ? "" : " muted"}`;
    heading.innerHTML = `
      <div>
        <p class="eyebrow">BED</p>
        <div class="bed-title">${escapeHtml(bed.name)}</div>
        <div class="object-meta">${escapeHtml(bed.layout)} · ${bed.channels.length} каналов · stereo downmix · ${bed.audio.kind === "media" ? "поток" : "buffer"}</div>
      </div>
      <button class="object-toggle ${bed.enabled ? "on" : ""}"
              aria-label="${bed.enabled ? "Выключить" : "Включить"} bed ${escapeHtml(bed.name)}"
              aria-pressed="${bed.enabled}"></button>`;
    heading.querySelector("button").addEventListener("click", () => {
      setBedEnabled(bed, !bed.enabled, state.context.currentTime);
      renderObjectList();
    });
    ui.objectList.append(heading);

    for (const channel of bed.channels) {
      const row = document.createElement("div");
      row.className = `object-row bed-channel${channel.enabled ? "" : " muted"}`;
      const channelState = channel.silent ? " · пустой" : "";
      row.innerHTML = `
        <i class="object-swatch bed-swatch"></i>
        <div class="object-copy">
          <div class="object-title">${escapeHtml(channel.name)}</div>
          <div class="object-meta">канал ${channel.index + 1} · ${escapeHtml(downmixLabel(channel.matrix))}${channelState}</div>
        </div>
        <button class="object-toggle ${channel.enabled ? "on" : ""}"
                aria-label="${channel.enabled ? "Выключить" : "Включить"} bed-канал ${escapeHtml(channel.name)}"
                aria-pressed="${channel.enabled}"
                ${channel.silent ? "disabled" : ""}></button>`;
      row.querySelector("button").addEventListener("click", () => {
        const now = state.context.currentTime;
        setBedChannelGain(channel, !channel.enabled, now);
        updateBedMaster(bed, now);
        renderObjectList();
      });
      ui.objectList.append(row);
    }
  }
}

function renderObjectsMaster() {
  ui.objectsMaster.disabled = !state.objects.length;
  ui.objectsMaster.classList.toggle("on", state.objectsEnabled);
  ui.objectsMaster.setAttribute("aria-pressed", String(state.objectsEnabled));
  ui.objectsMaster.setAttribute(
    "aria-label",
    state.objectsEnabled ? "Выключить все объекты" : "Включить все объекты");
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;").replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;").replaceAll('"', "&quot;");
}

function streamingAudioAssets() {
  return [
    ...state.objects.map((object) => object.audio),
    ...state.beds.map((bed) => bed.audio),
  ].filter((audio) => audio?.kind === "media");
}

function playbackTime() {
  if (!state.playing) return state.offset;
  const streaming = streamingAudioAssets()[0];
  if (streaming && Number.isFinite(streaming.media.currentTime)) {
    return Math.max(
      state.offset,
      Math.min(state.duration, streaming.media.currentTime));
  }
  return Math.max(
    state.offset,
    Math.min(
      state.duration,
      state.offset + state.context.currentTime - state.startedAt));
}

async function startPlayback() {
  if (!state.objects.length && !state.beds.length) return;
  await ensureAudio();
  if (state.offset >= state.duration - 0.001) state.offset = 0;
  for (const object of state.objects) {
    object.lastMetadataGain = null;
    updateObjectMetadataGain(object, state.offset, true);
  }
  state.generation += 1;
  const generation = state.generation;
  const streaming = streamingAudioAssets();
  for (const audio of streaming) {
    if (Math.abs(audio.media.currentTime - state.offset) > 0.01) {
      audio.media.currentTime = Math.min(state.offset, audio.duration);
    }
  }
  await Promise.all(streaming.map((audio) => audio.media.play()));
  const startAt = state.context.currentTime + 0.01;
  state.startedAt = startAt;
  const playables = [
    ...state.objects.filter((object) => object.buffer).map((object) => ({
      buffer: object.buffer,
      connect: (source) => source.connect(object.gain),
      prepare: () => object.gain.gain.setValueAtTime(
        state.objectsEnabled && object.enabled ? 1 : 0, startAt),
    })),
    ...state.beds.filter((bed) => bed.buffer).map((bed) => ({
      buffer: bed.buffer,
      connect: (source) => source.connect(bed.splitter),
      prepare: () => bed.outputGain.gain.setValueAtTime(
        bed.enabled ? 1 : 0, startAt),
    })),
  ];
  state.sources = playables.map((playable) => {
    const source = state.context.createBufferSource();
    source.buffer = playable.buffer;
    playable.connect(source);
    playable.prepare();
    if (state.offset < playable.buffer.duration) {
      source.start(startAt, state.offset);
    }
    return source;
  });
  const longest = state.sources[
    playables.findIndex((item) => item.buffer.duration === state.duration)];
  const handleEnded = () => {
    if (state.playing && generation === state.generation && playbackTime() >= state.duration - 0.03) {
      if (ui.loop.checked) {
        disposeSources();
        state.offset = 0;
        startPlayback();
      } else {
        stopPlayback(true);
      }
    }
  };
  if (longest) longest.onended = handleEnded;
  const longestStreaming = streaming.find(
    (audio) => Math.abs(audio.duration - state.duration) < 0.01);
  if (longestStreaming) longestStreaming.media.onended = handleEnded;
  state.playing = true;
  ui.playIcon.textContent = "❚❚";
  ui.playPause.setAttribute("aria-label", "Пауза");
}

function pausePlayback() {
  if (!state.playing) return;
  state.offset = playbackTime();
  disposeSources();
  state.playing = false;
  ui.playIcon.textContent = "▶";
  ui.playPause.setAttribute("aria-label", "Воспроизвести");
}

function stopPlayback(reset = true) {
  disposeSources();
  state.playing = false;
  if (reset) state.offset = 0;
  for (const audio of streamingAudioAssets()) {
    if (Math.abs(audio.media.currentTime - state.offset) > 0.01) {
      audio.media.currentTime = Math.min(state.offset, audio.duration);
    }
  }
  ui.playIcon.textContent = "▶";
  ui.playPause.setAttribute("aria-label", "Воспроизвести");
  ui.timeline.value = String(state.offset);
  ui.currentTime.textContent = formatTime(state.offset);
}

function disposeSources() {
  state.generation += 1;
  for (const source of state.sources) {
    source.onended = null;
    try { source.stop(); } catch (_) {}
    source.disconnect();
  }
  state.sources = [];
  for (const audio of streamingAudioAssets()) {
    audio.media.onended = null;
    audio.media.pause();
  }
}

function seek(time) {
  const wasPlaying = state.playing;
  if (wasPlaying) pausePlayback();
  state.offset = Math.max(0, Math.min(state.duration, time));
  if (wasPlaying) startPlayback();
}

function formatTime(value) {
  const safe = Number.isFinite(value) ? Math.max(0, value) : 0;
  const hours = Math.floor(safe / 3600);
  const minutes = Math.floor((safe % 3600) / 60);
  const seconds = Math.floor(safe % 60);
  return `${String(hours).padStart(2, "0")}:${String(minutes).padStart(2, "0")}:${String(seconds).padStart(2, "0")}`;
}

function timelineTickInterval(duration) {
  if (!(duration > 0)) return 1;
  const candidates = [
    1, 2, 5, 10, 15, 30,
    60, 120, 300, 600, 900, 1800,
    3600, 7200, 10800, 21600,
  ];
  const targetMarks = 6;
  let best = candidates[0];
  let bestScore = Infinity;
  for (const interval of candidates) {
    const marks = duration / interval;
    if (marks < 2 || marks > 12) continue;
    const score = Math.abs(marks - targetMarks);
    if (score < bestScore) {
      best = interval;
      bestScore = score;
    }
  }
  if (bestScore === Infinity) {
    return Math.max(1, duration / targetMarks);
  }
  return best;
}

function renderTimelineRuler() {
  ui.timelineRuler.replaceChildren();
  const duration = state.duration;
  if (!(duration > 0)) return;
  const interval = timelineTickInterval(duration);
  const times = [0];
  for (let time = interval; time < duration - interval * 0.35; time += interval) {
    times.push(time);
  }
  if (duration > 0) times.push(duration);

  for (const [index, time] of times.entries()) {
    const tick = document.createElement("div");
    const ratio = time / duration;
    const isStart = index === 0;
    const isEnd = index === times.length - 1;
    tick.className = `tick${isStart ? " edge edge-start" : ""}${isEnd ? " edge edge-end" : ""}`;
    tick.style.left = isEnd ? "" : `${(ratio * 100).toFixed(3)}%`;
    tick.textContent = formatTime(time);
    ui.timelineRuler.append(tick);
  }
}

function objectAppearanceTimes(object) {
  const points = object.coordinates;
  if (!points.length) return [];
  const times = [];
  let active = false;
  for (const point of points) {
    if (point.visualActive) {
      if (!active) {
        times.push(point.time);
        active = true;
      }
    } else {
      active = false;
    }
  }
  return times;
}

function renderTimelineEvents() {
  ui.timelineEvents.replaceChildren();
  const duration = state.duration;
  if (!(duration > 0)) return;
  const fragment = document.createDocumentFragment();
  for (const object of state.objects) {
    for (const time of objectAppearanceTimes(object)) {
      if (time < 0 || time > duration) continue;
      const mark = document.createElement("span");
      mark.className = "mark";
      mark.style.left = `${((time / duration) * 100).toFixed(3)}%`;
      mark.style.background = object.color;
      mark.style.color = object.color;
      mark.title = `${object.name} · ${formatTime(time)}`;
      fragment.append(mark);
    }
  }
  ui.timelineEvents.append(fragment);
}

function renderTimelineDecorations() {
  renderTimelineRuler();
  renderTimelineEvents();
}

function timelinePointAt(object, time) {
  const points = object.coordinates;
  if (!points.length) return null;
  const samplePosition = Math.floor(time * points[0].sampleRate);
  let low = 0;
  let high = points.length - 1;
  while (low < high) {
    const mid = Math.ceil((low + high) / 2);
    if (points[mid].ptsSamples <= samplePosition) low = mid;
    else high = mid - 1;
  }
  const point = points[low];
  if (samplePosition < point.ptsSamples) return null;
  if (point.durationSamples > 0
      && samplePosition >= point.ptsSamples + point.durationSamples) {
    return null;
  }
  return { point, index: low, samplePosition };
}

function objectMetadataGainAt(object, time) {
  if (!state.objectSettings.metadataGainEnabled) return 1;
  return timelinePointAt(object, time)?.point.metadataGainLinear ?? 1;
}

function updateObjectMetadataGain(object, time, immediate = false) {
  if (!state.context) return;
  const gain = objectMetadataGainAt(object, time);
  if (object.lastMetadataGain !== null
      && Math.abs(gain - object.lastMetadataGain) < 1e-7) return;
  object.lastMetadataGain = gain;
  const now = state.context.currentTime;
  object.metadataGain.gain.cancelScheduledValues(now);
  if (immediate) {
    object.metadataGain.gain.setValueAtTime(gain, now);
  } else {
    object.metadataGain.gain.setTargetAtTime(gain, now, 0.008);
  }
}

function positionAt(object, time) {
  const points = object.coordinates;
  if (!points.length) return null;
  const sampleRate = points[0].sampleRate;
  const samplePosition = Math.floor(time * sampleRate);
  const timeline = timelinePointAt(object, time);
  let low = timeline?.index ?? 0;
  if (!timeline && samplePosition >= points[0].ptsSamples) {
    let high = points.length - 1;
    while (low < high) {
      const mid = Math.ceil((low + high) / 2);
      if (points[mid].ptsSamples <= samplePosition) low = mid;
      else high = mid - 1;
    }
  }
  const current = points[low];
  const currentEnd = current.ptsSamples + current.durationSamples;
  if (samplePosition < current.ptsSamples) {
    return null;
  }

  let position = current;
  let visibility = 1;
  if (!current.visualActive) {
    if (current.lastVisibleIndex < 0) return null;
    const fadeSamples = OBJECT_FADE_SECONDS * sampleRate;
    visibility = 1 - (samplePosition - current.fadeStartSamples) / fadeSamples;
    if (visibility <= 0) return null;
    position = points[current.lastVisibleIndex];
  } else if (current.durationSamples > 0 && samplePosition >= currentEnd) {
    const fadeSamples = OBJECT_FADE_SECONDS * sampleRate;
    visibility = 1 - (samplePosition - currentEnd) / fadeSamples;
    if (visibility <= 0) return null;
  }

  if (object.staticCoordinates) position = object.staticPosition;
  const next = points[Math.min(low + 1, points.length - 1)];
  const interpolateToNext =
    !object.staticCoordinates
    && current.visualActive
    && current.interpolation === "linear"
    && next !== current
    && next.visualActive
    && (current.durationSamples <= 0 || next.ptsSamples <= currentEnd);
  const target = interpolateToNext ? next : position;
  const span = target.ptsSamples - position.ptsSamples;
  const mix = span > 0
    ? Math.max(0, Math.min(
        1, (samplePosition - position.ptsSamples) / span))
    : 0;
  let azDelta = target.azimuth - position.azimuth;
  if (azDelta > 180) azDelta -= 360;
  if (azDelta < -180) azDelta += 360;
  const azimuth = position.azimuth + azDelta * mix;
  const elevation = position.elevation + (target.elevation - position.elevation) * mix;
  const distance = position.distance + (target.distance - position.distance) * mix;
  const widthDeg = position.widthDeg + (target.widthDeg - position.widthDeg) * mix;
  const heightDeg = position.heightDeg + (target.heightDeg - position.heightDeg) * mix;
  const rotationDeg = position.rotationDeg
    + (target.rotationDeg - position.rotationDeg) * mix;
  const az = azimuth * Math.PI / 180;
  const el = elevation * Math.PI / 180;
  const radius = Math.max(0, distance);
  return {
    x: Math.sin(az) * Math.cos(el) * radius,
    y: Math.sin(el) * radius,
    z: Math.cos(az) * Math.cos(el) * radius,
    azimuth, elevation, distance, widthDeg, heightDeg, rotationDeg,
    visibility: Math.max(0, Math.min(1, visibility)),
  };
}

function updateStereoPan(object, position) {
  if (!object.panner || !state.context) return;
  if (!position && state.playing && object.hasPanPosition) return;
  object.hasPanPosition = Boolean(position);
  const pan = position
    // Use the Cartesian left/right displacement, not azimuth alone.
    // At the upper/lower pole azimuth is undefined, and an object close
    // to the listener centre must also approach the stereo centre.
    ? Math.max(-1, Math.min(1, position.x))
    : 0;
  if (Math.abs(pan - object.lastPan) < 0.001) return;
  object.lastPan = pan;
  object.panner.pan.setTargetAtTime(
    pan, state.context.currentTime, 0.015);
}

function updateObjectTrail(object, position, time) {
  if (object.staticCoordinates || !state.objectSettings.trailsEnabled) {
    object.trail.length = 0;
    object.lastTrailTime = time;
    object.lastTrailPosition = null;
    return;
  }
  const trailSeconds = state.objectSettings.trailSeconds;
  const discontinuity = object.lastTrailTime !== null
    && (time < object.lastTrailTime || time - object.lastTrailTime > 0.15);
  if (discontinuity) {
    object.trail.length = 0;
    object.lastTrailPosition = null;
  }
  object.trail = object.trail.filter(
    (point) => time - point.time <= trailSeconds);
  if (!position) {
    if (!object.trail.length) object.lastTrailPosition = null;
    object.lastTrailTime = time;
    return;
  }
  const previous = object.lastTrailPosition;
  const movement = previous
    ? Math.hypot(
        position.x - previous.x,
        position.y - previous.y,
        position.z - previous.z)
    : Infinity;
  if (movement > 0.0025) {
    object.trail.push({
      x: position.x,
      y: position.y,
      z: position.z,
      time,
      visibility: position.visibility,
      widthDeg: position.widthDeg,
      heightDeg: position.heightDeg,
    });
    if (object.trail.length > OBJECT_TRAIL_MAX_POINTS) {
      object.trail.splice(
        0, object.trail.length - OBJECT_TRAIL_MAX_POINTS);
    }
    object.lastTrailPosition = {
      x: position.x,
      y: position.y,
      z: position.z,
    };
  }
  object.lastTrailTime = time;
}

function drawObjectTrail(object, rect, time) {
  if (!state.objectSettings.trailsEnabled || object.trail.length < 2) return;
  const audible = state.objectsEnabled && object.enabled;
  const baseAlpha = audible ? 0.26 : 0.07;
  const trailSeconds = state.objectSettings.trailSeconds;
  const head = object.trail[object.trail.length - 1];
  const headAge = Math.max(0, time - head.time);
  const freshness = headAge <= 0.05
    ? 1
    : Math.max(0, 1 - (headAge - 0.05) / trailSeconds);
  ctx.lineCap = "round";
  for (let index = 1; index < object.trail.length; index++) {
    const previous = object.trail[index - 1];
    const current = object.trail[index];
    const age = Math.max(0, time - current.time);
    const taper = index === object.trail.length - 1
      ? 1
      : Math.max(0, 1 - age / trailSeconds);
    const life = taper * freshness;
    if (life <= 0) continue;
    const from = project(previous, rect);
    const to = project(current, rect);
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.strokeStyle = hexAlpha(
      object.color,
      baseAlpha * life * current.visibility);
    ctx.lineWidth = Math.max(
      0.35,
      objectMarkerRadius(object, to, audible, current) * 2 * life);
    ctx.stroke();
  }
  ctx.lineCap = "butt";
  ctx.lineWidth = 1;
}

function projectedDepthScale(projected) {
  return state.objectSettings.depthScaleEnabled
    ? Math.max(0.62, Math.min(1.42, projected.scale))
    : 1;
}

function normalizedMetadataSize(position) {
  if (!state.objectSettings.metadataSizeEnabled || !position) return 1;
  const angularExtent = Math.max(
    0,
    Number.isFinite(position.widthDeg) ? position.widthDeg : 0,
    Number.isFinite(position.heightDeg) ? position.heightDeg : 0);
  // DTS:X extents are angular values in the closed 0..360 degree range.
  // Map that range to a bounded 1x..2x marker scale so a full-sphere
  // extended source remains readable without covering the whole scene.
  return 1 + Math.min(1, angularExtent / 360);
}

function objectMarkerRadius(object, projected, audible, position = null) {
  const objectId = object.coordinates[0]?.objectId ?? "?";
  const idLength = String(objectId).length;
  return Math.max(11, 8 + idLength * 2.6)
    * projectedDepthScale(projected)
    * normalizedMetadataSize(position)
    * (audible ? 1 : 0.8);
}

const ctx = ui.canvas.getContext("2d");
function resizeCanvas() {
  const rect = ui.canvas.getBoundingClientRect();
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (ui.canvas.width !== width || ui.canvas.height !== height) {
    ui.canvas.width = width;
    ui.canvas.height = height;
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return rect;
}

function rotate(point) {
  const cy = Math.cos(state.camera.yaw), sy = Math.sin(state.camera.yaw);
  const cp = Math.cos(state.camera.pitch), sp = Math.sin(state.camera.pitch);
  const x1 = point.x * cy - point.z * sy;
  const z1 = point.x * sy + point.z * cy;
  return { x: x1, y: point.y * cp - z1 * sp, z: point.y * sp + z1 * cp };
}

function project(point, rect) {
  const p = rotate(point);
  const perspective = 3.6 / (4.3 - p.z);
  const scale = Math.min(rect.width, rect.height) * 0.34 * state.camera.zoom;
  return {
    x: rect.width / 2 + p.x * scale * perspective,
    y: rect.height / 2 - p.y * scale * perspective,
    z: p.z,
    scale: perspective,
  };
}

function line3d(a, b, rect, color, width = 1) {
  const pa = project(a, rect), pb = project(b, rect);
  ctx.beginPath();
  ctx.moveTo(pa.x, pa.y);
  ctx.lineTo(pb.x, pb.y);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.stroke();
}

function sphericalPoint(azimuth, elevation, radius = 1) {
  const az = azimuth * Math.PI / 180;
  const el = elevation * Math.PI / 180;
  return {
    x: Math.sin(az) * Math.cos(el) * radius,
    y: Math.sin(el) * radius,
    z: Math.cos(az) * Math.cos(el) * radius,
  };
}

function drawUnitSphere(rect) {
  const sphereColor = "rgba(92, 185, 205, 0.13)";
  const equatorColor = "rgba(92, 185, 205, 0.22)";
  for (const elevation of [-60, -30, 0, 30, 60]) {
    for (let azimuth = -180; azimuth < 180; azimuth += 10) {
      line3d(
        sphericalPoint(azimuth, elevation),
        sphericalPoint(azimuth + 10, elevation),
        rect,
        elevation === 0 ? equatorColor : sphereColor);
    }
  }
  for (let azimuth = -150; azimuth <= 180; azimuth += 30) {
    for (let elevation = -90; elevation < 90; elevation += 10) {
      line3d(
        sphericalPoint(azimuth, elevation),
        sphericalPoint(azimuth, elevation + 10),
        rect,
        sphereColor);
    }
  }
}

function bedSpeakerPosition(channel) {
  const positions = {
    FL: [-30, 0], FR: [30, 0], FC: [0, 0], C: [0, 0],
    BL: [-150, 0], BR: [150, 0], SL: [-90, 0], SR: [90, 0],
    BC: [180, 0], WL: [-60, 0], WR: [60, 0],
    TFL: [-30, 45], TFR: [30, 45],
    TBL: [-150, 45], TBR: [150, 45],
    TFC: [0, 45], TRC: [180, 45], TC: [0, 90],
  };
  const name = String(channel).toUpperCase();
  if (name === "LFE" || name === "LFE1" || name === "LFE2") {
    return sphericalPoint(0, -32, 0.72);
  }
  const angles = positions[name];
  return angles ? sphericalPoint(angles[0], angles[1], 1.06) : null;
}

function drawSpeakerLabel(item, rect) {
  const { channel, center } = item;
  const heightChannel = String(channel.name).toUpperCase().startsWith("T");
  const lfeChannel = String(channel.name).toUpperCase().startsWith("LFE");
  const color = heightChannel
    ? "#b58cff"
    : lfeChannel ? "#ff9f55" : "#48dce5";
  const active = item.bed.enabled && channel.enabled && !channel.silent;
  const label = project(center, rect);
  const depthScale = projectedDepthScale(label);
  const fontSize = Math.max(6, 12 * depthScale);
  ctx.globalAlpha = active
    ? 0.92
    : item.reference ? 0.5 : channel.silent ? 0.2 : 0.38;
  ctx.font = `750 ${fontSize.toFixed(1)}px Segoe UI`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillStyle = "rgba(2,7,10,0.92)";
  ctx.fillText(String(channel.name), label.x + 1, label.y + 1);
  ctx.fillStyle = active
    ? color
    : item.reference ? shadeHex(color, 0.72) : "#89949e";
  ctx.fillText(String(channel.name), label.x, label.y);
  ctx.globalAlpha = 1;
}

function drawBedSpeakers(rect) {
  if (!state.objectSettings.bedSpeakersEnabled) return;
  const speakers = new Map();
  for (const bed of state.beds) {
    for (const channel of bed.channels) {
      const key = String(channel.name).toUpperCase();
      if (!speakers.has(key)) {
        const center = bedSpeakerPosition(key);
        if (center) speakers.set(key, { bed, channel, center });
      }
    }
  }
  for (const name of REFERENCE_SPEAKER_CHANNELS) {
    if (speakers.has(name)) continue;
    const center = bedSpeakerPosition(name);
    if (!center) continue;
    speakers.set(name, {
      bed: { enabled: false },
      channel: { name, enabled: false, silent: false },
      center,
      reference: true,
    });
  }
  const ordered = [...speakers.values()].sort(
    (a, b) => rotate(a.center).z - rotate(b.center).z);
  for (const speaker of ordered) drawSpeakerLabel(speaker, rect);
}

function sofaVertexColor(rgb, shade) {
  const r = Math.round(Math.max(0, Math.min(255, rgb[0] * 255 * shade)));
  const g = Math.round(Math.max(0, Math.min(255, rgb[1] * 255 * shade)));
  const b = Math.round(Math.max(0, Math.min(255, rgb[2] * 255 * shade)));
  return `rgb(${r},${g},${b})`;
}

function drawLoungeSofa(rect) {
  if (!state.objectSettings.sofaEnabled) return;
  const mesh = window.LOUNGE_SOFA_MESH;
  if (!mesh?.vertices?.length || !mesh.triangles?.length) return;

  const world = mesh.vertices.map((vertex) => ({
    x: vertex[0],
    y: vertex[1],
    z: vertex[2],
  }));
  const rotated = world.map(rotate);
  const projected = world.map((vertex) => project(vertex, rect));
  const faces = [];
  for (const triangle of mesh.triangles) {
    const materialIndex = triangle[0];
    const ia = triangle[1];
    const ib = triangle[2];
    const ic = triangle[3];
    const a = rotated[ia];
    const b = rotated[ib];
    const c = rotated[ic];
    if (!a || !b || !c) continue;
    const ux = b.x - a.x;
    const uy = b.y - a.y;
    const uz = b.z - a.z;
    const vx = c.x - a.x;
    const vy = c.y - a.y;
    const vz = c.z - a.z;
    const nx = uy * vz - uz * vy;
    const ny = uz * vx - ux * vz;
    const nz = ux * vy - uy * vx;
    if (nz <= 0) continue;
    const length = Math.hypot(nx, ny, nz) || 1;
    const light = Math.max(0, (nx * 0.25 + ny * 0.75 + nz * 0.55) / length);
    const shade = 0.34 + light * 0.78;
    faces.push({
      materialIndex,
      shade,
      depth: (a.z + b.z + c.z) / 3,
      points: [projected[ia], projected[ib], projected[ic]],
    });
  }
  faces.sort((left, right) => left.depth - right.depth);

  ctx.save();
  ctx.globalAlpha = 0.88;
  ctx.lineJoin = "round";
  for (const face of faces) {
    const material = mesh.materials[face.materialIndex];
    if (!material) continue;
    const [p0, p1, p2] = face.points;
    ctx.beginPath();
    ctx.moveTo(p0.x, p0.y);
    ctx.lineTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.closePath();
    ctx.fillStyle = sofaVertexColor(material.rgb, face.shade);
    ctx.fill();
    ctx.strokeStyle = sofaVertexColor(material.rgb, face.shade * 0.62);
    ctx.lineWidth = 0.7;
    ctx.stroke();
  }
  ctx.restore();
}

function drawScene() {
  const rect = resizeCanvas();
  ctx.clearRect(0, 0, rect.width, rect.height);

  const gridColor = "rgba(126, 157, 177, 0.10)";
  for (let i = -4; i <= 4; i++) {
    line3d({ x: i / 4, y: 0, z: -1 }, { x: i / 4, y: 0, z: 1 }, rect, gridColor);
    line3d({ x: -1, y: 0, z: i / 4 }, { x: 1, y: 0, z: i / 4 }, rect, gridColor);
  }
  line3d({ x: -1.1, y: 0, z: 0 }, { x: 1.1, y: 0, z: 0 }, rect, "rgba(240,107,117,.6)", 1.4);
  line3d({ x: 0, y: -1, z: 0 }, { x: 0, y: 1.1, z: 0 }, rect, "rgba(120,220,152,.6)", 1.4);
  line3d({ x: 0, y: 0, z: -1.1 }, { x: 0, y: 0, z: 1.1 }, rect, "rgba(92,158,255,.6)", 1.4);
  drawUnitSphere(rect);
  drawLoungeSofa(rect);
  drawBedSpeakers(rect);

  const time = playbackTime();
  const positioned = state.objects
    .map((object) => ({ object, position: positionAt(object, time) }));
  for (const item of positioned) {
    updateObjectMetadataGain(item.object, time);
    updateStereoPan(item.object, item.position);
    updateObjectTrail(item.object, item.position, time);
    drawObjectTrail(item.object, rect, time);
  }
  const visible = positioned
    .filter((item) => item.position)
    .map((item) => ({ ...item, projected: project(item.position, rect) }))
    .sort((a, b) => a.projected.z - b.projected.z);

  for (const item of visible) {
    const { object, position, projected } = item;
    const audible = state.objectsEnabled && object.enabled;
    const alpha = (audible ? 1 : 0.25) * position.visibility;
    if (state.objectSettings.guideLinesEnabled) {
      const origin = project({ x: 0, y: 0, z: 0 }, rect);
      ctx.beginPath();
      ctx.moveTo(origin.x, origin.y);
      ctx.lineTo(projected.x, projected.y);
      ctx.strokeStyle = hexAlpha(
        object.color,
        (audible ? 0.2 : 0.08) * position.visibility);
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 5]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    const objectId = object.coordinates[0]?.objectId ?? "?";
    const idText = String(objectId);
    const depthScale = projectedDepthScale(projected);
    const radius = objectMarkerRadius(object, projected, audible, position);
    const glow = ctx.createRadialGradient(projected.x, projected.y, 0, projected.x, projected.y, radius * 3.4);
    glow.addColorStop(0, hexAlpha(object.color, 0.34 * alpha));
    glow.addColorStop(1, hexAlpha(object.color, 0));
    ctx.fillStyle = glow;
    ctx.beginPath();
    ctx.arc(projected.x, projected.y, radius * 3.4, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = alpha;
    const surface = ctx.createRadialGradient(
      projected.x - radius * 0.32,
      projected.y - radius * 0.38,
      radius * 0.08,
      projected.x,
      projected.y,
      radius * 1.18);
    surface.addColorStop(0, shadeHex(object.color, 1.28));
    surface.addColorStop(0.52, object.color);
    surface.addColorStop(1, shadeHex(object.color, 0.52));
    ctx.fillStyle = surface;
    ctx.beginPath();
    ctx.arc(projected.x, projected.y, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#071015";
    ctx.font = `700 ${Math.max(7, 9 * depthScale).toFixed(1)}px Segoe UI`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(idText, projected.x, projected.y + 0.5);
    ctx.globalAlpha = alpha;
    ctx.fillStyle = "#dfe8ec";
    ctx.font = "10px Segoe UI";
    ctx.textAlign = "left";
    ctx.fillText(
      `${object.name}  ${position.azimuth.toFixed(1)}° / ${position.elevation.toFixed(1)}°`,
      projected.x + radius + 7, projected.y - 1);
    ctx.globalAlpha = 1;
  }

  if (state.playing) {
    ui.timeline.value = String(time);
    ui.currentTime.textContent = formatTime(time);
  }
  requestAnimationFrame(drawScene);
}

function hexAlpha(hex, alpha) {
  const value = hex.replace("#", "");
  const number = parseInt(value, 16);
  return `rgba(${number >> 16},${(number >> 8) & 255},${number & 255},${alpha})`;
}

function shadeHex(hex, factor) {
  const value = parseInt(hex.replace("#", ""), 16);
  const channel = (shift) => Math.max(
    0,
    Math.min(255, Math.round(((value >> shift) & 255) * factor)));
  return `rgb(${channel(16)},${channel(8)},${channel(0)})`;
}

ui.openFolder.addEventListener("click", async () => {
  try {
    const handle = await window.showDirectoryPicker({ mode: "read" });
    const files = [];
    for await (const entry of handle.values()) if (entry.kind === "file") files.push(await entry.getFile());
    await loadFiles(files, handle.name);
  } catch (error) {
    if (error.name !== "AbortError") showError(error.message || String(error));
  }
});
ui.folderFallback.addEventListener("change", async () => {
  try {
    const files = [...ui.folderFallback.files];
    const label = files[0]?.webkitRelativePath?.split("/")[0] || "Выбранная папка";
    await loadFiles(files, label);
  } catch (error) {
    showError(error.message || String(error));
  } finally {
    ui.folderFallback.value = "";
  }
});
ui.playPause.addEventListener("click", async () => {
  try {
    if (state.playing) pausePlayback();
    else await startPlayback();
  } catch (error) {
    stopPlayback(false);
    showError(error.message || String(error));
  }
});
ui.stop.addEventListener("click", () => stopPlayback(true));
ui.timeline.addEventListener("pointerdown", () => {
  state.resumeAfterSeek = state.playing;
  if (state.playing) pausePlayback();
});
ui.timeline.addEventListener("input", () => {
  state.offset = Number(ui.timeline.value);
  ui.currentTime.textContent = formatTime(state.offset);
});
ui.timeline.addEventListener("change", () => {
  state.offset = Number(ui.timeline.value);
  if (state.resumeAfterSeek) startPlayback();
  state.resumeAfterSeek = false;
});
ui.masterVolume.addEventListener("input", () => {
  const value = Number(ui.masterVolume.value);
  ui.masterVolumeValue.textContent = `${Math.round(value * 100)}%`;
  if (state.masterGain) state.masterGain.gain.setTargetAtTime(value, state.context.currentTime, 0.01);
});
ui.objectsMaster.addEventListener("click", () => {
  const enabled = !state.objectsEnabled;
  state.objectsEnabled = enabled;
  const now = state.context.currentTime;
  for (const object of state.objects) {
    object.enabled = enabled;
    object.gain.gain.setTargetAtTime(
      enabled ? 1 : 0,
      now,
      0.008);
  }
  renderObjectList();
  renderObjectsMaster();
});
ui.objectTrails.addEventListener("change", () => {
  state.objectSettings.trailsEnabled = ui.objectTrails.checked;
  ui.objectTrailLength.disabled = !ui.objectTrails.checked;
  if (!ui.objectTrails.checked) {
    for (const object of state.objects) {
      object.trail.length = 0;
      object.lastTrailPosition = null;
    }
  }
});
ui.objectTrailLength.addEventListener("input", () => {
  const value = Number(ui.objectTrailLength.value);
  state.objectSettings.trailSeconds = value;
  ui.objectTrailLengthValue.textContent = `${value.toFixed(2)} с`;
});
ui.objectDepthScale.addEventListener("change", () => {
  state.objectSettings.depthScaleEnabled = ui.objectDepthScale.checked;
});
ui.objectMetadataSize.addEventListener("change", () => {
  state.objectSettings.metadataSizeEnabled = ui.objectMetadataSize.checked;
});
ui.objectMetadataGain.addEventListener("change", () => {
  state.objectSettings.metadataGainEnabled = ui.objectMetadataGain.checked;
  const time = playbackTime();
  for (const object of state.objects) {
    object.lastMetadataGain = null;
    updateObjectMetadataGain(object, time, true);
  }
});
ui.objectGuideLines.addEventListener("change", () => {
  state.objectSettings.guideLinesEnabled = ui.objectGuideLines.checked;
});
ui.bedSpeakers.addEventListener("change", () => {
  state.objectSettings.bedSpeakersEnabled = ui.bedSpeakers.checked;
});
ui.centerSofa.addEventListener("change", () => {
  state.objectSettings.sofaEnabled = ui.centerSofa.checked;
});

let drag = null;
ui.canvas.addEventListener("pointerdown", (event) => {
  drag = { x: event.clientX, y: event.clientY, yaw: state.camera.yaw, pitch: state.camera.pitch };
  ui.canvas.setPointerCapture(event.pointerId);
  ui.canvas.classList.add("dragging");
});
ui.canvas.addEventListener("pointermove", (event) => {
  if (!drag) return;
  state.camera.yaw = drag.yaw + (event.clientX - drag.x) * 0.008;
  state.camera.pitch = Math.max(-1.35, Math.min(1.35, drag.pitch + (event.clientY - drag.y) * 0.008));
});
ui.canvas.addEventListener("pointerup", () => {
  drag = null;
  ui.canvas.classList.remove("dragging");
});
ui.canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  state.camera.zoom = Math.max(0.48, Math.min(2.5, state.camera.zoom * Math.exp(-event.deltaY * 0.001)));
}, { passive: false });
ui.canvas.addEventListener("dblclick", () => {
  state.camera = { yaw: -0.68, pitch: -0.28, zoom: 1 };
});
window.addEventListener("keydown", (event) => {
  if (event.code === "Space" && event.target.tagName !== "INPUT" && !ui.playPause.disabled) {
    event.preventDefault();
    if (state.playing) pausePlayback();
    else startPlayback().catch((error) => {
      stopPlayback(false);
      showError(error.message || String(error));
    });
  }
});

drawScene();
