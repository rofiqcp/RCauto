import type { PanelExtensionContext } from "@foxglove/extension";
import {
  readCameraStreamQuality,
  saveCameraStreamQuality,
  type CameraStreamQuality,
} from "./cameraStreamQuality";

type Tuning = {
  key: string;
  title: string;
  min: number;
  max: number;
  step: number;
};

const definitions: readonly Tuning[] = [
  {
    key: "/rcauto_esp32_hotplug.max_motor_output_percent",
    title: "PWM maksimum ESP32 (%)",
    min: 0,
    max: 100,
    step: 1,
  },
  {
    key: "/rcauto_esp32_hotplug.max_steering_output_percent",
    title: "Sensitivitas kemudi ESP32 (%)",
    min: 0,
    max: 100,
    step: 1,
  },
  {
    key: "/rcauto_cpp_controller.controller.pid.kp",
    title: "Steering Kp",
    min: 0,
    max: 4,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_controller.controller.pid.ki",
    title: "Steering Ki",
    min: 0,
    max: 2,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_controller.controller.pid.kd",
    title: "Steering Kd",
    min: 0,
    max: 2,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_controller.controller.speed_pid.kp",
    title: "Visual speed Kp",
    min: 0,
    max: 5,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_controller.controller.speed_pid.ki",
    title: "Visual speed Ki",
    min: 0,
    max: 2,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_controller.controller.speed_pid.kd",
    title: "Visual speed Kd",
    min: 0,
    max: 2,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_controller.controller.fuzzy.error_weight",
    title: "Fuzzy error weight",
    min: 0,
    max: 2,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_controller.controller.fuzzy.rate_weight",
    title: "Fuzzy rate weight",
    min: 0,
    max: 2,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_controller.controller.fuzzy.speed_aggressiveness",
    title: "Fuzzy speed gain",
    min: 0.5,
    max: 2,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_controller.controller.ai.output_gain",
    title: "AI output gain",
    min: 0.5,
    max: 2,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_controller.drive.follow_speed_mps",
    title: "Follow speed (m/s)",
    min: 0.01,
    max: 0.5,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_controller.drive.max_speed_mps",
    title: "Max speed (m/s)",
    min: 0.01,
    max: 0.5,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_controller.drive.corner_speed_scale",
    title: "Corner speed factor",
    min: 0.4,
    max: 1,
    step: 0.02,
  },
  {
    key: "/rcauto_cpp_controller.drive.cone_speed_scale",
    title: "Cone avoidance speed factor",
    min: 0.2,
    max: 1,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_controller.drive.max_steering_rad",
    title: "Maximum steering angle (rad; 0.35 ≈ 20°)",
    min: 0.1,
    max: 0.55,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_controller.drive.steering_scale",
    title: "Steering scale / sensitivity",
    min: 0.2,
    max: 1.5,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_controller.drive.steering_deadband_norm",
    title: "Steering deadband",
    min: 0,
    max: 0.15,
    step: 0.005,
  },
  {
    key: "/rcauto_cpp_controller.drive.steering_slew_rad_s",
    title: "Steering slew (rad/s)",
    min: 0.1,
    max: 8,
    step: 0.1,
  },
  {
    key: "/rcauto_cpp_controller.drive.accel_mps2",
    title: "Acceleration (m/s²)",
    min: 0.05,
    max: 3,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_controller.drive.decel_mps2",
    title: "Deceleration (m/s²)",
    min: 0.1,
    max: 5,
    step: 0.1,
  },
  {
    key: "/rcauto_cpp_vision.vision.target_distance_ratio",
    title: "Target distance: fraction of visible road (farther →)",
    min: 0.15,
    max: 0.85,
    step: 0.05,
  },
  {
    key: "/rcauto_cpp_vision.vision.lookahead_weight",
    title: "Camera lookahead",
    min: 0,
    max: 1,
    step: 0.02,
  },
  {
    key: "/rcauto_cpp_vision.vision.error_smoothing_alpha",
    title: "Error smoothing",
    min: 0.05,
    max: 1,
    step: 0.01,
  },
  {
    key: "/rcauto_cpp_vision.vision.min_saturation",
    title: "Line saturation",
    min: 0,
    max: 255,
    step: 1,
  },
  {
    key: "/rcauto_cpp_vision.vision.min_brightness",
    title: "Line brightness",
    min: 0,
    max: 255,
    step: 1,
  },
  {
    key: "/rcauto_cpp_vision.vision.cone_avoidance_offset_ratio",
    title: "Cone offset: center → lane border (%)",
    min: 0.1,
    max: 0.85,
    step: 0.05,
  },
];

const parameterHelp: Record<string, readonly [string, string, string]> = {
  "PWM maksimum ESP32 (%)": [
    "Batas persentase PWM motor BTS7960 yang dikeluarkan bridge ROS. Nilai 0 berarti selalu tidak ada tenaga motor.",
    "Naik: motor dapat menerima perintah PWM yang lebih besar (hanya ketika mode hardware aktif dan START).",
    "Turun: batas PWM motor mengecil; nilai 0 menonaktifkan perintah tenaga motor.",
  ],
  "Sensitivitas kemudi ESP32 (%)": [
    "Pengali keluaran kemudi yang dikirim melalui USB ke servo ESP32; tidak mengubah parameter PID atau kalibrasi mekanis.",
    "Naik: servo dapat menerima perintah belok yang lebih besar dari kontroler kamera.",
    "Turun: belokan servo lebih lembut; nilai 0 berarti perintah steering netral.",
  ],
  "Steering Kp": [
    "Penguatan proporsional pada error posisi jalur. Mengoreksi arah berdasarkan selisih target dengan posisi robot.",
    "Naik: kemudi lebih cepat dan agresif mengikuti garis; terlalu tinggi dapat membuat goyangan kiri-kanan.",
    "Turun: respons kemudi lebih halus, tetapi terlambat mengikuti tikungan.",
  ],
  "Steering Ki": [
    "Koreksi akumulasi error agar penyimpangan kecil yang menetap dapat dihilangkan.",
    "Naik: lebih cepat mengoreksi offset tetap; terlalu tinggi berisiko overshoot dan osilasi.",
    "Turun: lebih stabil terhadap perubahan mendadak, tetapi bias posisi dapat bertahan.",
  ],
  "Steering Kd": [
    "Meredam perubahan error yang cepat melalui komponen derivatif kemudi.",
    "Naik: meredam overshoot pada tikungan, tetapi terlalu tinggi sensitif terhadap noise kamera.",
    "Turun: kemudi lebih reaktif namun berpotensi berayun.",
  ],
  "Visual speed Kp": [
    "Penguatan kontrol kecepatan berdasarkan risiko visual, bukan sensor kecepatan roda.",
    "Naik: respons pengaturan kecepatan terhadap perubahan kondisi jalur lebih tegas.",
    "Turun: kecepatan berubah lebih pelan saat jalur mulai berbelok.",
  ],
  "Visual speed Ki": [
    "Akumulasi error kontrol kecepatan visual untuk menangani deviasi berkepanjangan.",
    "Naik: koreksi jangka panjang lebih besar; terlalu tinggi dapat membuat respons lambat pulih.",
    "Turun: mengurangi akumulasi respons, tetapi error menetap mungkin tersisa.",
  ],
  "Visual speed Kd": [
    "Menentukan seberapa kuat kontrol kecepatan merespons perubahan risiko visual.",
    "Naik: antisipasi perubahan risiko lebih cepat, namun sensitif noise deteksi.",
    "Turun: kecepatan lebih stabil tetapi pengereman antisipatif bisa terlambat.",
  ],
  "Fuzzy error weight": [
    "Bobot error posisi pada kendali fuzzy untuk menentukan besar koreksi kemudi.",
    "Naik: pengaruh posisi garis pada kemudi lebih dominan, sehingga lebih agresif.",
    "Turun: kemudi lebih lembut tetapi koreksi posisi bisa kurang kuat.",
  ],
  "Fuzzy rate weight": [
    "Bobot laju perubahan error pada kendali fuzzy untuk antisipasi perubahan arah.",
    "Naik: respons terhadap tikungan mendatang lebih kuat; terlalu tinggi dapat berosilasi.",
    "Turun: mengurangi sensitivitas terhadap perubahan mendadak, namun terlambat berbelok.",
  ],
  "Fuzzy speed gain": [
    "Tingkat agresivitas kendali fuzzy dalam menyesuaikan kecepatan berdasarkan risiko kamera.",
    "Naik: respons keluaran kecepatan fuzzy lebih agresif terhadap kondisi jalur.",
    "Turun: respons kecepatan lebih konservatif dan lembut.",
  ],
  "AI output gain": [
    "Skala keluaran steering dari model ONNX AI; tidak mengubah bobot pelatihan model.",
    "Naik: belokan dari AI lebih tajam, tetapi lebih mudah overshoot.",
    "Turun: belokan AI lebih lembut, namun dapat gagal mengikuti tikungan tajam.",
  ],
  "Follow speed (m/s)": [
    "Kecepatan nominal saat mengikuti jalur; dibatasi oleh Max speed dan faktor risiko kamera.",
    "Naik: target maju pada lintasan normal lebih cepat.",
    "Turun: target maju lebih pelan, memberi waktu respons lebih panjang.",
  ],
  "Max speed (m/s)": [
    "Batas atas kecepatan yang dapat diminta controller; bukan pengukuran kecepatan nyata.",
    "Naik: batas kecepatan lebih tinggi, butuh pengereman lebih awal.",
    "Turun: membatasi kecepatan puncak robot.",
  ],
  "Corner speed factor": [
    "Faktor batas kecepatan saat melewati tikungan dan ketika kamera mengindikasikan risiko.",
    "Naik: robot cenderung mempertahankan kecepatan saat menikung.",
    "Turun: robot lebih lambat pada tikungan untuk menjaga lintasan.",
  ],
  "Cone avoidance speed factor": [
    "Pengali kecepatan saat manuver menghindari cone.",
    "Naik: robot dapat melintasi rintangan lebih cepat; ruang koreksi kemudi berkurang.",
    "Turun: robot melambat lebih banyak ketika menghindari cone.",
  ],
  "Maximum steering angle (rad; 0.35 ≈ 20°)": [
    "Batas sudut kemudi dalam radian (0,35 rad kira-kira 20 derajat).",
    "Naik: memungkinkan belokan lebih tajam dalam batas mekanik servo.",
    "Turun: membatasi ketajaman belokan dan mencegah manuver berlebihan.",
  ],
  "Steering scale / sensitivity": [
    "Pengali sensitivitas perintah kemudi sesudah kontrol PID/Fuzzy/AI.",
    "Naik: perubahan error menghasilkan sudut kemudi yang lebih besar.",
    "Turun: kemudi lebih tenang dan stabil untuk lintasan lurus.",
  ],
  "Steering deadband": [
    "Zona error kecil yang diperlakukan sebagai lurus untuk mengurangi gerakan servo yang tidak perlu.",
    "Naik: lebih sedikit koreksi kecil; risiko melewatkan belokan ringan.",
    "Turun: koreksi kecil lebih sering; berisiko servo bergerak bolak-balik.",
  ],
  "Steering slew (rad/s)": [
    "Batas seberapa cepat sudut kemudi boleh berubah tiap detik.",
    "Naik: kemudi dapat berbelok lebih cepat, tetapi gerakan menjadi mendadak.",
    "Turun: belokan lebih halus, tetapi bisa lambat pada tikungan tajam.",
  ],
  "Acceleration (m/s²)": [
    "Batas kenaikan kecepatan per detik kuadrat saat target kecepatan meningkat.",
    "Naik: akselerasi lebih cepat namun lebih agresif.",
    "Turun: start dan kenaikan kecepatan lebih halus.",
  ],
  "Deceleration (m/s²)": [
    "Batas laju pengurangan kecepatan terkontrol; STOP tetap memerintahkan kecepatan nol.",
    "Naik: perlambatan lebih cepat, berpotensi lebih mendadak.",
    "Turun: perlambatan normal lebih halus namun memerlukan jarak lebih jauh.",
  ],
  "Target distance: fraction of visible road (farther →)": [
    "Posisi target longitudinal di citra sebagai fraksi jarak jalan terlihat dari robot.",
    "Naik: titik target terlihat lebih jauh, membantu antisipasi tikungan.",
    "Turun: titik target lebih dekat, cenderung lebih akurat dekat robot tetapi lebih reaktif.",
  ],
  "Camera lookahead": [
    "Bobot proyeksi jalan ke depan saat membentuk target kemudi dari kamera.",
    "Naik: respons terhadap bentuk jalan jauh lebih dominan.",
    "Turun: kemudi lebih menekankan posisi jalan dekat robot.",
  ],
  "Error smoothing": [
    "Koefisien penghalusan error kamera; nilai besar memberi bobot lebih kuat pada pengamatan terbaru.",
    "Naik: error lebih responsif tetapi lebih sensitif noise gambar.",
    "Turun: error lebih halus tetapi lebih lambat mengikuti perubahan.",
  ],
  "Line saturation": [
    "Ambang saturasi warna HSV minimum untuk mengenali garis pada kamera.",
    "Naik: lebih selektif terhadap warna pekat, risiko garis pudar tidak terdeteksi.",
    "Turun: warna pudar lebih mudah terdeteksi, tetapi noise warna meningkat.",
  ],
  "Line brightness": [
    "Ambang kecerahan minimum piksel untuk dianggap sebagai garis.",
    "Naik: hanya bagian terang yang diterima; garis gelap bisa hilang.",
    "Turun: garis dalam cahaya redup lebih mudah terdeteksi; noise dapat meningkat.",
  ],
  "Cone offset: center → lane border (%)": [
    "Besaran pergeseran titik target lateral ketika menghindari cone, relatif terhadap lebar jalur.",
    "Naik: posisi target menjauh lebih jauh dari cone, tetapi mendekati batas lajur.",
    "Turun: perubahan jalur lebih kecil, tetapi jarak bebas dari cone berkurang.",
  ],
};

function add<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  parent: HTMLElement,
  text?: string,
): HTMLElementTagNameMap[K] {
  const result = document.createElement(tag);
  if (text !== undefined) result.textContent = text;
  parent.appendChild(result);
  return result;
}

export function initControlPanel(context: PanelExtensionContext): () => void {
  const root = context.panelElement;
  root.replaceChildren();
  root.style.cssText =
    "height:100%;box-sizing:border-box;overflow:auto;background:#151b25;color:#e8edf7;font:13px system-ui,sans-serif;padding:14px";
  const heading = add("div", root, "RCAUTO / CAMERA + CONE AVOIDANCE");
  heading.style.cssText = "font-weight:750;color:#84c7ff;font-size:15px;letter-spacing:.04em";
  const status = add("div", root, "Connecting to ROS2 controller...");
  status.style.cssText =
    "margin:10px 0;padding:10px;background:#202b39;border-left:4px solid #d99527;border-radius:7px";
  const camera = add("div", root, "USB camera: searching for device...");
  camera.style.cssText = "font-size:11px;color:#fbbf70;white-space:pre-wrap;margin-bottom:3px";
  const motor = add("div", root, "USB motor interface: checking...");
  motor.style.cssText = "font-size:11px;color:#bdcddd;white-space:pre-wrap";
  const detection = add("div", root, "Cone avoidance: waiting...");
  detection.style.cssText = "font-size:11px;color:#d4dce6;margin-top:4px";
  const feedback = add("div", root, "");
  feedback.style.cssText = "font-size:12px;color:#fbbf70;min-height:19px;margin:7px 0";
  const saveStatus = add("div", root, "YAML: waiting for confirmed change");
  saveStatus.style.cssText = "font-size:11px;color:#91e2b4;min-height:18px;margin:4px 0";
  const qualityLabel = add("label", root, "Camera stream · this computer");
  qualityLabel.style.cssText = "display:block;font-size:12px;color:#b9cde0;margin:9px 0 7px";
  const quality = add("select", qualityLabel);
  quality.style.cssText =
    "display:block;width:100%;margin-top:4px;padding:7px;background:#24344b;color:#fff;border:1px solid #465a70;border-radius:6px";
  for (const [value, title] of [
    ["auto", "Auto · full resolution, adapt bandwidth"],
    ["normal", "Normal · full resolution, 15 FPS"],
    ["eco", "Hemat · full resolution, 15 FPS"],
  ] as const) {
    const option = add("option", quality, title);
    option.value = value;
  }
  quality.value = readCameraStreamQuality();
  quality.onchange = () => {
    const choice = quality.value as CameraStreamQuality;
    const stored = saveCameraStreamQuality(choice);
    feedback.textContent = stored
      ? "Camera quality for this computer: " + choice
      : "Camera preference storage unavailable";
  };
  const buttons = add("div", root);
  buttons.style.cssText = "display:grid;grid-template-columns:1fr;gap:7px";
  const button = (label: string, background: string) => {
    const b = add("button", buttons, label);
    b.style.cssText =
      "padding:11px 3px;border:0;border-radius:7px;color:white;font-weight:750;cursor:pointer;background:" +
      background;
    return b;
  };
  const toggle = button("START", "#16734c");
  toggle.style.cssText += ";width:100%;margin:6px 0 10px;min-height:42px";
  const modeLabel = add("label", root, "Mode kontroler");
  modeLabel.style.cssText = "display:block;margin:6px 0";
  const mode = add("select", modeLabel);
  mode.style.cssText =
    "display:block;width:100%;background:#24344b;color:#fff;padding:8px;margin-top:5px;border:1px solid #465a70;border-radius:6px";
  for (const [value, label] of [
    ["pid", "PID steering + visual-speed PID"],
    ["fuzzy", "Fuzzy steering + fuzzy speed"],
    ["ai", "AI dual-output (requires ONNX model)"],
  ] as const) {
    const choice = add("option", mode, label);
    choice.value = value;
  }
  const note = add("div", root, "Pengaturan kontroler · dapat diubah saat START maupun STOP");
  note.style.cssText = "font-size:13px;color:#d4e5f4;font-weight:700;margin-top:16px";
  const explanation = add(
    "div",
    root,
    "Slider diterapkan langsung pada ROS 2 dan perubahan yang diterima otomatis disimpan ke robot.yaml. Klik ⓘ untuk membaca fungsi serta dampak menaikkan atau menurunkan nilainya.",
  );
  explanation.style.cssText = "font-size:11px;color:#91a6bb;margin:4px 0 7px";
  const tuning = add("div", root);
  const sharedHeading = add("div", root, "Pengaturan umum kemudi, kamera, kecepatan, dan cone");
  sharedHeading.style.cssText = "font-size:13px;color:#d4e5f4;font-weight:700;margin-top:17px";
  const common = add("div", root);
  const espSettings = document.createElement("div");
  const groupFor = (key: string): "pid" | "fuzzy" | "ai" | "esp" | null => {
    if (key.startsWith("/rcauto_esp32_hotplug.")) return "esp";
    if (key.includes(".controller.pid.") || key.includes(".controller.speed_pid.")) return "pid";
    if (key.includes(".controller.fuzzy.")) return "fuzzy";
    if (key.includes(".controller.ai.")) return "ai";
    return null;
  };
  const rows = new Map<
    string,
    {
      wrapper: HTMLDivElement;
      slider: HTMLInputElement;
      output: HTMLOutputElement;
      config: Tuning;
    }
  >();
  const known = new Set<string>();
  const pending = new Map<string, { value: number; time: number; label: string }>();

  for (const config of definitions) {
    const family = groupFor(config.key);
    const label = add("div", family === "esp" ? espSettings : family ? tuning : common);
    label.style.cssText = "display:block;padding:7px 0;border-bottom:1px solid #2d3b4a";
    add("div", label, config.title).style.cssText = "color:#ccd7e4;margin-bottom:5px";
    const line = add("div", label);
    line.style.cssText = "display:flex;align-items:center;gap:8px";
    const slider = add("input", line);
    slider.type = "range";
    slider.min = String(config.min);
    slider.max = String(config.max);
    slider.step = String(config.step);
    slider.value = String(config.min);
    slider.style.cssText = "flex:1;accent-color:#4daeff;min-width:0";
    const output = add("output", line, "—");
    output.style.cssText = "font:12px monospace;min-width:49px;text-align:right";
    const help = parameterHelp[config.title];
    if (!help) throw new Error("Petunjuk parameter tidak tersedia: " + config.title);
    const details = add("details", label);
    details.style.cssText =
      "margin-top:6px;padding:5px 8px;border-radius:5px;background:#1d2836;font-size:11px;color:#b9cee3;line-height:1.6";
    const summary = add("summary", details, "ⓘ Petunjuk: fungsi, nilai naik dan turun");
    summary.style.cssText = "cursor:pointer;color:#86c9ff;user-select:none";
    add("div", details, "Fungsi: " + help[0]);
    add("div", details, "▲ Jika dinaikkan: " + help[1]);
    add("div", details, "▼ Jika diturunkan: " + help[2]);
    rows.set(config.key, { wrapper: label, slider, output, config });
    slider.oninput = () => {
      output.textContent = slider.value;
    };
    slider.onchange = () => {
      if (slider.disabled) return;
      const value = config.step === 1 ? Math.round(Number(slider.value)) : Number(slider.value);
      feedback.textContent = "Sending " + config.title + " = " + String(value);
      try {
        context.setParameter(config.key, value);
        pending.set(config.key, { value, time: Date.now(), label: config.title });
      } catch (error) {
        feedback.textContent = "Parameter rejected: " + String(error);
      }
    };
  }
  const hardware = add("div", root, "ESP32 / BTS7960");
  hardware.style.cssText = "font-size:13px;color:#d5e2ed;font-weight:700;margin:17px 0 5px";
  const hardwareHelp = add(
    "div",
    root,
    "BTS7960: GPIO14 (maju) / GPIO27 (mundur), PWM 20 kHz 10-bit. Servo: GPIO13, pulsa netral 1500 µs. Tombol: GPIO12, tekan cepat START, tahan 2 detik STOP. Kamera dan ESP32 USB mendukung hotplug; sambungan ulang tidak pernah otomatis mengaktifkan motor.",
  );
  hardwareHelp.style.cssText = "font-size:11px;color:#aabfce;line-height:1.5";
  const espRuntime = add("div", root, "ESP32: menunggu telemetri firmware...");
  espRuntime.style.cssText =
    "margin:6px 0;padding:7px 9px;background:#1c2a39;border-radius:6px;color:#bfdbef;font-size:11px;white-space:pre-wrap;line-height:1.6";
  const espMode = add("div", root, "Mode hardware: menunggu parameter bridge...");
  espMode.style.cssText = "font-size:11px;color:#9cb8d0;margin:5px 0";
  root.appendChild(espSettings);

  let ready = false,
    armed = false,
    currentMode = "pid";
  let modeChangePending: { value: string; started: number } | undefined;
  let commandPending: { expected: boolean; started: number } | undefined;
  let lastHeartbeat = 0;
  let avoidanceActive = false,
    avoidanceBlocked = false,
    avoidanceOffset = 0;

  const publish = (topic: string, data: boolean | string) => {
    try {
      context.publish?.(topic, { data });
      feedback.textContent = topic + " ← " + String(data);
    } catch (error) {
      feedback.textContent = "Command failed: " + String(error);
    }
  };
  const render = () => {
    toggle.disabled = !ready || commandPending !== undefined;
    toggle.textContent = armed ? "STOP" : "START";
    toggle.style.background = armed ? "#b45309" : "#16734c";
    toggle.title = armed
      ? "STOP segera melalui Foxglove. Tombol fisik ESP32: tahan 2 detik untuk STOP."
      : "START segera; tombol fisik ESP32: tekan singkat sekali untuk START.";
    mode.disabled = !ready;
    if (document.activeElement !== mode || modeChangePending === undefined) {
      mode.value = currentMode;
    }
    for (const [key, { wrapper, slider }] of rows) {
      const family = groupFor(key);
      wrapper.style.display =
        family === null || family === "esp" || family === currentMode ? "block" : "none";
      slider.disabled = !ready || !known.has(key);
    }
    sharedHeading.style.display = "block";
    detection.textContent =
      "Cone avoidance: " +
      (avoidanceBlocked ? "BLOCKED · STOP" : avoidanceActive ? "ACTIVE" : "INACTIVE") +
      " · lateral offset " +
      avoidanceOffset.toFixed(3);
    status.style.borderLeftColor = armed ? "#24c487" : "#d99527";
  };
  toggle.onclick = () => {
    if (!ready || commandPending) return;
    commandPending = { expected: !armed, started: Date.now() };
    publish("/rcauto/operator/enable", !armed);
    render();
  };
  mode.onchange = () => {
    const requestedMode = mode.value;
    modeChangePending = { value: requestedMode, started: Date.now() };
    currentMode = requestedMode;
    render();
    publish("/rcauto/operator/mode", requestedMode);
  };
  const topics = [
    "/rcauto/status",
    "/rcauto/control/mode",
    "/rcauto/esp32/status",
    "/rcauto/firmware/status",
    "/rcauto/firmware/armed",
    "/rcauto/camera/status",
    "/rcauto/vision/avoidance_active",
    "/rcauto/vision/avoidance_blocked",
    "/rcauto/vision/avoidance_offset_norm",
    "/rcauto/tuning/persistence_status",
  ];
  context.subscribe(topics.map((topic) => ({ topic })));
  context.watch("currentFrame");
  context.watch("parameters");
  if (context.advertise) {
    context.advertise("/rcauto/operator/enable", "std_msgs/msg/Bool");
    context.advertise("/rcauto/operator/mode", "std_msgs/msg/String");
  }
  context.onRender = (state, done) => {
    try {
      if (state.parameters) {
        const hardwareEnabled = state.parameters.get("/rcauto_esp32_hotplug.hardware_enabled");
        if (typeof hardwareEnabled === "boolean") {
          espMode.textContent = hardwareEnabled
            ? "Mode hardware AKTIF — output motor mengikuti START/STOP dan batas PWM"
            : "Mode PREVIEW — perintah PWM motor selalu nol, START hanya mengaktifkan kontrol kamera";
          espMode.style.color = hardwareEnabled ? "#eac681" : "#91e2b4";
        }
        for (const [key, control] of rows) {
          const value = state.parameters.get(key);
          if (typeof value !== "number" || !Number.isFinite(value)) continue;
          known.add(key);
          const waiting = pending.get(key);
          if (waiting && Math.abs(value - waiting.value) < 0.0001) {
            pending.delete(key);
            feedback.textContent = "ROS accepted " + waiting.label + " = " + String(value);
          } else if (waiting && Date.now() - waiting.time > 3500) {
            pending.delete(key);
            feedback.textContent = "ROS menolak " + waiting.label + "; periksa batas parameter.";
          }
          if (document.activeElement !== control.slider) {
            control.slider.value = String(value);
            control.output.textContent = value.toFixed(control.config.step >= 1 ? 0 : 3);
          }
        }
      }
      for (const message of state.currentFrame ?? []) {
        const data = (message.message as { data?: unknown }).data;
        switch (message.topic) {
          case "/rcauto/status":
            if (typeof data === "string") {
              ready = true;
              status.textContent = data;
              armed = data.includes("armed=1");
              if (
                armed &&
                commandPending?.expected !== false &&
                Date.now() - lastHeartbeat >= 250
              ) {
                try {
                  context.publish?.("/rcauto/operator/enable", { data: true });
                  lastHeartbeat = Date.now();
                } catch (error) {
                  feedback.textContent = "Koneksi kontrol gagal: " + String(error);
                }
              }
              if (commandPending && armed === commandPending.expected) {
                commandPending = undefined;
                feedback.textContent = armed
                  ? "START diterima controller"
                  : "STOP diterima controller";
              } else if (commandPending && Date.now() - commandPending.started > 2500) {
                commandPending = undefined;
                feedback.textContent =
                  "Perintah belum dikonfirmasi oleh controller; periksa status koneksi.";
              }
            }
            break;
          case "/rcauto/control/mode":
            if (typeof data === "string") {
              const pendingMode = modeChangePending;
              if (pendingMode && pendingMode.value === data) {
                modeChangePending = undefined;
                currentMode = data;
              } else if (!pendingMode || Date.now() - pendingMode.started > 2000) {
                modeChangePending = undefined;
                currentMode = data;
              }
            }
            break;
          case "/rcauto/esp32/status":
            if (typeof data === "string") {
              motor.textContent = data;
              motor.style.color = data.startsWith("ESP32 CONNECTED") ? "#91e2b4" : "#fbbf70";
            }
            break;
          case "/rcauto/firmware/status":
            if (typeof data === "string" && data.startsWith("STATUS,")) {
              const fields = new Map(
                data
                  .slice(7)
                  .split(",")
                  .filter((part) => part.includes("="))
                  .map((part) => {
                    const separator = part.indexOf("=");
                    return [part.slice(0, separator), part.slice(separator + 1)];
                  }),
              );
              espRuntime.textContent =
                "Firmware: " +
                (fields.get("armed") === "1" ? "ARMED" : "DISARMED") +
                " | Steering: " +
                (fields.get("steer") ?? "—") +
                " | Motor aktual: " +
                (fields.get("speed") ?? "—") +
                " | Target: " +
                (fields.get("target") ?? "—") +
                "\nDuty BTS7960 maju: " +
                (fields.get("forward_duty") ?? "—") +
                "/1023 | mundur: " +
                (fields.get("reverse_duty") ?? "—") +
                "/1023";
            }
            break;
          case "/rcauto/firmware/armed":
            break;
          case "/rcauto/camera/status":
            if (typeof data === "string") {
              camera.textContent = "USB camera: " + data;
              camera.style.color = data.startsWith("CONNECTED") ? "#91e2b4" : "#fbbf70";
            }
            break;
          case "/rcauto/vision/avoidance_active":
            if (typeof data === "boolean") avoidanceActive = data;
            break;
          case "/rcauto/vision/avoidance_blocked":
            if (typeof data === "boolean") avoidanceBlocked = data;
            break;
          case "/rcauto/vision/avoidance_offset_norm":
            if (typeof data === "number") avoidanceOffset = data;
            break;
          case "/rcauto/tuning/persistence_status":
            if (typeof data === "string") {
              saveStatus.textContent = data;
              saveStatus.style.color = data.includes("FAILED") ? "#fca5a5" : "#91e2b4";
            }
            break;
        }
      }
      render();
    } finally {
      done();
    }
  };
  render();
  return () => {
    context.onRender = undefined;
    context.subscribe([]);
    for (const topic of ["/rcauto/operator/enable", "/rcauto/operator/mode"])
      context.unadvertise?.(topic);
    root.replaceChildren();
  };
}
