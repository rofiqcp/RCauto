import type { PanelExtensionContext } from "@foxglove/extension";
import { cameraQualityChangedEvent, readCameraStreamQuality } from "./cameraStreamQuality";

const normalTopic = "/rcauto/camera/annotated/compressed";
const bandwidthTopic = "/rcauto/camera/preview/compressed";

type CompressedImageMessage = {
  format?: string;
  data?: Uint8Array | readonly number[] | ArrayBuffer;
};

export function initCameraPanel(context: PanelExtensionContext): () => void {
  const root = context.panelElement;
  root.style.cssText =
    "height:100%;width:100%;display:flex;align-items:center;justify-content:center;position:relative;background:#080f18;overflow:hidden";

  const canvas = document.createElement("canvas");
  canvas.setAttribute("aria-label", "RCAUTO live camera with lane detection overlay");
  canvas.style.cssText = "width:100%;height:100%;object-fit:contain;display:block";
  const bitmapRenderer = canvas.getContext("bitmaprenderer");
  const drawContext = bitmapRenderer == null ? canvas.getContext("2d") : null;

  const status = document.createElement("div");
  status.textContent = "Connecting to live camera…";
  status.style.cssText =
    "position:absolute;color:#b7cadc;font:14px sans-serif;text-align:center;pointer-events:none;padding:16px";
  const stats = document.createElement("div");
  stats.style.cssText =
    "position:absolute;bottom:8px;left:8px;color:#c4e8f5;background:#08111bb8;padding:4px 7px;border-radius:4px;font:11px sans-serif;pointer-events:none";
  root.append(canvas, status, stats);

  let disposed = false;
  let decoding = false;
  let latest: Blob | undefined;
  let displayedFrames = 0;
  let lastSampleFrame = 0;
  let lastSampleAt = performance.now();
  let samples = 0;
  let activeTopic = readCameraStreamQuality() === "eco" ? bandwidthTopic : normalTopic;

  // JPEG decoding happens asynchronously on Chromium's image-decoding threads.
  // A single pending slot prevents stale video frames from accumulating.
  const presentLatest = (): void => {
    if (disposed || decoding || latest == undefined) return;
    const next = latest;
    latest = undefined;
    decoding = true;
    void createImageBitmap(next)
      .then((bitmap) => {
        if (disposed) {
          bitmap.close();
          return;
        }
        if (canvas.width !== bitmap.width) canvas.width = bitmap.width;
        if (canvas.height !== bitmap.height) canvas.height = bitmap.height;
        if (bitmapRenderer != null) {
          bitmapRenderer.transferFromImageBitmap(bitmap);
        } else {
          drawContext?.drawImage(bitmap, 0, 0);
          bitmap.close();
        }
        displayedFrames += 1;
        status.style.display = "none";
      })
      .catch(() => {
        if (disposed) return;
        status.textContent = "Camera frame decode failed";
        status.style.display = "block";
      })
      .finally(() => {
        decoding = false;
        presentLatest();
      });
  };

  const subscribe = (topic: string): void => {
    if (topic === activeTopic) return;
    activeTopic = topic;
    latest = undefined;
    context.subscribe([{ topic }]);
  };

  const updateQuality = (): void => {
    if (disposed) return;
    samples = 0;
    lastSampleAt = performance.now();
    lastSampleFrame = displayedFrames;
    subscribe(readCameraStreamQuality() === "eco" ? bandwidthTopic : normalTopic);
  };

  context.subscribe([{ topic: activeTopic }]);
  window.addEventListener(cameraQualityChangedEvent, updateQuality);
  window.addEventListener("storage", updateQuality);

  const adaptiveCheck = window.setInterval(() => {
    if (disposed) return;
    const now = performance.now();
    if (document.visibilityState !== "visible") {
      lastSampleFrame = displayedFrames;
      lastSampleAt = now;
      samples = 0;
      return;
    }
    const seconds = (now - lastSampleAt) / 1000;
    if (seconds < 2.8) return;
    const fps = (displayedFrames - lastSampleFrame) / seconds;
    lastSampleFrame = displayedFrames;
    lastSampleAt = now;
    stats.textContent = canvas.width + " × " + canvas.height + " · " + fps.toFixed(1) + " FPS";
    if (readCameraStreamQuality() === "auto" && activeTopic === normalTopic) {
      samples += 1;
      if (samples >= 3 && fps < 12) subscribe(bandwidthTopic);
    }
  }, 3000);

  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    try {
      const frames = renderState.currentFrame;
      if (frames != undefined) {
        // The newest decoded ROS image wins; never queue an old frame.
        for (let i = frames.length - 1; i >= 0; --i) {
          const event = frames[i];
          if (event?.topic !== activeTopic) continue;
          const message = event.message as CompressedImageMessage;
          const bytes = message.data;
          if (
            !(bytes instanceof Uint8Array) &&
            !Array.isArray(bytes) &&
            !(bytes instanceof ArrayBuffer)
          ) {
            status.textContent = "Unsupported camera encoding";
            status.style.display = "block";
            break;
          }
          const source =
            bytes instanceof Uint8Array
              ? bytes
              : bytes instanceof ArrayBuffer
                ? new Uint8Array(bytes)
                : new Uint8Array(bytes as number[]);
          const stableBytes = new Uint8Array(source);
          const mime = (message.format ?? "").toLowerCase().includes("png")
            ? "image/png"
            : "image/jpeg";
          latest = new Blob([stableBytes.buffer as ArrayBuffer], { type: mime });
          break;
        }
        presentLatest();
      }
    } finally {
      done();
    }
  };

  return () => {
    disposed = true;
    context.onRender = undefined;
    context.subscribe([]);
    window.clearInterval(adaptiveCheck);
    window.removeEventListener(cameraQualityChangedEvent, updateQuality);
    window.removeEventListener("storage", updateQuality);
    latest = undefined;
    root.replaceChildren();
  };
}
