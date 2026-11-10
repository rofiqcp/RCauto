export type CameraStreamQuality = "auto" | "normal" | "eco";

export const cameraQualityChangedEvent = "rcauto:camera-quality-changed";
const key = "rcauto.camera.stream-quality";

export function readCameraStreamQuality(): CameraStreamQuality {
  try {
    const value = window.localStorage.getItem(key);
    if (value === "normal" || value === "eco") {
      return value;
    }
  } catch {
    // Electron may deny storage in a restricted window; auto remains usable.
  }
  return "auto";
}

export function saveCameraStreamQuality(value: CameraStreamQuality): boolean {
  try {
    window.localStorage.setItem(key, value);
    window.dispatchEvent(new Event(cameraQualityChangedEvent));
    return true;
  } catch {
    return false;
  }
}
