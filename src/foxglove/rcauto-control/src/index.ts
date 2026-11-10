import type { ExtensionContext } from "@foxglove/extension";

import { initCameraPanel } from "./CameraPanel";
import { initControlPanel } from "./ControlPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "control", initPanel: initControlPanel });
  extensionContext.registerPanel({ name: "camera", initPanel: initCameraPanel });
}