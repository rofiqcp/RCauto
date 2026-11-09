"""Train ONNX camera-preview policies for steering and throttle control.

Inputs: near/far camera lane error, visual curvature, error derivative, camera
risk. Outputs: normalized steering and speed factor. Supports synthetic
baseline or supervised imitation of simulated Fuzzy-driving telemetry.
These models are simulation candidates, not validated real-vehicle policies.
"""

import argparse
import csv
from pathlib import Path

import numpy as np


def synthetic_targets(features):
    near, far, curvature, rate = (features[:, index] for index in range(4))
    steering = np.clip(0.70 * near + 0.40 * far +
                       0.10 * curvature + 0.030 * rate, -0.90, 0.90)
    risk = features[:, 4]
    throttle = np.clip(1.0 - 0.75 * risk - 0.09 * np.abs(rate),
                       0.25, 1.0)
    return np.stack((steering, throttle), axis=-1).astype(np.float32)


def train(seed=23, iterations=2100):
    rng = np.random.default_rng(seed)
    data = rng.uniform(-1, 1, (10000, 5)).astype(np.float32)
    data[:, 1] = np.clip(data[:, 0] + rng.normal(0, 0.34, len(data)), -1, 1)
    data[:, 4] = np.clip(np.maximum.reduce((1.25 * np.abs(data[:, 0]),
                                    0.90 * np.abs(data[:, 1]),
                                    0.78 * np.abs(data[:, 2]),
                                    0.24 * np.ones(len(data), dtype=np.float32))), 0, 1)
    target = synthetic_targets(data)
    w1 = rng.normal(0, 0.45, (5, 32)).astype(np.float32)
    b1 = np.zeros(32, dtype=np.float32)
    w2 = rng.normal(0, 0.08, (32, 2)).astype(np.float32)
    b2 = np.zeros(2, dtype=np.float32)
    for _ in range(iterations):
        index = rng.integers(0, len(data), 256)
        sample, truth = data[index], target[index]
        hidden = np.tanh(sample @ w1 + b1)
        prediction = hidden @ w2 + b2
        gradient = 2 * (prediction - truth) / len(sample)
        delta_w2 = hidden.T @ gradient
        delta_b2 = gradient.sum(axis=0)
        gradient_hidden = (gradient @ w2.T) * (1 - hidden**2)
        delta_w1 = sample.T @ gradient_hidden
        delta_b1 = gradient_hidden.sum(axis=0)
        learning_rate = 0.024
        w2 -= learning_rate * delta_w2
        b2 -= learning_rate * delta_b2
        w1 -= learning_rate * delta_w1
        b1 -= learning_rate * delta_b1
    predicted = np.tanh(data @ w1 + b1) @ w2 + b2
    mae = np.mean(np.abs(predicted - target), axis=0)
    return (w1, b1, w2, b2), mae



def load_expert_samples(paths):
    """Learn from real simulated camera-controller traces, never odom inputs."""
    features, labels, times = [], [], []
    for path in paths:
        previous = None
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                try:
                    t = float(row["time_s"])
                    near = float(row["near_error"])
                    far = float(row["far_error"])
                    curve = float(row["curvature"])
                    risk = float(row["visual_risk"])
                    conf = float(row["lane_confidence"])
                    speed = float(row["speed_cmd_mps"])
                    offset = float(row["centerline_offset_eval_only"])
                    steering = float(row["steering_normalized"])
                    throttle = float(row["speed_factor"])
                except (KeyError, ValueError):
                    continue
                fused = 0.3 * near + 0.7 * far
                rate = 0.0
                if previous is not None:
                    dt = t - previous[0]
                    if 0.0 < dt < 0.25:
                        rate = float(np.clip((fused - previous[1])/dt/3, -1, 1))
                previous = (t, fused)
                x = [near, far, curve, rate, risk]
                if (row["status"] != "FOLLOW" or conf < 0.55 or
                        speed < 0.04 or offset > 0.15 or
                        not np.isfinite(x + [steering, throttle]).all() or
                        not -1.01 <= steering <= 1.01 or
                        not 0.24 <= throttle <= 1.01):
                    continue
                features.append(x)
                labels.append([steering, throttle])
                times.append(t)
    if len(features) < 400:
        raise ValueError("At least 400 reliable camera observations required")
    return (np.asarray(features, np.float32),
            np.asarray(labels, np.float32),
            np.asarray(times, np.float32))


def train_expert(paths, iterations=3000, seed=47):
    data, targets, times = load_expert_samples(paths)
    # Separate every fifth four-second block from training, to limit leakage.
    heldout = (np.floor(times/4).astype(np.int32) % 5) == 0
    if heldout.sum() < 50 or (~heldout).sum() < 300:
        raise ValueError("Insufficient holdout data")
    x, y = data[~heldout], targets[~heldout]
    valid_x, valid_y = data[heldout], targets[heldout]
    rng = np.random.default_rng(seed)
    w1 = rng.normal(0, 0.33, (5, 32)).astype(np.float32)
    b1 = np.zeros(32, np.float32)
    w2 = rng.normal(0, 0.08, (32, 2)).astype(np.float32)
    b2 = np.zeros(2, np.float32)
    for _ in range(iterations):
        idx = rng.integers(0, len(x), 192)
        batch, labels = x[idx], y[idx]
        h = np.tanh(batch @ w1 + b1)
        pred = h @ w2 + b2
        grad = 2 * (pred-labels) / len(batch)
        gw2, gb2 = h.T @ grad, grad.sum(axis=0)
        delta = (grad @ w2.T) * (1-h*h)
        gw1, gb1 = batch.T @ delta, delta.sum(axis=0)
        w2 -= .026*gw2
        b2 -= .026*gb2
        w1 -= .026*gw1
        b1 -= .026*gb1
    predictions = np.tanh(valid_x @ w1 + b1) @ w2 + b2
    mae = np.abs(predictions-valid_y).mean(axis=0)
    print("Samples",len(data),"holdout",len(valid_x),
          "MAE steering/throttle",mae)
    if not np.isfinite(mae).all() or mae[0] > .13 or mae[1] > .14:
        raise RuntimeError(f"Expert imitation is too inaccurate: {mae}")
    return w1, b1, w2, b2

def save_onnx(path, weights, *, expert=False):
    import onnx
    import onnxruntime as ort
    from onnx import TensorProto, helper, numpy_helper

    w1, b1, w2, b2 = weights
    nodes = [
        helper.make_node("MatMul", ["input", "w1"], ["mm1"]),
        helper.make_node("Add", ["mm1", "b1"], ["h0"]),
        helper.make_node("Tanh", ["h0"], ["h1"]),
        helper.make_node("MatMul", ["h1", "w2"], ["mm2"]),
        helper.make_node("Add", ["mm2", "b2"], ["control"]),
    ]
    graph = helper.make_graph(
        nodes, "expert_imitation_visual_dual_output" if expert else "synthetic_visual_dual_output",
        [helper.make_tensor_value_info("input", TensorProto.FLOAT, [None, 5])],
        [helper.make_tensor_value_info("control", TensorProto.FLOAT, [None, 2])],
        initializer=[numpy_helper.from_array(array, name)
                     for array, name in ((w1, "w1"), (b1, "b1"),
                                         (w2, "w2"), (b2, "b2"))],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)],
                              producer_name="rcauto-expert-imitation" if expert else "rcauto-visual-dual-synthetic")
    model.ir_version = 8
    onnx.checker.check_model(model)
    path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, path)
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    sample = np.array([[0,0,0,0,.24], [.05,.80,.70,.25,.744],
                       [-.4,-.7,-.8,0,.696]], dtype=np.float32)
    output = session.run(None, {"input": sample})[0]
    if (output.shape != (3, 2) or not np.isfinite(output).all() or
            not (0.18 <= output[:, 1]).all() or
            not (output[:, 1] <= 1.10).all() or
            not (np.abs(output[:, 0]) <= 1.10).all() or
            not output[0, 1] > output[1, 1]):
        raise RuntimeError(f"Unsafe ONNX output: {output}")
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--iterations", type=int)
    parser.add_argument("--expert-csv", nargs="+", type=Path)
    arguments = parser.parse_args()
    if arguments.expert_csv:
        weights = train_expert(arguments.expert_csv,
                               iterations=arguments.iterations or 3000)
    else:
        weights, mae = train(iterations=arguments.iterations or 2100)
        print("Synthetic training MAE steering/throttle", mae)
    checks = save_onnx(arguments.output, weights,
                       expert=bool(arguments.expert_csv))
    print("Straight and curve output checks:", checks)
    print("Model:", arguments.output)
