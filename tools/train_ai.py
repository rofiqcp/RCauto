"""Train a synthetic two-output ONNX preview controller for simulation.

Inputs (camera-only): near lateral error, far lateral error, visual curvature,
error derivative, and camera risk. Outputs: normalized steering and speed factor [0.25, 1].
This is a synthetic imitation baseline, not a validated on-road AI policy.
"""

import argparse
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


def save_onnx(path, weights):
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
        nodes, "synthetic_visual_dual_output",
        [helper.make_tensor_value_info("input", TensorProto.FLOAT, [None, 5])],
        [helper.make_tensor_value_info("control", TensorProto.FLOAT, [None, 2])],
        initializer=[numpy_helper.from_array(array, name)
                     for array, name in ((w1, "w1"), (b1, "b1"),
                                         (w2, "w2"), (b2, "b2"))],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)],
                              producer_name="rcauto-visual-dual-synthetic")
    model.ir_version = 8
    onnx.checker.check_model(model)
    path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, path)
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    sample = np.array([[0,0,0,0,.24], [.05,.80,.70,.25,.744],
                       [-.4,-.7,-.8,0,.696]], dtype=np.float32)
    output = session.run(None, {"input": sample})[0]
    if (output.shape != (3, 2) or not np.isfinite(output).all() or
            not (0.25 <= output[:, 1]).all() or
            not (output[:, 1] <= 1.01).all() or
            not output[0, 1] > output[1, 1] or
            not output[0, 1] > .75):
        raise RuntimeError(f"Unsafe synthesized ONNX output: {output}")
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--iterations", type=int, default=2100)
    arguments = parser.parse_args()
    weights, mae = train(iterations=arguments.iterations)
    checks = save_onnx(arguments.output, weights)
    print("Synthetic dual-output training MAE steering/speed:", mae)
    print("Straight/curve example outputs:", checks)
    print("Model:", arguments.output)
