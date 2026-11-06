"""Train an ONNX neural steering baseline on SYNTHETIC PD targets.

Research baseline only. Does not represent a learned policy from real vehicle footage.
Real-world deployment requires actual track data, validation and safety case.
"""

from pathlib import Path
import argparse
import numpy as np


def train(seed=23, iterations=1800):
    rng = np.random.default_rng(seed)
    x = rng.uniform(-1, 1, size=(5000, 2)).astype(np.float32)
    y = np.clip(0.68 * x[:, :1] + 0.22 * x[:, 1:], -0.85, 0.85)
    w1 = rng.normal(0, 0.5, (2, 16)).astype(np.float32)
    b1 = np.zeros(16, dtype=np.float32)
    w2 = rng.normal(0, 0.08, (16, 1)).astype(np.float32)
    b2 = np.zeros(1, dtype=np.float32)
    lr = 0.03
    for _ in range(iterations):
        ix = rng.integers(0, len(x), 256)
        xb, yb = x[ix], y[ix]
        hidden = np.tanh(xb @ w1 + b1)
        pred = hidden @ w2 + b2
        grad = 2 * (pred - yb) / len(xb)
        dw2 = hidden.T @ grad
        db2 = grad.sum(axis=0)
        dz = (grad @ w2.T) * (1 - hidden ** 2)
        dw1 = xb.T @ dz
        db1 = dz.sum(axis=0)
        w2 -= lr * dw2
        b2 -= lr * db2
        w1 -= lr * dw1
        b1 -= lr * db1
    err = float(np.mean(np.abs(np.tanh(x @ w1 + b1) @ w2 + b2 - y)))
    return (w1, b1, w2, b2), err


def save_onnx(output, weights):
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    w1, b1, w2, b2 = weights
    nodes = [
        helper.make_node("Gemm", ["input", "w1", "b1"], ["h0"]),
        helper.make_node("Tanh", ["h0"], ["h1"]),
        helper.make_node("Gemm", ["h1", "w2", "b2"], ["steering"]),
    ]
    graph = helper.make_graph(
        nodes, "synthetic_steering_baseline",
        [helper.make_tensor_value_info("input", TensorProto.FLOAT, [None, 2])],
        [helper.make_tensor_value_info("steering", TensorProto.FLOAT, [None, 1])],
        initializer=[
            numpy_helper.from_array(w1, "w1"),
            numpy_helper.from_array(b1, "b1"),
            numpy_helper.from_array(w2, "w2"),
            numpy_helper.from_array(b2, "b2"),
        ],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)],
                              producer_name="rcauto-synthetic-pd-distillation")
    model.ir_version = 8
    onnx.checker.check_model(model)
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output))
    import onnxruntime as ort
    session = ort.InferenceSession(str(output), providers=["CPUExecutionProvider"])
    actual = session.run(None, {"input": np.array([[0.3, 0.0]], dtype=np.float32)})[0]
    assert 0 < float(actual[0, 0]) < 0.85, "Invalid steering prediction"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    weights, mae = train()
    save_onnx(args.output, weights)
    print(f"Neural baseline trained on synthetic data: MAE={mae:.4f}, model={args.output}")
