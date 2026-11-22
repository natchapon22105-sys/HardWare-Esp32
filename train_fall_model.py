import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np
import pandas as pd


CHANNELS = ("ax", "ay", "az", "gx", "gy", "gz")


def load_resample(path, target_hz):
    frame = pd.read_csv(path)
    required = {"timestamp", *CHANNELS}
    missing = required.difference(frame.columns)
    if missing:
        raise ValueError(f"{path}: missing columns {sorted(missing)}")

    timestamp = frame["timestamp"].to_numpy(dtype=np.float64)
    if np.min(np.diff(timestamp)) <= 0:
        raise ValueError(f"{path}: timestamp must be strictly increasing")

    scale = 1000.0
    if np.max(timestamp) - np.min(timestamp) <= 1000:
        scale = 1.0
    time_s = (timestamp - timestamp[0]) / scale

    duration = time_s[-1]
    new_time = np.arange(0.0, duration + 1.0 / target_hz, 1.0 / target_hz)
    values = frame[list(CHANNELS)].to_numpy(dtype=np.float32)
    resampled = np.column_stack(
        [np.interp(new_time, time_s, values[:, i]) for i in range(values.shape[1])]
    )
    return resampled


def make_windows(samples, window, step):
    windows = []
    for start in range(0, len(samples) - window + 1, step):
        windows.append(samples[start:start + window])
    return windows


def load_dataset(data_dir, window_sec, step_sec, target_hz):
    fall_files = sorted(glob.glob(os.path.join(data_dir, "fall*.csv")))
    nofall_files = sorted(glob.glob(os.path.join(data_dir, "nofall*.csv")))
    if not fall_files or not nofall_files:
        raise RuntimeError("Need at least one fall*.csv and one nofall*.csv file")

    window = round(window_sec * target_hz)
    step = round(step_sec * target_hz)
    subject_windows = []

    for label, paths in ((1, fall_files), (0, nofall_files)):
        for path in paths:
            samples = load_resample(path, target_hz)
            windows = make_windows(samples, window, step)
            if not windows:
                print(f"skip {Path(path).name}: shorter than {window_sec:.1f} seconds")
                continue
            subject_windows.append({
                "subject": Path(path).stem,
                "label": label,
                "windows": np.stack(windows),
            })
            print(f"{Path(path).name}: {len(windows)} windows")

    return subject_windows, window


def subject_split(subject_windows, test_fraction):
    fall = [item for item in subject_windows if item["label"] == 1]
    nofall = [item for item in subject_windows if item["label"] == 0]
    if len(fall) < 2 or len(nofall) < 2:
        raise RuntimeError(
            f"Need at least 2 subject files per class for a subject-independent split; "
            f"found {len(fall)} fall and {len(nofall)} nofall."
        )

    test_fall = max(1, round(len(fall) * test_fraction))
    test_nofall = max(1, round(len(nofall) * test_fraction))

    test_subjects = fall[-test_fall:] + nofall[-test_nofall:]
    test_ids = {item["subject"] for item in test_subjects}
    train = [item for item in subject_windows if item["subject"] not in test_ids]
    test = [item for item in subject_windows if item["subject"] in test_ids]
    if not train or not test:
        raise RuntimeError("Subject split produced an empty train or test set")
    return train, test


def stack_subjects(items):
    x = np.concatenate([item["windows"] for item in items], axis=0)
    y = np.concatenate([
        np.full(len(item["windows"]), item["label"], dtype=np.float32)
        for item in items
    ])
    return x, y


def normalize_fit(x):
    mean = x.reshape(-1, x.shape[-1]).mean(axis=0)
    std = x.reshape(-1, x.shape[-1]).std(axis=0)
    std[std < 1e-6] = 1.0
    return mean.astype(np.float32), std.astype(np.float32)


def augment(x):
    rng = np.random.default_rng(12345)
    output = x.copy()
    output += rng.normal(0.0, 0.015, output.shape).astype(np.float32)
    scale = rng.uniform(0.95, 1.05, size=(len(output), 1, 1)).astype(np.float32)
    output *= scale
    if len(output) > 1:
        for i in range(len(output)):
            if rng.random() < 0.2:
                output[i] = np.roll(output[i], rng.integers(-5, 6), axis=0)
    return output


def build_model(window, target_hz):
    try:
        import tensorflow as tf
    except ImportError as exc:
        raise RuntimeError(
            "TensorFlow is required. Create a Python 3.10-3.12 environment and "
            "install: pip install -r requirements-fall.txt"
        ) from exc

    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=(window, 6)),
        tf.keras.layers.Conv1D(16, 7, activation="relu", padding="same"),
        tf.keras.layers.BatchNormalization(),
        tf.keras.layers.MaxPool1D(2),
        tf.keras.layers.Conv1D(24, 5, activation="relu", padding="same"),
        tf.keras.layers.BatchNormalization(),
        tf.keras.layers.MaxPool1D(2),
        tf.keras.layers.Conv1D(32, 3, activation="relu", padding="same"),
        tf.keras.layers.GlobalAveragePooling1D(),
        tf.keras.layers.Dense(16, activation="relu"),
        tf.keras.layers.Dropout(0.25),
        tf.keras.layers.Dense(1, activation="sigmoid"),
    ])
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3),
        loss="binary_crossentropy",
        metrics=[
            tf.keras.metrics.BinaryAccuracy(name="accuracy"),
            tf.keras.metrics.AUC(name="auc"),
            tf.keras.metrics.Precision(name="precision"),
            tf.keras.metrics.Recall(name="recall"),
        ],
    )
    return model


def representative_dataset(x):
    for i in range(0, len(x), max(1, len(x) // 100)):
        sample = x[i:i + 1].astype(np.float32)
        yield [sample]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", default=".")
    parser.add_argument("--window-sec", type=float, default=2.0)
    parser.add_argument("--step-sec", type=float, default=0.5)
    parser.add_argument("--target-hz", type=int, default=100)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--test-fraction", type=float, default=0.25)
    parser.add_argument("--output-dir", default="fall_model")
    args = parser.parse_args()

    subject_windows, window = load_dataset(
        args.data_dir, args.window_sec, args.step_sec, args.target_hz
    )
    train_items, test_items = subject_split(subject_windows, args.test_fraction)

    x_train, y_train = stack_subjects(train_items)
    x_test, y_test = stack_subjects(test_items)
    mean, std = normalize_fit(x_train)
    x_train = (x_train - mean) / std
    x_test = (x_test - mean) / std
    x_train = augment(x_train)

    import tensorflow as tf
    tf.keras.utils.set_random_seed(12345)

    model = build_model(window, args.target_hz)
    counts = np.bincount(y_train.astype(np.int32), minlength=2)
    class_weights = {
        0: float(len(y_train) / max(1, 2 * counts[0])),
        1: float(len(y_train) / max(1, 2 * counts[1])),
    }

    os.makedirs(args.output_dir, exist_ok=True)
    callbacks = [
        tf.keras.callbacks.EarlyStopping(
            monitor="val_auc", mode="max", patience=8, restore_best_weights=True
        ),
        tf.keras.callbacks.ModelCheckpoint(
            os.path.join(args.output_dir, "best.keras"),
            monitor="val_auc", mode="max", save_best_only=True
        ),
    ]
    model.fit(
        x_train, y_train,
        validation_split=0.25,
        epochs=args.epochs,
        batch_size=64,
        class_weight=class_weights,
        callbacks=callbacks,
        verbose=2,
    )

    probabilities = model.predict(x_test, verbose=0).reshape(-1)
    threshold = 0.5
    from sklearn.metrics import confusion_matrix, classification_report, average_precision_score
    predictions = (probabilities >= threshold).astype(np.int32)
    print("\nTest report")
    print(confusion_matrix(y_test, predictions))
    print(classification_report(y_test, predictions, target_names=["nofall", "fall"]))
    print("Average precision:", average_precision_score(y_test, probabilities))

    model.save(os.path.join(args.output_dir, "fall_model.keras"))
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = lambda: representative_dataset(
        np.concatenate([x_train, x_test], axis=0)
    )
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.float32
    tflite_model = converter.convert()
    with open(os.path.join(args.output_dir, "fall_model_int8.tflite"), "wb") as output:
        output.write(tflite_model)

    info = {
        "window_samples": window,
        "channels": list(CHANNELS),
        "target_hz": args.target_hz,
        "mean": mean.tolist(),
        "std": std.tolist(),
        "threshold": threshold,
        "input_dtype": "int8",
        "output_dtype": "float32",
    }
    with open(os.path.join(args.output_dir, "fall_model_info.json"), "w") as output:
        json.dump(info, output, indent=2)

    np.savez(
        os.path.join(args.output_dir, "evaluation.npz"),
        y_true=y_test,
        y_probability=probabilities,
        mean=mean,
        std=std,
    )


if __name__ == "__main__":
    main()
