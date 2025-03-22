#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import csv
import numpy as np
import librosa

import tensorflow as tf
import tensorflow_hub as hub

# Подставьте ваш реальный тип сообщения
from audio_utils_msgs.msg import AudioFrame

def load_yamnet_labels(path_csv: str):
    labels = []
    with open(path_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            labels.append(row["display_name"])
    return labels

class OdasClassifierNode(Node):
    def __init__(self):
        super().__init__('odas_classifier_node')

        # Подписка на AudioFrame
        self.subscription = self.create_subscription(
            AudioFrame,
            '/sss',
            self.audio_callback,
            10
        )

        # Загружаем YAMNet-модель
        self.get_logger().info("Loading YAMNet...")
        self.yamnet_model = hub.load("https://tfhub.dev/google/yamnet/1")
        self.get_logger().info("YAMNet loaded.")

        # Определяем путь к CSV с метками внутри share/odas_classifier/
        pkg_share_dir = get_package_share_directory('odas_classifier')
        csv_path = f"{pkg_share_dir}/yamnet_class_map_with_db.csv"
        self.labels = load_yamnet_labels(csv_path)
        self.get_logger().info(f"Loaded {len(self.labels)} YAMNet labels.")

    def audio_callback(self, msg: AudioFrame):
        """
        Колбэк вызывается на каждое поступившее аудиосообщение.
        Обрабатываем каждый канал по отдельности как «источник».
        """

        sr = msg.sampling_frequency
        channels = msg.channel_count
        fmt = msg.format
        sample_count = msg.frame_sample_count

        # Выбираем dtype на основе msg.format
        if fmt == "float":
            dtype = np.float32
        elif fmt == "double":
            dtype = np.float64
        elif fmt == "signed_16":
            dtype = np.int16
        elif fmt == "signed_32":
            dtype = np.int32
        else:
            self.get_logger().error(f"Unsupported audio format: {fmt}")
            return

        audio_array = np.frombuffer(msg.data, dtype=dtype)
        expected_size = sample_count * channels
        if audio_array.size != expected_size:
            self.get_logger().error(
                f"Data size mismatch: got {audio_array.size}, expected {expected_size}"
            )
            return

        audio_array = audio_array.reshape(sample_count, channels)

        # Обработка каждого канала
        for ch in range(channels):
            audio_channel = audio_array[:, ch]

            # Ресемплим при необходимости
            if sr != 16000:
                audio_channel = librosa.resample(
                    audio_channel, orig_sr=sr, target_sr=16000
                )
                sr_out = 16000
            else:
                sr_out = sr

            audio_16k = audio_channel.astype(np.float32)

            # Запуск модели
            scores, embeddings, log_mel = self.yamnet_model(audio_16k)
            mean_scores = tf.reduce_mean(scores, axis=0)
            predicted_class_idx = tf.math.argmax(mean_scores).numpy()
            confidence = mean_scores[predicted_class_idx].numpy()

            if predicted_class_idx < len(self.labels):
                predicted_label_str = self.labels[predicted_class_idx]
            else:
                predicted_label_str = f"Class_{predicted_class_idx}"

            ignored_classes = {"silence", "sound effect"}

            if predicted_label_str.lower() in ignored_classes:
                return

            self.get_logger().info(
                f"Channel={ch}, SR={sr_out} => Label='{predicted_label_str}', "
                f"Confidence={confidence:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = OdasClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
