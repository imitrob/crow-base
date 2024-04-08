import copy
import os.path as osp
from typing import Tuple

import crow_detector.abstract_detector as abstract_detector
import crow_utils.crow_config
import numpy as np
from crow_detector.lib.crow_vision_yolact.inference_tool import InfTool
import cv2


class YolactDetector(abstract_detector.AbstractDetector):

    def __init__(self, config, config_file, model_file, logger=None):
        super().__init__()

        self.config = config
        self.logger = logger
        config = crow_utils.crow_config.get_config_file(config_file)
        model = crow_utils.crow_config.get_config_file(model_file)

        if self.logger:
            self.logger(f"Using config '{config}'")
            self.logger(f"Using weights from file '{model}'.")

        assert osp.exists(model), f"Provided path to model weights does not exist! {model}"

        self.cnn = InfTool(
            weights=model,
            top_k=self.config["top_k"],
            score_threshold=self.config["threshold"],
            config=config,
        )

    def infer(self, image: np.array) -> Tuple[int, object]:
        preds, frame = self.cnn.process_batch(image)

        (
            classes,
            class_names,
            scores,
            bboxes,
            masks,
            centroids,
        ) = self.cnn.raw_inference(image, preds, frame)

        return len(preds), (image, preds, frame, (classes, class_names, scores, bboxes, masks, centroids))

    def get_labeled(self, processed_data) -> np.array:
        image, preds, frame, _ = processed_data
        img_labeled = self.cnn.label_image(preds, copy.deepcopy(preds), copy.deepcopy(frame))
        img_labeled = cv2.cvtColor(img_labeled, cv2.COLOR_BGR2RGB)
        return img_labeled

    def get_bboxes(self, processed_data) -> Tuple[np.array, np.array, np.array, np.array]:
        image, preds, frame,(
            classes,
            class_names,
            scores,
            bboxes,
            masks,
            centroids,
        ) = processed_data

        classes = classes.astype(int).tolist()
        scores = scores.astype(float).tolist()

        return bboxes, classes, class_names, scores

    def get_masks(self, processed_data) -> Tuple[np.array, np.array, np.array, np.array, np.array]:
        image, preds, frame,(
            classes,
            class_names,
            scores,
            bboxes,
            masks,
            centroids,
        ) = processed_data

        classes = classes.astype(int).tolist()
        scores = scores.astype(float).tolist()
        object_ids = list(range(0, len(classes)))

        return masks, object_ids, classes, class_names, scores
