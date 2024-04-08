from abc import ABC, abstractmethod
from typing import Tuple

import numpy as np


class AbstractDetector(ABC):
    """
    Implement this class to implement another detector
    Flow:

         image
         │
         │
         │
         ▼
   ┌───────────────────────────────────────────────────────────────┐
   │infer()                                                        │
   │                                                               │
   │n_detections, processed_data                                   │
   └─────┬──────────────┬──────────────────────────────────────────┘
         │              │
         │              │       ┌────────────────┐
         │              ├──────►│get_labeled()   ├───────►labeled image
         │              │       └────────────────┘
         │              │
         │              │       ┌────────────────┐
         │              ├──────►│get_bboxes()    ├───────►bboxes
         │              │       └────────────────┘
         │              │
         │              │       ┌────────────────┐
         │              └──────►│get_masks()     ├───────►masks
         │                      └────────────────┘
         ▼
    """

    def __init__(self):
        super().__init__()

    @abstractmethod
    def infer(self, image: np.array) -> Tuple[int, object]:
        """
        Do the main processing, return tuple of

            1) Number of detected images
            2) Object holding the data

        :param image: ( W x H x 3 ) sized image from cameras
        :return: number of detections, object holding all the processed data
        """
        pass

    @abstractmethod
    def get_labeled(self, processed_data) -> np.array:
        """
        :param processed_data: object
        :return: Original image, with masks drawn over it
        """
        pass

    @abstractmethod
    def get_bboxes(self, processed_data) -> Tuple[np.array, np.array, np.array, np.array]:
        """
        :param processed_data: object
        :return: Tuple of
            1) bboxes:
            2) classes:
            3) class_names:
            4) scores:
        """
        pass

    @abstractmethod
    def get_masks(self, processed_data) -> Tuple[np.array, np.array, np.array, np.array, np.array]:
        """
        :param processed_data: object
        :return: Tuple of
            1) masks:
            2) object_ids:
            3) classes:
            4) class_names:
            5) scores:
        """
        pass
