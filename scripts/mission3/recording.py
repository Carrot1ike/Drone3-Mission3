"""인식 입력을 무손실로 보존한다. 저장 코덱은 센서·전송 사양이 아니다."""
import hashlib
from pathlib import Path
import cv2
import numpy as np


def frame_sha256(gray):
    if gray.shape != (320,320) or gray.dtype != np.uint8:
        raise ValueError('recording requires native320x320 mono8, no conversion/resizing')
    return hashlib.sha256(np.ascontiguousarray(gray).tobytes()).hexdigest()


def open_lossless_writer(path, fps):
    writer=cv2.VideoWriter(str(Path(path)),cv2.VideoWriter_fourcc(*'FFV1'),fps,(320,320),isColor=False)
    if not writer.isOpened():raise RuntimeError('FFV1 native mono lossless recording unavailable')
    return writer
